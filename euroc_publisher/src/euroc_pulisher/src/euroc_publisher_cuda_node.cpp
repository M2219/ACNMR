#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "rosbag2_cpp/reader.hpp"
#include <signal.h>
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <cuda_runtime_api.h>
#include <vector>
#include <string>
#include <filesystem>
#include <chrono>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include "tf2_ros/transform_broadcaster.h"

#include <opencv2/ximgproc/edge_filter.hpp>

std::mutex queue_mutex;
std::condition_variable queue_cv;
bool shutdown_flag = false;

namespace fs = std::filesystem;
using namespace std::chrono;

int net_input_height_ = 512;
int net_input_width_ = 768;
int pad_right;
int pad_bottom;
double max_disp = 96;
cv::Mat disp_filtered;
float alpha = 0.4;  // Adjust for responsiveness vs. smoothness
bool record_video = false;  // Set to false to disable recording
cv::VideoWriter video_writer;

//std::string model_path_ = "/tmp/model_ghustereo8_nce_euroc.plan";
std::string model_path_ = "/tmp/model_stereoacc_euroc.plan";

nvinfer1::ICudaEngine* engine_{nullptr};
nvinfer1::IExecutionContext* context_{nullptr};
void* buffers_[3]{nullptr, nullptr, nullptr};
cudaStream_t stream_;
int leftIndex_, rightIndex_, outputIndex_;
size_t inputSize_, outputSize_;

class Logger : public nvinfer1::ILogger
{
    void log(Severity severity, const char* msg) noexcept override
    {
        if (severity <= Severity::kWARNING)
        {
            std::cout << "[TensorRT] " << msg << std::endl;
        }
    }
};

static Logger gLogger;

float* preprocess_image(const cv::Mat& img) {
    int w = img.cols;
    int h = img.rows;
    int m = 32;

    // Calculate padded dimensions
    int wi = (w / m + 1) * m;
    int hi = (h / m + 1) * m;
    pad_right = wi - w;
    pad_bottom = hi - h;

    // Pad the image (single channel input assumed)
    cv::Mat padded_img;
    cv::copyMakeBorder(img, padded_img, 0, pad_bottom, 0, pad_right, cv::BORDER_CONSTANT, cv::Scalar(0));

    // Convert to 3-channel RGB
    cv::Mat img_rgb;
    cv::cvtColor(padded_img, img_rgb, cv::COLOR_GRAY2RGB);

    // Convert to float and normalize to [0, 1]
    img_rgb.convertTo(img_rgb, CV_32FC3, 1.0 / 255.0);

    // Split channels
    std::vector<cv::Mat> channels(3);
    cv::split(img_rgb, channels);

    // Mean and std (same as PyTorch)
    float mean_vals[3] = {0.485f, 0.456f, 0.406f};
    float std_vals[3]  = {0.229f, 0.224f, 0.225f};

    // Normalize each channel
    for (int c = 0; c < 3; ++c) {
        channels[c] = (channels[c] - mean_vals[c]) / std_vals[c];
    }

    // Allocate CHW float buffer
    int size = 3 * img_rgb.rows * img_rgb.cols;
    float* chw = new float[size];

    // Fill in CHW order
    int idx = 0;
    for (int c = 0; c < 3; ++c) {
        for (int h = 0; h < img_rgb.rows; ++h) {
            for (int w = 0; w < img_rgb.cols; ++w) {
                chw[idx++] = channels[c].at<float>(h, w);
            }
        }
    }

    return chw;
}

nvinfer1::ICudaEngine* loadEngine(const std::string& engineFile) {
    std::ifstream engineFileStream(engineFile, std::ios::binary);
    if (!engineFileStream) {
        std::cerr << "Error opening engine file: " << engineFile << std::endl;
        return nullptr;
    }

    engineFileStream.seekg(0, std::ios::end);
    size_t size = engineFileStream.tellg();
    engineFileStream.seekg(0, std::ios::beg);

    std::vector<char> engineData(size);
    engineFileStream.read(engineData.data(), size);
    engineFileStream.close();

    static Logger logger;
    nvinfer1::IRuntime* runtime = nvinfer1::createInferRuntime(logger);

    if (!runtime) {
        std::cerr << "Error creating TensorRT runtime" << std::endl;
        return nullptr;
    }

    nvinfer1::ICudaEngine* engine = runtime->deserializeCudaEngine(engineData.data(), size);
    delete runtime;

    if (!engine) {
        std::cerr << "Error deserializing engine" << std::endl;
        return nullptr;
    }

    return engine;
}

bool initializeTensorRT() {
    engine_ = loadEngine(model_path_);
    if (!engine_) {
         std::cerr << "Error loading engine" << std::endl;
    }

    context_ = engine_->createExecutionContext();

    // Set up stream
    cudaStreamCreate(&stream_);

    // Input/output dims
    inputSize_ = 1 * 3 * net_input_height_ * net_input_width_ * sizeof(float);
    outputSize_ = 1 * net_input_height_ * net_input_width_ * sizeof(float);

    std::vector<std::string> leftNames  = {"input1", "input_left", "left", "input_left:0", "input_1"};
    std::vector<std::string> rightNames = {"input2", "input_right", "right", "input_right:0", "input_2"};
    std::vector<std::string> outputNames = {"output", "disp", "output_0", "output:0"};

    leftIndex_ = -1;
    rightIndex_ = -1;
    outputIndex_ = -1;

    // Find tensor indices
    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
        const char* name = engine_->getIOTensorName(i);

        for (const auto& leftName : leftNames)
            if (strcmp(name, leftName.c_str()) == 0) leftIndex_ = i;

        for (const auto& rightName : rightNames)
            if (strcmp(name, rightName.c_str()) == 0) rightIndex_ = i;

        for (const auto& outputName : outputNames)
            if (strcmp(name, outputName.c_str()) == 0) outputIndex_ = i;
    }

    // Set shapes
    nvinfer1::Dims4 inputDims = {1, 3, net_input_height_, net_input_width_};
    context_->setInputShape(engine_->getIOTensorName(leftIndex_), inputDims);
    context_->setInputShape(engine_->getIOTensorName(rightIndex_), inputDims);

    // Set tensor addresses
    // Allocate buffers
    cudaMalloc(&buffers_[leftIndex_], inputSize_);
    cudaMalloc(&buffers_[rightIndex_], inputSize_);
    cudaMalloc(&buffers_[outputIndex_], outputSize_);

    return true;
}

void setupStereoRectification(cv::Size image_size,
                             cv::Mat& R1, cv::Mat& R2,
                             cv::Mat& P1, cv::Mat& P2,
                             cv::Mat& Q,
                             cv::Mat& map11, cv::Mat& map12,
                             cv::Mat& map21, cv::Mat& map22) {
    cv::Mat K1 = (cv::Mat_<double>(3,3) <<
        458.654, 0, 367.215,
        0, 457.296, 248.375,
        0, 0, 1);

    cv::Mat D1 = (cv::Mat_<double>(4,1) <<
        -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);

    cv::Mat K2 = (cv::Mat_<double>(3,3) <<
        457.587, 0, 379.999,
        0, 456.134, 255.238,
        0, 0, 1);

    cv::Mat D2 = (cv::Mat_<double>(4,1) <<
        -0.28368365, 0.07451284, -0.00010473, -3.55590700e-05);

    cv::Mat T = (cv::Mat_<double>(4,4) <<
        0.999997256477797, 0.002317135723275, 0.000343393120620, -0.110074137800478,
        -0.002312067192432, 0.999898048507103, 0.014090668452683, 0.000156612054392,
        -0.000376008102320,-0.014089835846691, 0.999900662638081, -0.000889382785432,
        0, 0, 0, 1.000000000000000);

    cv::Mat R = T(cv::Rect(0,0,3,3)).clone();
    cv::Mat T_vec = (cv::Mat_<double>(3,1) << T.at<double>(0,3),
                                             T.at<double>(1,3),
                                             T.at<double>(2,3));
    cv::stereoRectify(K1, D1, K2, D2, image_size, R, T_vec,
                     R1, R2, P1, P2, Q,
                     cv::CALIB_ZERO_DISPARITY, 0, image_size);
    cv::initUndistortRectifyMap(K1, D1, R1, P1, image_size, CV_16SC2, map11, map12);
    cv::initUndistortRectifyMap(K2, D2, R2, P2, image_size, CV_16SC2, map21, map22);
}

void rectifyStereoPair(const cv::Mat& left_img, const cv::Mat& right_img,
                      cv::Mat& left_rect, cv::Mat& right_rect,
                      const cv::Mat& map11, const cv::Mat& map12,
                      const cv::Mat& map21, const cv::Mat& map22) {
    cv::remap(left_img, left_rect, map11, map12, cv::INTER_LINEAR);
    cv::remap(right_img, right_rect, map21, map22, cv::INTER_LINEAR);
}


// Function to apply histogram matching from source to template image
cv::Mat matchHistogram(const cv::Mat& src, const cv::Mat& tmpl) {
    CV_Assert(src.type() == CV_8UC1 && tmpl.type() == CV_8UC1);

    // Compute histograms and cumulative histograms
    int histSize = 256;
    float range[] = { 0, 256 }; 
    const float* histRange = { range };

    cv::Mat src_hist, tmpl_hist;
    cv::calcHist(&src, 1, 0, cv::Mat(), src_hist, 1, &histSize, &histRange);
    cv::calcHist(&tmpl, 1, 0, cv::Mat(), tmpl_hist, 1, &histSize, &histRange);

    // Normalize histograms
    cv::normalize(src_hist, src_hist, 1, 0, cv::NORM_L1);
    cv::normalize(tmpl_hist, tmpl_hist, 1, 0, cv::NORM_L1);

    // Compute cumulative distribution functions (CDF)
    cv::Mat src_cdf, tmpl_cdf;
    src_cdf = src_hist.clone();
    tmpl_cdf = tmpl_hist.clone();
    for (int i = 1; i < histSize; ++i) {
        src_cdf.at<float>(i) += src_cdf.at<float>(i - 1);
        tmpl_cdf.at<float>(i) += tmpl_cdf.at<float>(i - 1);
    }

    // Create a lookup table (LUT) for pixel value mapping
    uchar lut[256];
    for (int i = 0; i < histSize; ++i) {
        float val = src_cdf.at<float>(i);
        uchar j = 0;
        while (j < 255 && tmpl_cdf.at<float>(j) < val) j++;
        lut[i] = j;
    }

    // Apply LUT to source image
    cv::Mat result = src.clone();
    for (int y = 0; y < src.rows; ++y)
        for (int x = 0; x < src.cols; ++x)
            result.at<uchar>(y, x) = lut[src.at<uchar>(y, x)];

    return result;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("stereo_processor");
    node->declare_parameter<std::string>("bag_path", "./rosbag2.db3");
    std::string bag_path = node->get_parameter("bag_path").as_string();

    rosbag2_cpp::Reader reader_;
    reader_.open(bag_path);

    auto disparity_pub = node->create_publisher<sensor_msgs::msg::Image>("/disparity/image_raw", 10);
    auto left_rect_pub = node->create_publisher<sensor_msgs::msg::Image>("/cam0/image_raw", 10);
    auto right_rect_pub = node->create_publisher<sensor_msgs::msg::Image>("/cam1/image_raw", 10);
    auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("/imu0", 10);
    auto tf_pub = node->create_publisher<geometry_msgs::msg::TransformStamped>("/gt_tf", 10);

    if (!left_rect_pub || !right_rect_pub || !disparity_pub || !imu_pub) {
        RCLCPP_ERROR(node->get_logger(), "Failed to create publishers!");
        rclcpp::shutdown();
        return -1;
    }

    if (!initializeTensorRT()) {
        std::cerr << "TensorRT initialization failed!" << std::endl;
        return 1;
    }

    cv::Size image_size(752, 480);
    cv::Mat R1, R2, P1, P2, Q;
    cv::Mat map11, map12, map21, map22;
    setupStereoRectification(image_size, R1, R2, P1, P2, Q,
                           map11, map12, map21, map22);

    rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization;
    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
    rclcpp::Serialization<geometry_msgs::msg::TransformStamped> tf_serialization;

    std::shared_ptr<sensor_msgs::msg::Imu> imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
    std::shared_ptr<sensor_msgs::msg::Image> left = std::make_shared<sensor_msgs::msg::Image>();
    std::shared_ptr<sensor_msgs::msg::Image> right = std::make_shared<sensor_msgs::msg::Image>();
    std::shared_ptr<sensor_msgs::msg::Image> left_msg_rect = std::make_shared<sensor_msgs::msg::Image>();
    std::shared_ptr<sensor_msgs::msg::Image> right_msg_rect = std::make_shared<sensor_msgs::msg::Image>();
    std::shared_ptr<sensor_msgs::msg::Image> disp_msg = std::make_shared<sensor_msgs::msg::Image>();
    std::shared_ptr<geometry_msgs::msg::TransformStamped> tf_msg = std::make_shared<geometry_msgs::msg::TransformStamped>();

    auto left_msg = std::make_shared<sensor_msgs::msg::Image>();
    auto right_msg = std::make_shared<sensor_msgs::msg::Image>();

    cv::Mat disp_filtered_16;
    cv::Mat left_img, right_img;
    cv::Mat left_raw, right_raw;

    std_msgs::msg::Header header_l;
    std_msgs::msg::Header header_r;

    bool stop_requested = false;

    rclcpp::Time last_msg_time;
    bool first_msg = true;

    while (reader_.has_next() && !stop_requested && rclcpp::ok()) {

        auto start = high_resolution_clock::now();
        auto bag_msg = reader_.read_next();

        rclcpp::Time current_msg_time(bag_msg->time_stamp, RCL_ROS_TIME);

        if (!first_msg) {
            auto dt = current_msg_time - last_msg_time;
            if (dt.nanoseconds() > 0) {
                rclcpp::sleep_for(std::chrono::nanoseconds(dt.nanoseconds()));
            }
        } else {
            first_msg = false;
        }

        last_msg_time = current_msg_time;

        rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);

        if (bag_msg->topic_name == "/vicon/firefly_sbx/firefly_sbx") {
            tf_serialization.deserialize_message(&serialized_msg, tf_msg.get());
            tf_pub->publish(*tf_msg);
        }
        if (bag_msg->topic_name == "/cam0/image_raw") {
            image_serialization.deserialize_message(&serialized_msg, left_msg.get());
            left_rect_pub->publish(*left_msg);
            left = left_msg;
        }
        else if (bag_msg->topic_name == "/cam1/image_raw") {
            image_serialization.deserialize_message(&serialized_msg, right_msg.get());
            right_rect_pub->publish(*right_msg);
            right = right_msg;
        }
        else if (bag_msg->topic_name == "/imu0") {
            imu_serialization.deserialize_message(&serialized_msg, imu_msg.get());
            imu_pub->publish(*imu_msg);
        }

        if (left->data.empty() || right->data.empty()) continue;

        left_raw = cv_bridge::toCvCopy(left, "mono8")->image;
        right_raw = cv_bridge::toCvCopy(right, "mono8")->image;

        int original_height = left_raw.rows;
        int original_width = left_raw.cols;

        rectifyStereoPair(left_raw, right_raw, left_img, right_img, map11, map12, map21, map22);

        header_l.stamp = left->header.stamp;
        header_l.frame_id = "euroc_stereo_frame";

        header_r.stamp = right->header.stamp;
        header_r.frame_id = "euroc_stereo_frame";

        //left_msg_rect = cv_bridge::CvImage(header_l, "mono8", left_img).toImageMsg();
        //right_msg_rect = cv_bridge::CvImage(header_r, "mono8", right_img).toImageMsg();
        //left_rect_pub->publish(*left_msg_rect);
        //right_rect_pub->publish(*right_msg_rect);

        right_img = matchHistogram(right_img, left_img);

        // Run stereo inference
        float* outputData = new float[1 * net_input_height_ * net_input_width_];
        float* inputLeft = preprocess_image(left_img);
        float* inputRight = preprocess_image(right_img);

        // Copy input data to device
        cudaMemcpyAsync(buffers_[leftIndex_], inputLeft, inputSize_, cudaMemcpyHostToDevice, stream_);
        cudaMemcpyAsync(buffers_[rightIndex_], inputRight, inputSize_, cudaMemcpyHostToDevice, stream_);

        context_->setTensorAddress(engine_->getIOTensorName(leftIndex_), buffers_[leftIndex_]);
        context_->setTensorAddress(engine_->getIOTensorName(rightIndex_), buffers_[rightIndex_]);
        context_->setTensorAddress(engine_->getIOTensorName(outputIndex_), buffers_[outputIndex_]);

        // Run inference
        if (!context_->enqueueV3(stream_)) {
            std::cerr << "Inference failed\n";
        }

        // Copy output back to host
        cudaMemcpyAsync(outputData, buffers_[outputIndex_], outputSize_, cudaMemcpyDeviceToHost, stream_);
        cudaStreamSynchronize(stream_);

        // Convert and display
        cv::Mat disp_mat(net_input_height_, net_input_width_, CV_32FC1, outputData);

        // Crop the disparity cv::Mat to remove padding
        if (pad_bottom > 0 || pad_right > 0) {
            disp_mat = disp_mat(cv::Rect(0, 0, original_width, original_height));
        }

        // 1. Spatial smoothing
        cv::medianBlur(disp_mat, disp_filtered, 5);

        // 2. Temporal smoothing
        //static cv::Mat prev_disp;
        //if (prev_disp.empty()) prev_disp = disp_filtered.clone();
        //cv::addWeighted(disp_filtered, alpha, prev_disp, 1.0 - alpha, 0, disp_filtered);
        //prev_disp = disp_filtered.clone();

        cv::Mat valid_mask = (disp_filtered > 0) & (disp_filtered < max_disp);

        disp_filtered.setTo(0, ~valid_mask);
        disp_filtered.convertTo(disp_filtered_16, CV_16UC1, 256.0);

        cv::Mat disp_norm, disp_color;

        double max_val, min_val;
        cv::minMaxLoc(disp_filtered_16, &min_val, &max_val, nullptr, nullptr, valid_mask);

        // Step 2: Normalize (bright = close)
        disp_filtered_16.convertTo(disp_norm, CV_8UC1, -255.0 / (max_val - min_val), 255.0 * max_val / (max_val - min_val));

        // Step 3: Apply perceptually uniform colormap
        cv::applyColorMap(disp_norm, disp_color, cv::COLORMAP_MAGMA);

        //cv::applyColorMap(disp_norm, disp_color, cv::COLORMAP_JET);
        cv::Mat left_color;
        if (left_img.channels() == 1) {
            cv::cvtColor(left_img, left_color, cv::COLOR_GRAY2BGR);
        } else {
            left_color = left_img.clone();
        }

        // Resize if needed to match heights (optional, if they mismatch due to processing)
        if (left_color.size() != disp_color.size()) {
            cv::resize(left_color, left_color, disp_color.size());
        }

        cv::Mat combined;

        cv::hconcat(left_color, disp_color, combined);
        cv::imshow("Left + Disparity", combined);
        cv::waitKey(1);
        if (record_video && !video_writer.isOpened()) {
            int fps = 30;
            std::string output_path = "disparity_output.mp4";
            int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
            cv::Size frame_size(combined.cols, combined.rows);
            video_writer.open(output_path, fourcc, fps, frame_size);
        }

        // Write frame to video (if recording is enabled)
        if (record_video && video_writer.isOpened()) {
            video_writer.write(combined);
        }
        std::cout << "Original Image Size: " << left_img.cols << " x " << left_img.rows << std::endl;
        std::cout << "Disparity Size: " << disp_color.cols << " x " << disp_color.rows << std::endl;

        delete[] inputLeft;
        delete[] inputRight;
        delete[] outputData;


        disp_msg = cv_bridge::CvImage(header_l, "16UC1", disp_filtered_16).toImageMsg();
        disparity_pub->publish(*disp_msg);


        auto end = high_resolution_clock::now();
        double elapsed_ms = duration<double, std::milli>(end - start).count();
        std::cout << "Elapsed time: " << elapsed_ms << " ms" << std::endl;
        left = std::make_shared<sensor_msgs::msg::Image>();
        right = std::make_shared<sensor_msgs::msg::Image>();
    }

    reader_.close();
    if (context_) delete context_;
    if (engine_) delete engine_;
    for (int i = 0; i < 3; ++i) if (buffers_[i]) cudaFree(buffers_[i]);
    node.reset();
    rcutils_reset_error();
    left_rect_pub.reset();
    right_rect_pub.reset();
    disparity_pub.reset();
    imu_pub.reset();
    stop_requested = true;

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

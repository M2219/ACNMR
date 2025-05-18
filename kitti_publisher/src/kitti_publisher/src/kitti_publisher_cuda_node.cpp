#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

std::mutex queue_mutex;
std::condition_variable queue_cv;
bool shutdown_flag = false;

namespace fs = std::filesystem;
using namespace std::chrono;

int net_input_height_ = 384;
int net_input_width_ = 1248;
int pad_right;
int pad_bottom;
double max_disp = 96;
cv::Mat disp_filtered;
float alpha = 0.4;  // Adjust for responsiveness vs. smoothness
bool record_video = false;  // Set to false to disable recording
cv::VideoWriter video_writer;

//std::string model_path_ = "/tmp/model_ghustereo8_nce_kitti.plan";
std::string model_path_ = "/tmp/model_stereoacc_kitti.plan";

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


class KittiImagePublisher : public rclcpp::Node {
public:
    KittiImagePublisher() : Node("kitti_image_publisher"), current_index_(0) {
        RCLCPP_INFO(this->get_logger(), "Image Publisher Node Started!");

        // Parameters
        this->declare_parameter<std::string>("kitti_path", "./10");
        kitti_path = this->get_parameter("kitti_path").as_string();

        fps_ = 20;
        left_dir_ = kitti_path + "/image_2";
        right_dir_ = kitti_path + "/image_3";

        if (!fs::exists(left_dir_) || !fs::exists(right_dir_)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid KITTI dataset path: %s", kitti_path.c_str());
            throw std::runtime_error("KITTI dataset directories not found!");
        }

        for (const auto& entry : fs::directory_iterator(left_dir_)) {
            left_images_.push_back(entry.path().string());
        }
        for (const auto& entry : fs::directory_iterator(right_dir_)) {
            right_images_.push_back(entry.path().string());
        }
        std::sort(left_images_.begin(), left_images_.end());
        std::sort(right_images_.begin(), right_images_.end());

        if (left_images_.size() != right_images_.size()) {
            RCLCPP_ERROR(this->get_logger(), "Mismatch in number of images between left and right cameras.");
            throw std::runtime_error("Left and right image counts do not match!");
        }

        left_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/cam0/image_raw", 10);
        right_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/cam1/image_raw", 10);
        disparity_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/disparity/image_raw", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / fps_),
            std::bind(&KittiImagePublisher::publishImages, this)
        );

        if (!initializeTensorRT()) {
            RCLCPP_ERROR(this->get_logger(), "TensorRT initialization failed!");
            rclcpp::shutdown();
        }
    }

private:
    void publishImages() {

        auto start = high_resolution_clock::now();

        if (current_index_ >= left_images_.size()) {
            current_index_ = 0;
        }

        cv::Mat left_img = cv::imread(left_images_[current_index_], cv::IMREAD_GRAYSCALE);
        cv::Mat right_img = cv::imread(right_images_[current_index_], cv::IMREAD_GRAYSCALE);

        int original_height = left_img.rows;
        int original_width = left_img.cols;

        if (left_img.empty() || right_img.empty()) {
            RCLCPP_WARN(this->get_logger(), "Failed to read images at index %d", current_index_);
            return;
        }

        rclcpp::Time current_time = this->get_clock()->now();

        // Convert to ROS Image messages
        auto left_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", left_img).toImageMsg();
        auto right_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", right_img).toImageMsg();

        left_msg->header.stamp = current_time;
        right_msg->header.stamp = current_time;

        left_msg->header.frame_id = "left_camera";
        right_msg->header.frame_id = "right_camera";

        left_pub_->publish(*left_msg);
        right_pub_->publish(*right_msg);

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

        cv::vconcat(left_color, disp_color, combined);
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


        auto disp_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", disp_filtered_16).toImageMsg();

        disp_msg->header.stamp = current_time;
        disp_msg->header.frame_id = "left_camera";

        disparity_pub_->publish(*disp_msg);


        auto end = high_resolution_clock::now();
        double elapsed_ms = duration<double, std::milli>(end - start).count();
        std::cout << "Elapsed time: " << elapsed_ms << " ms" << std::endl;

        current_index_++;
    }


    std::string kitti_path;
    std::string left_dir_;
    std::string right_dir_;
    std::vector<std::string> left_images_;
    std::vector<std::string> right_images_;
    cv::Mat disp_filtered;
    cv::Mat disp_filtered_16;
    size_t current_index_;
    int fps_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<KittiImagePublisher>();
    rclcpp::spin(node);

    if (context_) delete context_;
    if (engine_) delete engine_;
    for (int i = 0; i < 3; ++i) if (buffers_[i]) cudaFree(buffers_[i]);
    rclcpp::shutdown();
    return 0;
}




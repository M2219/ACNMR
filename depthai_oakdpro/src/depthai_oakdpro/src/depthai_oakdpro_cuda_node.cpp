#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "sensor_msgs/msg/imu.hpp"

#include "depthai/depthai.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"

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
#include <iomanip>

std::mutex queue_mutex;
std::condition_variable queue_cv;
bool shutdown_flag = false;

namespace fs = std::filesystem;
using namespace std::chrono;

int net_input_height_ = 416;
int net_input_width_ = 672;
int pad_right;
int pad_bottom;
double max_disp = 96;
cv::Mat disp_filtered;
float alpha = 0.5;  // Adjust for responsiveness vs. smoothness
bool record_video = false;  // Set to false to disable recording
cv::VideoWriter video_writer;
std::string model_path_ = "/tmp/model_ghustereo8_nce_f32.plan";

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

void visualize_and_record_disparity(
    const cv::Mat& disparity,
    const cv::Mat& disp_filtered_16,
    const cv::Mat& left_img,
    const cv::Mat& valid_mask,
    bool record_video,
    double elapsed_ms,
    cv::VideoWriter& video_writer
) {

    double fx = 809.4182764202308 / 2;
    double baseline = 0.07505134045288388;  // from T_cn_cnm1[0][3], absolute value

    // --- Calculate center pixel ---
    int center_x = disparity.cols / 2;
    int center_y = disparity.rows / 2;

    // --- Disparity value (assuming CV_16UC1 and scaled by 256) ---
    float disp_val = disparity.at<float>(center_y, center_x);

    std::string depth_text;
    if (disp_val > 0.0) {
        double depth = (fx * baseline) / disp_val;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << depth << " m";
        depth_text = oss.str();
    } else {
        depth_text = "N/A";
    }

    // Normalize disparity
    double max_val, min_val;
    cv::minMaxLoc(disp_filtered_16, &min_val, &max_val, nullptr, nullptr, valid_mask);
    std::cout << "Disparity range: [" << min_val << ", " << max_val << "]" << std::endl;
    cv::Mat disp_norm, disp_color;

    disp_filtered_16.convertTo(disp_norm, CV_8UC1, -255.0 / (max_val - min_val), 255.0 * max_val / (max_val - min_val));
    cv::applyColorMap(disp_norm, disp_color, cv::COLORMAP_MAGMA);

    // Convert grayscale left image to BGR if needed
    cv::Mat left_color;
    if (left_img.channels() == 1) {
        cv::cvtColor(left_img, left_color, cv::COLOR_GRAY2BGR);
    } else {
        left_color = left_img.clone();
    }

    // Match dimensions if needed
    if (left_color.size() != disp_color.size()) {
        cv::resize(left_color, left_color, disp_color.size());
    }

    // Concatenate images horizontally
    cv::circle(disp_color, cv::Point(center_x, center_y), 5, cv::Scalar(0, 0, 255), -1);
    cv::putText(disp_color, depth_text, cv::Point(center_x + 10, center_y - 10), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);

    cv::Mat combined;
    cv::hconcat(left_color, disp_color, combined);

    // Elapsed time annotation (FPS)
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << 1000.0 / elapsed_ms << " HZ";
    std::string text = oss.str();

    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 1.0;
    int thickness = 4;
    cv::Scalar text_color(0, 255, 0);  // Green
    int baseline_2 = 0;
    cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline_2);
    cv::Point text_org(combined.cols - text_size.width - 10, text_size.height + 10);
    cv::putText(combined, text, text_org, font_face, font_scale, text_color, thickness);

    // Show in window
    cv::imshow("Left + Disparity", combined);
    cv::waitKey(1);

    // Write to video file
    if (record_video && !video_writer.isOpened()) {
        std::string output_path = "disparity_output.mp4";
        int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        int fps = 30;
        cv::Size frame_size(combined.cols, combined.rows);
        video_writer.open(output_path, fourcc, fps, frame_size);
    }

    if (record_video && video_writer.isOpened()) {
        video_writer.write(combined);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("depthai_oakdpro_cuda_node");

    auto disparity_pub = node->create_publisher<sensor_msgs::msg::Image>("/disparity/image_raw", 10);
    auto left_rect_pub = node->create_publisher<sensor_msgs::msg::Image>("/left/image_rect", 10);
    auto right_rect_pub = node->create_publisher<sensor_msgs::msg::Image>("/right/image_rect", 10);

    if (!initializeTensorRT()) {
        std::cerr << "TensorRT initialization failed!" << std::endl;
        return 1;
    }

    // Initialize pipeline
    std::shared_ptr<dai::Pipeline> pipeline;
    pipeline = std::make_shared<dai::Pipeline>();

    // IMU
    auto imu = pipeline->create<dai::node::IMU>();
    auto xoutImu = pipeline->create<dai::node::XLinkOut>();
    xoutImu->setStreamName("imu");
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 200);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 200);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(1);  // Get one message only for now.
    imu->out.link(xoutImu->input);

    // Create mono cameras
    auto camLeft = pipeline->create<dai::node::MonoCamera>();
    auto camRight = pipeline->create<dai::node::MonoCamera>();
    auto stereo = pipeline->create<dai::node::StereoDepth>();

    auto xoutLeft = pipeline->create<dai::node::XLinkOut>();
    auto xoutRight = pipeline->create<dai::node::XLinkOut>();

    auto xoutRectifL = pipeline->create<dai::node::XLinkOut>();
    auto xoutRectifR = pipeline->create<dai::node::XLinkOut>();

    auto controlIn = pipeline->create<dai::node::XLinkIn>();


    controlIn->setStreamName("control");
    controlIn->out.link(camRight->inputControl);
    controlIn->out.link(camLeft->inputControl);


    // Set camera properties
    camLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    camLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    camLeft->setFps(20.0);
    //camLeft.setSyncMode(True)

    camRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    camRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    camRight->setFps(20.0);
    //camRight.setSyncMode(True)

    stereo->setRectifyEdgeFillColor(0);
    camLeft->out.link(stereo->left);
    camRight->out.link(stereo->right);
    stereo->syncedLeft.link(xoutLeft->input);
    stereo->syncedRight.link(xoutRight->input);

    // Set XLinkOut stream names
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");

    xoutRectifL->setStreamName("rectified_left");
    xoutRectifR->setStreamName("rectified_right");

    stereo->rectifiedLeft.link(xoutRectifL->input);
    stereo->rectifiedRight.link(xoutRectifR->input);

    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>(*pipeline);
    auto calibrationHandler = device->readCalibration();

    auto controlQueue = device->getInputQueue("control");
    // Set manual exposure
    dai::CameraControl ctrl;
    ctrl.setManualExposure(2000, 100);
    ctrl.setAutoExposureLock(true);
    ctrl.setManualWhiteBalance(5500);
    ctrl.setAutoWhiteBalanceLock(true);
    ctrl.setManualFocus(128);
    ctrl.setAntiBandingMode(dai::CameraControl::AntiBandingMode::MAINS_50_HZ);
    ctrl.setLumaDenoise(2);
    ctrl.setChromaDenoise(2);
    ctrl.setSharpness(1);
    controlQueue->send(ctrl);
    //ctrl.setContrast(1);
    //ctrl.setBrightness(0);
    //ctrl.setSaturation(0);


    auto leftQueue = device->getOutputQueue("left", 30, false);
    auto rightQueue = device->getOutputQueue("right", 30, false);

    auto rectifLeftQueue = device->getOutputQueue("rectified_left", 30, false);
    auto rectifRightQueue = device->getOutputQueue("rectified_right", 30, false);

    // Image converters
    dai::rosBridge::ImageConverter imageConverterLeft("oak_left_camera_optical_frame", true);
    //imageConverterLeft.setUpdateRosBaseTimeOnToRosMsg(true);

    dai::rosBridge::ImageConverter imageConverterRight("oak_right_camera_optical_frame", true);
    //imageConverterRight.setUpdateRosBaseTimeOnToRosMsg(true);

    dai::rosBridge::ImageConverter imageConverterLeftRect("oak_left_camera_optical_frame", true);
    //imageConverterLeftRect.setUpdateRosBaseTimeOnToRosMsg(true);

    dai::rosBridge::ImageConverter imageConverterRightRect("oak_right_camera_optical_frame", true);
    //imageConverterRightRect.setUpdateRosBaseTimeOnToRosMsg(true);


    const std::string leftPubName =  std::string("left/image_raw");
    const std::string rightPubName = std::string("right/image_raw");

    const std::string leftRectPubName =  std::string("left/image_rect");
    const std::string rightRectPubName = std::string("right/image_rect");

    int width = 1280;
    int height = 720;
    auto leftCameraInfo = imageConverterLeft.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_B, width, height);
    auto rightCameraInfo = imageConverterRight.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, width, height);

    // Bridge Publishers
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> leftPublish(
        leftQueue,
        node,
        leftPubName,
        [&](std::shared_ptr<dai::ImgFrame> frame, std::deque<sensor_msgs::msg::Image> &rosMsgs) {
        imageConverterLeft.toRosMsg(frame, rosMsgs);
        if (!rosMsgs.empty()) {
            //rosMsgs.front().header.stamp = node->now();
            rclcpp::Time dai_time = rclcpp::Time(frame->getTimestamp().time_since_epoch().count());
            rosMsgs.front().header.stamp = dai_time;

        }
        },
        30,
        leftCameraInfo,
        "left");

    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightPublish(
        rightQueue,
        node,
        rightPubName,
        [&](std::shared_ptr<dai::ImgFrame> frame, std::deque<sensor_msgs::msg::Image> &rosMsgs) {
        imageConverterRight.toRosMsg(frame, rosMsgs);
        if (!rosMsgs.empty()) {
            //rosMsgs.front().header.stamp = node->now();
            rclcpp::Time dai_time = rclcpp::Time(frame->getTimestamp().time_since_epoch().count());
            rosMsgs.front().header.stamp = dai_time;

        }
        },
        30,
        rightCameraInfo,
        "right");

    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> leftRectPublish(
        rectifLeftQueue,
        node,
        leftRectPubName,
        [&](std::shared_ptr<dai::ImgFrame> frame, std::deque<sensor_msgs::msg::Image> &rosMsgs) {
        imageConverterLeftRect.toRosMsg(frame, rosMsgs);
        if (!rosMsgs.empty()) {
            //rosMsgs.front().header.stamp = node->now();
            rclcpp::Time dai_time = rclcpp::Time(frame->getTimestamp().time_since_epoch().count());
            rosMsgs.front().header.stamp = dai_time;

        }
        },
        30,
        leftCameraInfo,
        "left");

    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightRectPublish(
        rectifRightQueue,
        node,
        rightRectPubName,
        [&](std::shared_ptr<dai::ImgFrame> frame, std::deque<sensor_msgs::msg::Image> &rosMsgs) {
        imageConverterRightRect.toRosMsg(frame, rosMsgs);
        if (!rosMsgs.empty()) {
            //rosMsgs.front().header.stamp = node->now();
            rclcpp::Time dai_time = rclcpp::Time(frame->getTimestamp().time_since_epoch().count());
            rosMsgs.front().header.stamp = dai_time;

        }
        },
        30,
        rightCameraInfo,
        "right");

    //rightPublish.addPublisherCallback();
    //leftPublish.addPublisherCallback();

    //rightRectPublish.addPublisherCallback();
    //leftRectPublish.addPublisherCallback();

    // RGB camera
    auto imuQueue = device->getOutputQueue("imu", 30, false);

    double angularVelCovariance = 0, linearAccelCovariance = 0;
    dai::ros::ImuSyncMethod imuMode = dai::ros::ImuSyncMethod::COPY;
    dai::rosBridge::ImuConverter imuConverter("oak_imu_frame", imuMode, linearAccelCovariance, angularVelCovariance);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData> imuPublish(
        imuQueue,
        node,
        std::string("/oak/imu"),
        [&](std::shared_ptr<dai::IMUData> imuData, std::deque<sensor_msgs::msg::Imu> &rosMsgs) {
        imuConverter.toRosMsg(imuData, rosMsgs);
        if (!rosMsgs.empty()) {
            //rosMsgs.front().header.stamp = node->now();
            rclcpp::Time dai_time = rclcpp::Time(imuData->getTimestamp().time_since_epoch().count());
            rosMsgs.front().header.stamp = dai_time;
        }
        },
        30,
        "",
        "imu");

    imuPublish.addPublisherCallback();

    while (rclcpp::ok()) {
        auto left = rectifLeftQueue->get<dai::ImgFrame>();
        auto right = rectifRightQueue->get<dai::ImgFrame>();

        if (!left || !right) continue;
        if (left->getData().empty() || right->getData().empty()) {
           continue;
         }

        auto start = high_resolution_clock::now();

        cv::Mat left_img = left->getCvFrame();
        cv::Mat right_img = right->getCvFrame();

        // Run stereo inference
        float* outputData = new float[1 * 416 * 672];
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

        int original_height = left_img.rows;
        int original_width = left_img.cols;

        // Crop the disparity cv::Mat to remove padding
        if (pad_bottom > 0 || pad_right > 0) {
            disp_mat = disp_mat(cv::Rect(0, 0, original_width, original_height));
        }

        // 1. Spatial smoothing
        cv::medianBlur(disp_mat, disp_filtered, 5);

        // 2. Temporal smoothing (IIR)
        static cv::Mat prev_disp;
        if (prev_disp.empty()) prev_disp = disp_filtered.clone();
        cv::addWeighted(disp_filtered, alpha, prev_disp, 1.0 - alpha, 0, disp_filtered);
        prev_disp = disp_filtered.clone();

        // 3. Mask invalid pixels
        cv::Mat valid_mask = (disp_filtered > 0) & (disp_filtered < max_disp);
        disp_filtered.setTo(0, ~valid_mask);

        cv::Mat disp_filtered_16;
        disp_filtered.convertTo(disp_filtered_16, CV_16UC1, 256.0);

        auto end = high_resolution_clock::now();
        double elapsed_ms = duration<double, std::milli>(end - start).count();
        std::cout << "Elapsed time: " << elapsed_ms << " ms" << std::endl;


        visualize_and_record_disparity(
            disp_filtered,
            disp_filtered_16,
            left_img,
            valid_mask,
            record_video,
            elapsed_ms,
            video_writer
        );

        std::cout << "Original Image Size: " << left_img.cols << " x " << left_img.rows << std::endl;

        delete[] inputLeft;
        delete[] inputRight;
        delete[] outputData;


        std_msgs::msg::Header header;
        header.stamp = rclcpp::Time(left->getTimestamp().time_since_epoch().count());
        header.frame_id = "oak_stereo_frame";

        sensor_msgs::msg::Image::SharedPtr disp_msg = cv_bridge::CvImage(header, "16UC1", disp_filtered_16).toImageMsg();
        disparity_pub->publish(*disp_msg);

        sensor_msgs::msg::Image::SharedPtr left_msg = cv_bridge::CvImage(header, "mono8", left_img).toImageMsg();
        left_rect_pub->publish(*left_msg);

        sensor_msgs::msg::Image::SharedPtr right_msg = cv_bridge::CvImage(header, "mono8", right_img).toImageMsg();
        right_rect_pub->publish(*right_msg);

        //auto end = high_resolution_clock::now();
        //double elapsed_ms = duration<double, std::milli>(end - start).count();
        //std::cout << "Elapsed time: " << elapsed_ms << " ms" << std::endl;
    }

    if (context_) delete context_;
    if (engine_) delete engine_;
    for (int i = 0; i < 3; ++i) if (buffers_[i]) cudaFree(buffers_[i]);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

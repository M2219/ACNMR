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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/videoio.hpp>

namespace fs = std::filesystem;
using namespace std::chrono;

class StereoNode : public rclcpp::Node {
public:
    StereoNode() : Node("stereo_inference_node") {
        this->declare_parameter<std::string>("model_path", "model_stereoacc.plan");
        this->get_parameter("model_path", model_path_);

        left_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/left/image_rect", 20,
            std::bind(&StereoNode::left_callback, this, std::placeholders::_1));

        right_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/right/image_rect", 20,
            std::bind(&StereoNode::right_callback, this, std::placeholders::_1));

        engine_ = loadEngine(model_path_);
        if (!engine_) {
             std::cerr << "Error loading engine" << std::endl;
        }

        // Create context and CUDA stream once
        context_ = engine_->createExecutionContext();
        cudaStreamCreate(&stream_);

        // Input/output dims
        inputSize_ = 1 * 3 * net_input_height_ * net_input_width_ * sizeof(float);
        outputSize_ = 1 * net_input_height_ * net_input_width_ * sizeof(float);

        // Tensor name matching with flexible naming conventions
        //leftNames = {"input1", "input_left", "left", "input_left:0", "input_1"};
        ///rightNames = {"input2", "input_right", "right", "input_right:0", "input_2"};
        //outputNames = {"output", "disp", "output_0", "output:0"};
        //leftIndex_ = -1, rightIndex_ = -1, outputIndex_ = -1;

        // Tensor name matching with flexible naming conventions
        std::vector<std::string> leftNames  = {"input1", "input_left", "left", "input_left:0", "input_1"};
        std::vector<std::string> rightNames = {"input2", "input_right", "right", "input_right:0", "input_2"};
        std::vector<std::string> outputNames = {"output", "disp", "output_0", "output:0"};

        leftIndex_ = -1;
        rightIndex_ = -1;
        outputIndex_ = -1;

        // Find bindings by flexible name matching
        for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
            const char* name = engine_->getIOTensorName(i);

            if (leftIndex_ == -1) {
                for (const auto& leftName : leftNames) {
                    if (strcmp(name, leftName.c_str()) == 0) {
                        leftIndex_ = i;
                        break;
                    }
                }
            }

            if (rightIndex_ == -1) {
                for (const auto& rightName : rightNames) {
                    if (strcmp(name, rightName.c_str()) == 0) {
                        rightIndex_ = i;
                        break;
                    }
                }
            }

            if (outputIndex_ == -1) {
                for (const auto& outputName : outputNames) {
                    if (strcmp(name, outputName.c_str()) == 0) {
                        outputIndex_ = i;
                        break;
                    }
                }
            }
        }
        // Set tensor shapes (adjust dimensions as needed)
        inputDims = {1, 3, net_input_height_, net_input_width_};
        context_->setInputShape(engine_->getIOTensorName(leftIndex_), inputDims);
        context_->setInputShape(engine_->getIOTensorName(rightIndex_), inputDims);

        // Allocate buffers once
        cudaMalloc(&buffers_[leftIndex_], inputSize_);
        cudaMalloc(&buffers_[rightIndex_], inputSize_);
        cudaMalloc(&buffers_[outputIndex_], outputSize_);

        RCLCPP_INFO(this->get_logger(), "Stereo inference node initialized.");
    }

    ~StereoNode()
    {
        if (context_) delete context_;
        if (engine_) delete engine_;
        for (int i = 0; i < 3; ++i) if (buffers_[i]) cudaFree(buffers_[i]);
    }

private:
    std::string model_path_;
    sensor_msgs::msg::Image::SharedPtr left_img_, right_img_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_, right_sub_;

    nvinfer1::ICudaEngine* engine_{nullptr};
    nvinfer1::IExecutionContext* context_{nullptr};
    void* buffers_[3]{nullptr, nullptr, nullptr};
    cudaStream_t stream_;
    int leftIndex_, rightIndex_, outputIndex_;
    size_t inputSize_, outputSize_;

    //std::vector<std::string> leftNames = {"input1", "input_left", "left", "input_left:0", "input_1"};
    //std::vector<std::string> rightNames = {"input2", "input_right", "right", "input_right:0", "input_2"};
    //std::vector<std::string> outputNames = {"output", "disp", "output_0", "output:0"};

    nvinfer1::Dims4 inputDims;

    int net_input_height_ = 416;
    int net_input_width_ = 672;
    //int net_input_height_ = 384; //kitti
    //int net_input_width_ = 1248; //kitti
    int pad_right;
    int pad_bottom;
    double max_disp = 192;
    cv::Mat disp_filtered;
    float alpha = 0.5;  // Adjust for responsiveness vs. smoothness
    bool record_video = false;  // Set to false to disable recording
    cv::VideoWriter video_writer;
    int frame_counter = 0;

    void left_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        left_img_ = msg;
        process_if_ready();
    }


    void rectifyStereoImages(const cv::Mat& left, const cv::Mat& right,
                             cv::Mat& left_rect, cv::Mat& right_rect) {
        // cam0 intrinsics
        cv::Mat K1 = (cv::Mat_<double>(3, 3) << 809.4182764202308, 0.0, 647.6422542510353,
                                                0.0, 805.6864499913798, 362.1203344621752,
                                                0.0, 0.0, 1.0);
        cv::Mat D1 = (cv::Mat_<double>(4, 1) << -0.0043279930076504936, -0.0354540987944918,
                                                0.002101823987952484, 0.00012060718821675076);

        // cam1 intrinsics
        cv::Mat K2 = (cv::Mat_<double>(3, 3) << 806.279611933925, 0.0, 647.781267496406,
                                                0.0, 803.1187646522317, 346.8637536464489,
                                                0.0, 0.0, 1.0);
        cv::Mat D2 = (cv::Mat_<double>(4, 1) << -0.003041970188352833, -0.03517262328959603,
                                                0.0029221267247867286, -0.001623803476429974);

        // Rotation and translation from cam0 to cam1
        cv::Mat R = (cv::Mat_<double>(3, 3) << 0.99985716, -0.00507341, -0.01612204,
                                               0.00516383,  0.99997114,  0.00557172,
                                               0.0160933,  -0.00565418,  0.99985451);
        cv::Mat T = (cv::Mat_<double>(3, 1) << -0.07505134, -0.00021969, -0.00077246);  // in meters

        // Image size (you can use actual image size or set it manually)
        cv::Size imageSize = left.size();

        // Outputs
        cv::Mat R1, R2, P1, P2, Q;
        cv::stereoRectify(K1, D1, K2, D2, imageSize, R, T, R1, R2, P1, P2, Q);

        // Create rectification maps
        cv::Mat map1x, map1y, map2x, map2y;
        cv::initUndistortRectifyMap(K1, D1, R1, P1, imageSize, CV_32FC1, map1x, map1y);
        cv::initUndistortRectifyMap(K2, D2, R2, P2, imageSize, CV_32FC1, map2x, map2y);

        // Apply remapping
        cv::remap(left, left_rect, map1x, map1y, cv::INTER_LINEAR);
        cv::remap(right, right_rect, map2x, map2y, cv::INTER_LINEAR);
    }

    void right_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        right_img_ = msg;
        process_if_ready();
    }

    void process_if_ready() {
        if (!left_img_ || !right_img_) return;

        cv::Mat left = cv_bridge::toCvCopy(left_img_, "mono8")->image;
        cv::Mat right = cv_bridge::toCvCopy(right_img_, "mono8")->image;

        // Optional: stereo rectification
        //cv::Mat left_rect, right_rect;
        //rectifyStereoImages(left, right, left_rect, right_rect);
        //left = left_rect; right = right_rect;

        float* inputLeft = new float[1 * 3 * net_input_height_ * net_input_width_];
        float* inputRight = new float[1 * 3 * net_input_height_ * net_input_width_];
        float* outputData = new float[1 * net_input_height_ * net_input_width_];

        inputLeft = preprocess_image(left);
        inputRight = preprocess_image(right);

        // Copy input data to device
        cudaMemcpyAsync(buffers_[leftIndex_], inputLeft, inputSize_, cudaMemcpyHostToDevice, stream_);
        cudaMemcpyAsync(buffers_[rightIndex_], inputRight, inputSize_, cudaMemcpyHostToDevice, stream_);

        // Set tensor addresses
        context_->setTensorAddress(engine_->getIOTensorName(leftIndex_), buffers_[leftIndex_]);
        context_->setTensorAddress(engine_->getIOTensorName(rightIndex_), buffers_[rightIndex_]);
        context_->setTensorAddress(engine_->getIOTensorName(outputIndex_), buffers_[outputIndex_]);

        auto start = high_resolution_clock::now();

        // Run inference
        if (!context_->enqueueV3(stream_)) {
            std::cerr << "Inference failed\n";
        }

        // Copy output back to host
        cudaMemcpyAsync(outputData, buffers_[outputIndex_], outputSize_, cudaMemcpyDeviceToHost, stream_);
        cudaStreamSynchronize(stream_);

        auto end = high_resolution_clock::now();

        // Timing
        double elapsed_ms = duration<double, std::milli>(end - start).count();
        std::cout << "Elapsed time: " << elapsed_ms << " ms" << std::endl;
        // Convert and display
        cv::Mat disp_mat(net_input_height_, net_input_width_, CV_32FC1, outputData);

        int original_height = left.rows;
        int original_width = left.cols;

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
        cv::Mat disp_norm, disp_color;

        disp_filtered.convertTo(disp_norm, CV_8UC1, 255.0 / max_disp);

        double min_val = 0.1, max_val = max_disp;
        cv::minMaxLoc(disp_filtered, &min_val, &max_val, nullptr, nullptr, valid_mask);

        // Step 2: Normalize (bright = close)
        disp_filtered.convertTo(disp_norm, CV_8UC1, -255.0 / (max_val - min_val), 255.0 * max_val / (max_val - min_val));

        // Step 3: Apply perceptually uniform colormap
        cv::applyColorMap(disp_norm, disp_color, cv::COLORMAP_MAGMA);

        // cv::applyColorMap(disp_norm, disp_color, cv::COLORMAP_JET);

        cv::Mat left_color;
        if (left.channels() == 1) {
        cv::cvtColor(left, left_color, cv::COLOR_GRAY2BGR);
        } else {
        left_color = left.clone();
        }

        // Resize if needed to match heights (optional, if they mismatch due to processing)
        if (left_color.size() != disp_color.size()) {
        cv::resize(left_color, left_color, disp_color.size());
        }

        cv::Mat combined;

        // Show disparity image
        cv::hconcat(left_color, disp_color, combined);
        // Show side-by-side result

        if (record_video && !video_writer.isOpened()) {
            int fps = 30;  // Adjust based on your frame rate
            std::string output_path = "disparity_output.mp4";  // Or .mp4 if codec supports it
            int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');  // Codec (adjust as needed)
            cv::Size frame_size(combined.cols, combined.rows);
            video_writer.open(output_path, fourcc, fps, frame_size);
            if (!video_writer.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open video writer!");
            }
        }

        cv::imshow("Left + Disparity", combined);
        cv::waitKey(1);
        // Print sizes

        // Write frame to video (if recording is enabled)
        if (record_video && video_writer.isOpened()) {
            video_writer.write(combined);
        }

        delete[] inputLeft;
        delete[] inputRight;
        delete[] outputData;

        std::cout << "Original Image Size: " << left.cols << " x " << left.rows << std::endl;
        std::cout << "Disparity Size: " << disp_color.cols << " x " << disp_color.rows << std::endl;

        frame_counter = frame_counter + 1;
        std::cout << "Number of frames: " << frame_counter << std::endl;

        left_img_.reset();
        right_img_.reset();


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
};

StereoNode::Logger StereoNode::gLogger;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoNode>());
    rclcpp::shutdown();
    return 0;
}

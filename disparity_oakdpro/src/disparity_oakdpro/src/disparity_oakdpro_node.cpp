#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <torch/script.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <filesystem>
#include <opencv2/videoio.hpp>  // Required for VideoWriter
#include <memory>  // for std::shared_ptr
std::shared_ptr<torch::Tensor> disp = std::make_shared<torch::Tensor>();

using namespace std::chrono;
namespace fs = std::filesystem;

class StereoNode : public rclcpp::Node {
public:
    StereoNode() : Node("stereo_inference_node") {
        this->declare_parameter<std::string>("model_path", "GHUStereo8_nce_scripted.pt");
        this->get_parameter("model_path", model_path_);

        left_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/left/image_rect", 20,
            std::bind(&StereoNode::left_callback, this, std::placeholders::_1));

        right_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/right/image_rect", 20,
            std::bind(&StereoNode::right_callback, this, std::placeholders::_1));

        model_ = torch::jit::load(model_path_);
        model_.to(torch::kCUDA);
        model_.eval();

        RCLCPP_INFO(this->get_logger(), "Stereo inference node initialized.");
    }

private:
    torch::jit::script::Module model_;
    std::string model_path_;
    sensor_msgs::msg::Image::SharedPtr left_img_, right_img_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_, right_sub_;

    int pad_right;
    int pad_bottom;
    double max_disp = 96;
    cv::Mat disp_filtered;
    float alpha = 0.5;  // Adjust for responsiveness vs. smoothness

    bool record_video = true;  // Set to false to disable recording
    cv::VideoWriter video_writer;

    void left_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        left_img_ = msg;
        process_if_ready();
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
        // cv::Mat left_rect, right_rect;
        // rectifyStereoImages(left, right, left_rect, right_rect);
        // left = left_rect; right = right_rect;

        auto left_tensor = preprocess_image(left).to(torch::kCUDA);
        auto right_tensor = preprocess_image(right).to(torch::kCUDA);

        auto start = high_resolution_clock::now();
        std::vector<torch::jit::IValue> inputs = {left_tensor, right_tensor};
        torch::Tensor disparity_out = model_.forward(inputs).toTensor();

        auto disparity = disparity_out.clone();  // Safe copy
        std::thread([disparity]() {
            *disp = disparity.squeeze().detach().to(torch::kCPU, /*non_blocking=*/true);
             //Do your post-processing here
        }).detach();

        auto end = high_resolution_clock::now();
        double elapsed_ms = duration<double, std::milli>(end - start).count();

        if (disp->defined()) {
            RCLCPP_INFO(this->get_logger(), "Inference time: %.2f ms", elapsed_ms);
            std::cout << "Disparity Size: " << disp->size(0) << " x " << disp->size(1) << std::endl;

            // Convert and display
            cv::Mat disp_mat(disp->size(0), disp->size(1), CV_32FC1, disp->data_ptr<float>());
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
            std::cout << "Original Image Size: " << left.cols << " x " << left.rows << std::endl;
            std::cout << "Disparity Size: " << disp_color.cols << " x " << disp_color.rows << std::endl;
        }

        left_img_.reset();
        right_img_.reset();
    }

    torch::Tensor preprocess_image(const cv::Mat& img) {

        int w = img.cols;
        int h = img.rows;
        int m = 32;
        int wi = (w / m + 1) * m;
        int hi = (h / m + 1) * m;
        pad_right = wi - w;
        pad_bottom = hi - h;

        cv::Mat padded_img;
        cv::copyMakeBorder(img, padded_img, 0, pad_bottom, 0, pad_right, cv::BORDER_CONSTANT, cv::Scalar(0));
        padded_img.convertTo(padded_img, CV_32FC1, 1.0 / 255.0);

        cv::Mat img_rgb;
        cv::cvtColor(padded_img, img_rgb, cv::COLOR_GRAY2RGB);

        std::cout << "Feeding size: " << img_rgb.cols << " x " << img_rgb.rows << std::endl;

        auto tensor = torch::from_blob(img_rgb.data, {1, img_rgb.rows, img_rgb.cols, 3}, torch::kFloat);
        tensor = tensor.permute({0, 3, 1, 2}).clone();

        // Apply normalization: (tensor - mean) / std
        torch::Tensor mean = torch::tensor({0.485, 0.456, 0.406}).view({1, 3, 1, 1}).to(tensor.device());
        torch::Tensor std  = torch::tensor({0.229, 0.224, 0.225}).view({1, 3, 1, 1}).to(tensor.device());
        tensor = (tensor - mean) / std;
        return tensor;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoNode>());
    rclcpp::shutdown();
    return 0;
}

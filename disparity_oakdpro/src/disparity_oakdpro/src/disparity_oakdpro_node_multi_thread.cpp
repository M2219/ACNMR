#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <torch/script.h>
#include <torch/torch.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

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

        // Launch CPU thread for post-processing
        cpu_thread_ = std::thread([this]() {
            while (run_cpu_thread) {
                std::unique_lock<std::mutex> lock(queue_mutex);
                queue_cv.wait(lock, [this] { return !cpu_queue.empty() || !run_cpu_thread; });

                if (!run_cpu_thread) break;

                auto tensor = cpu_queue.front();
                cpu_queue.pop();
                lock.unlock();

                //auto disp = tensor.squeeze().detach().cpu();
                auto disp = tensor.squeeze().detach().to(torch::kCPU, /*non_blocking=*/true);
                post_process_disparity(disp);
            }
        });

        RCLCPP_INFO(this->get_logger(), "Stereo inference node initialized.");
    }

    ~StereoNode() {
        run_cpu_thread = false;
        queue_cv.notify_all();
        if (cpu_thread_.joinable()) {
            cpu_thread_.join();
        }
    }

private:
    torch::jit::script::Module model_;
    std::string model_path_;
    sensor_msgs::msg::Image::SharedPtr left_img_, right_img_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_, right_sub_;
    bool run_cpu_thread = true;
    bool record_video = false;
    int pad_right;
    int pad_bottom;
    double max_disp = 96;
    cv::Mat disp_filtered;
    float alpha = 0.5;
    cv::VideoWriter video_writer;
    cv::Mat left;
    cv::Mat right;

    std::queue<torch::Tensor> cpu_queue;
    std::mutex queue_mutex;
    std::condition_variable queue_cv;

    std::thread cpu_thread_;

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

        left = cv_bridge::toCvCopy(left_img_, "mono8")->image;
        right = cv_bridge::toCvCopy(right_img_, "mono8")->image;

        auto left_tensor = preprocess_image(left).to(torch::kCUDA);
        auto right_tensor = preprocess_image(right).to(torch::kCUDA);

        auto start = high_resolution_clock::now();
        std::vector<torch::jit::IValue> inputs = {left_tensor, right_tensor};
        torch::Tensor disparity = model_.forward(inputs).toTensor();

        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            if (cpu_queue.size() < 2) {
                cpu_queue.push(disparity);
                queue_cv.notify_one();
            }
        }

        auto end = high_resolution_clock::now();
        double elapsed_ms = duration<double, std::milli>(end - start).count();
        RCLCPP_INFO(this->get_logger(), "Inference time: %.2f ms", elapsed_ms);


        left_img_.reset();
        right_img_.reset();
    }

    void post_process_disparity(const torch::Tensor& disparity) {
        // Convert and display
        cv::Mat disp(disparity.size(0), disparity.size(1), CV_32FC1, disparity.data_ptr<float>());

        int original_height = left.rows;
        int original_width = left.cols;

        // Crop the disparity cv::Mat to remove padding
        if (pad_bottom > 0 || pad_right > 0) {
            disp = disp(cv::Rect(0, 0, original_width, original_height)); // Crop based on original size
        }

        //std::cout << "Disparity values:\n" << disp << std::endl;
        // 1. Spatial smoothing
        cv::medianBlur(disp, disp_filtered, 5);

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

        auto tensor = torch::from_blob(img_rgb.data, {1, img_rgb.rows, img_rgb.cols, 3}, torch::kFloat);
        tensor = tensor.permute({0, 3, 1, 2}).clone();

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

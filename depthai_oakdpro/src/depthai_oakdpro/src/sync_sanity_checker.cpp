#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

class SyncSanityChecker : public rclcpp::Node {
public:
    SyncSanityChecker() : Node("sync_sanity_checker") {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/left/image_raw", 10,
            std::bind(&SyncSanityChecker::image_callback, this, std::placeholders::_1)
        );

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/oak/imu", 100,
            std::bind(&SyncSanityChecker::imu_callback, this, std::placeholders::_1)
        );
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::Time last_image_time_;
    rclcpp::Time last_imu_time_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        last_image_time_ = msg->header.stamp;
        RCLCPP_INFO(this->get_logger(), "Image timestamp: %.9f", last_image_time_.seconds());
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        last_imu_time_ = msg->header.stamp;

        if (last_image_time_.nanoseconds() > 0) {
            double diff = (last_image_time_ - last_imu_time_).seconds();
            RCLCPP_INFO(this->get_logger(), "IMU timestamp: %.9f, Delta: %.6f sec", last_imu_time_.seconds(), diff);
        }
    }
};
  
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SyncSanityChecker>());
    rclcpp::shutdown();
    return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <queue>
#include <mutex>

class IMUBufferChecker : public rclcpp::Node
{
public:
    IMUBufferChecker()
        : Node("imu_buffer_checker")
    {
        std::string imu_topic = "/olive/imu/id01/filtered_imu";  // Change if needed
        imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 
            rclcpp::QoS(20000).best_effort(),  
            std::bind(&IMUBufferChecker::ImuCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscribed to IMU topic: %s", imu_topic.c_str());
    }

private:
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msgIMU)
    {
        if (!msgIMU)
        {
            RCLCPP_ERROR(this->get_logger(), "Received NULL IMU message!");
            return;
        }

        std::lock_guard<std::mutex> lock(bufMutex_);
        imuBuf_.push(msgIMU);

        // Print buffer size
        RCLCPP_INFO(this->get_logger(), "IMU Buffer Size: %zu", imuBuf_.size());

        // Optional: Prevent infinite growth by keeping only last 500 messages
        while (imuBuf_.size() > 500)
        {
            imuBuf_.pop();
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
    std::queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf_;
    std::mutex bufMutex_;
};

// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMUBufferChecker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

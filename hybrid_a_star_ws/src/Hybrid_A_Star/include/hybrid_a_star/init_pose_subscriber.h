#ifndef HYBRID_A_STAR_INIT_POSE_SUBSCRIBER_H
#define HYBRID_A_STAR_INIT_POSE_SUBSCRIBER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <deque>
#include <mutex>

class InitPoseSubscriber2D {
public:
    // Use an existing node instead of creating a new one
    explicit InitPoseSubscriber2D(const std::shared_ptr<rclcpp::Node> &node, const std::string &topic_name, size_t buff_size);

    void ParseData(std::deque<std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped>> &pose_data_buff);

private:
    void MessageCallBack(const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> init_pose_ptr);

    std::shared_ptr<rclcpp::Node> node_;  // Store reference to existing node
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscriber_;

    std::deque<std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped>> init_poses_;
    std::mutex buff_mutex_;
};

#endif // HYBRID_A_STAR_INIT_POSE_SUBSCRIBER_H

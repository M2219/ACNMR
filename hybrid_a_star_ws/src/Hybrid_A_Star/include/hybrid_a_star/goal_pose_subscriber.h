#ifndef HYBRID_A_STAR_GOAL_POSE_SUBSCRIBER_H
#define HYBRID_A_STAR_GOAL_POSE_SUBSCRIBER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <deque>
#include <mutex>

class GoalPoseSubscriber2D {
public:
    // Constructor must match the implementation file
    explicit GoalPoseSubscriber2D(const std::shared_ptr<rclcpp::Node> &node, const std::string &topic_name, size_t buff_size);

    void ParseData(std::deque<std::shared_ptr<geometry_msgs::msg::PoseStamped>> &pose_data_buff);

private:
    void MessageCallBack(const std::shared_ptr<geometry_msgs::msg::PoseStamped> goal_pose_ptr);

    std::shared_ptr<rclcpp::Node> node_;  // Reference to existing node
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_;

    std::deque<std::shared_ptr<geometry_msgs::msg::PoseStamped>> goal_poses_;
    std::mutex buff_mutex_;
};

#endif // HYBRID_A_STAR_GOAL_POSE_SUBSCRIBER_H

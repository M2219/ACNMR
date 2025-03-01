#include "hybrid_a_star/goal_pose_subscriber.h"

GoalPoseSubscriber2D::GoalPoseSubscriber2D(const std::shared_ptr<rclcpp::Node> &node, const std::string &topic_name, size_t buff_size)
    : node_(node) {  // Store reference to existing node

    subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic_name,
        buff_size,
        std::bind(&GoalPoseSubscriber2D::MessageCallBack, this, std::placeholders::_1)
    );
}

void GoalPoseSubscriber2D::MessageCallBack(const std::shared_ptr<geometry_msgs::msg::PoseStamped> goal_pose_ptr) {
    std::lock_guard<std::mutex> lock(buff_mutex_);
    goal_poses_.emplace_back(goal_pose_ptr);
}

void GoalPoseSubscriber2D::ParseData(std::deque<std::shared_ptr<geometry_msgs::msg::PoseStamped>> &pose_data_buff) {
    std::lock_guard<std::mutex> lock(buff_mutex_);
    if (!goal_poses_.empty()) {
        pose_data_buff.insert(pose_data_buff.end(), goal_poses_.begin(), goal_poses_.end());
        goal_poses_.clear();
    }
}


#include "hybrid_a_star/costmap_subscriber.h"

CostMapSubscriber::CostMapSubscriber(const std::shared_ptr<rclcpp::Node> &node, const std::string &topic_name, size_t buff_size)
    : node_(node) {  // Store reference to existing node

    subscriber_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        topic_name,
        buff_size,
        std::bind(&CostMapSubscriber::MessageCallBack, this, std::placeholders::_1)
    );
}

void CostMapSubscriber::MessageCallBack(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> costmap_msg_ptr) {
    std::unique_lock<std::mutex> lock(buff_mutex_);
    deque_costmap_.emplace_back(costmap_msg_ptr);
}
void CostMapSubscriber::ParseData(std::deque<std::shared_ptr<nav_msgs::msg::OccupancyGrid>> &deque_costmap_msg_ptr) {

    if (!rclcpp::ok()) {
        return;
    }

    std::unique_lock<std::mutex> lock(buff_mutex_);

    if (deque_costmap_.empty()) {
        return;
    }

    std::swap(deque_costmap_msg_ptr, deque_costmap_);
}

#include "hybrid_a_star/costmap_subscriber.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/srv/get_map.hpp"

CostMapSubscriber::CostMapSubscriber(const std::shared_ptr<rclcpp::Node> &node, const std::string &topic_name, size_t buff_size)
    : node_(node) {

    // Create a client for requesting the map
    map_client_ = node_->create_client<nav_msgs::srv::GetMap>("/map_server/map");

    // Wait for the service to be available before sending the request
    while (!rclcpp::ok() || !map_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node_->get_logger(), "Waiting for /map_server/map service...");
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    // Request the map
    RequestMap();

    // Subscribe to the /map topic
    subscriber_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        topic_name,
        buff_size,
        std::bind(&CostMapSubscriber::MessageCallBack, this, std::placeholders::_1)
    );
}

void CostMapSubscriber::RequestMap() {
    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();

    RCLCPP_INFO(node_->get_logger(), "Requesting map from /map_server/map...");

    // Send request asynchronously
    auto future = map_client_->async_send_request(request);

    // 🔹 Equivalent to `rclpy.spin_until_future_complete(self, future)`
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response) {
            RCLCPP_INFO(node_->get_logger(), "Successfully received map from service.");
            MessageCallBack(std::make_shared<nav_msgs::msg::OccupancyGrid>(response->map));
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Received empty response from map service.");
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get map from service.");
    }
}

void CostMapSubscriber::MessageCallBack(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> costmap_msg_ptr) {

    RCLCPP_INFO(node_->get_logger(), "Received map: width=%d, height=%d, resolution=%.2f",
                costmap_msg_ptr->info.width, costmap_msg_ptr->info.height,
                costmap_msg_ptr->info.resolution);

    std::unique_lock<std::mutex> lock(buff_mutex_);
    deque_costmap_.emplace_back(costmap_msg_ptr);
}
void CostMapSubscriber::ParseData(std::deque<std::shared_ptr<nav_msgs::msg::OccupancyGrid>> &deque_costmap_msg_ptr) {

    std::unique_lock<std::mutex> lock(buff_mutex_);

    if (deque_costmap_.empty()) {
        return;
    }

    std::swap(deque_costmap_msg_ptr, deque_costmap_);
}

#ifndef HYBRID_A_STAR_COSTMAP_SUBSCRIBER_H
#define HYBRID_A_STAR_COSTMAP_SUBSCRIBER_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <deque>
#include <mutex>

class CostMapSubscriber {
public:
    CostMapSubscriber(const std::shared_ptr<rclcpp::Node> &node, const std::string &topic_name, size_t buff_size);
    void ParseData(std::deque<std::shared_ptr<nav_msgs::msg::OccupancyGrid>> &deque_costmap_msg_ptr);

private:
    void MessageCallBack(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> costmap_msg_ptr);
    void RequestMap();  // Function to request the map

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_client_;  // 🔹 ADD THIS LINE
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscriber_;

    std::deque<std::shared_ptr<nav_msgs::msg::OccupancyGrid>> deque_costmap_;
    std::mutex buff_mutex_;
};

#endif // HYBRID_A_STAR_COSTMAP_SUBSCRIBER_H

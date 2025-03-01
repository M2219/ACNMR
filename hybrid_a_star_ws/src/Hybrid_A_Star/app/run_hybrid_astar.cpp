#include "hybrid_a_star/hybrid_a_star_flow.h"
#include "3rd/backward.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

namespace backward {
backward::SignalHandling sh;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::cout << "before creation" << std::endl;

    auto node = std::make_shared<HybridAStarFlow>(); // Create node using std::make_shared

    std::cout << "after creation" << std::endl;

    node->initializeSubscribers();

    std::cout << "initialized" << std::endl;

    std::cout << "Spinning node with Run() loop..." << std::endl;

    // Continuous loop for calling Run() alongside rclcpp::spin
    rclcpp::Rate loop_rate(10);  // Adjust loop frequency as needed (10 Hz)
    while (rclcpp::ok()) {
        node->Run();  // Call Run() continuously
        rclcpp::spin_some(node); // Allow ROS to process callbacks
        loop_rate.sleep();
    }

    std::cout << "Node stopped spinning!" << std::endl;

    rclcpp::shutdown();
    return 0;
}

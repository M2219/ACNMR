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


    auto node = std::make_shared<HybridAStarFlow>(); // Create node using std::make_shared

    node->initializeSubscribers();

    // Continuous loop for calling Run() alongside rclcpp::spin
    rclcpp::Rate loop_rate(10);  // Adjust loop frequency as needed (10 Hz)
    while (rclcpp::ok()) {
        node->Run();  // Call Run() continuously
        rclcpp::spin_some(node); // Allow ROS to process callbacks
        loop_rate.sleep();
    }

//rclcpp::executors::SingleThreadedExecutor executor;
//executor.add_node(node);
//executor.spin();  // Fully manages callbacks and subscriptions


    std::cout << "Node stopped spinning!" << std::endl;

    rclcpp::shutdown();
    return 0;
}

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "stereo-inertial-node.hpp"

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo - imu path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    auto node = std::make_shared<ORB_SLAM3_Wrapper::StereoInertialSlamNode>(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO);
    std::cout << "============================ " << std::endl;

    //auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();        
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    //executor->add_node(node);
    //executor->spin();
    rclcpp::shutdown();

    return 0;
}

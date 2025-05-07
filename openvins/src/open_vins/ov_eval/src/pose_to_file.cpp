/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "utils/Recorder.h"
#include "utils/print.h"

class PoseToFileNode : public rclcpp::Node {
public:
    PoseToFileNode() : Node("pose_to_file") {
        // Verbosity setting

        std::string verbosity;
        this->declare_parameter<std::string>("verbosity", "INFO");
        this->get_parameter("verbosity", verbosity);
        ov_core::Printer::setPrintLevel(verbosity);

        // Get parameters to subscribe
        std::string topic, topic_type, fileoutput;
        this->declare_parameter<std::string>("topic");
        this->declare_parameter<std::string>("topic_type");
        this->declare_parameter<std::string>("output");

        this->get_parameter("topic", topic);
        this->get_parameter("topic_type", topic_type);
        this->get_parameter("output", fileoutput);
        // Debug
        PRINT_DEBUG("Done reading config values");
        PRINT_DEBUG(" - topic = %s", topic.c_str());
        PRINT_DEBUG(" - topic_type = %s", topic_type.c_str());
        PRINT_DEBUG(" - file = %s", fileoutput.c_str());

        // Create the recorder object

        recorder_ = std::make_shared<ov_eval::Recorder>(fileoutput);


        // Subscribe to topic
        if (topic_type == "PoseWithCovarianceStamped") {
            subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                topic, 10, std::bind(&ov_eval::Recorder::callback_posecovariance, recorder_, std::placeholders::_1));
        } else if (topic_type == "PoseStamped") {
            subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                topic, 10, std::bind(&ov_eval::Recorder::callback_pose, recorder_, std::placeholders::_1));
        } else if (topic_type == "TransformStamped") {
            subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
                topic, 10, std::bind(&ov_eval::Recorder::callback_transform, recorder_, std::placeholders::_1));
        } else if (topic_type == "Odometry") {
            subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                topic, 10, std::bind(&ov_eval::Recorder::callback_odometry, recorder_, std::placeholders::_1));
        } else {
            PRINT_ERROR("The specified topic type is not supported");
            PRINT_ERROR("topic_type = %s", topic_type.c_str());
            PRINT_ERROR("please select from: PoseWithCovarianceStamped, PoseStamped, TransformStamped, Odometry");
            rclcpp::shutdown();
            return;
        }
    }

private:
    std::shared_ptr<ov_eval::Recorder> recorder_;
    rclcpp::SubscriptionBase::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create the ROS 2 node
    rclcpp::spin(std::make_shared<PoseToFileNode>());

    // Shutdown ROS 2
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}

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

#ifndef OV_EVAL_RECORDER_H
#define OV_EVAL_RECORDER_H

#include <fstream>
#include <iostream>
#include <string>

#include <Eigen/Eigen>
#include <boost/filesystem.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ov_eval {

/**
 * @brief This class takes in published poses and writes them to file.
 *
 * Original code is based on this modified [posemsg_to_file](https://github.com/rpng/posemsg_to_file/).
 * Output is in a text file that is space deliminated and can be read by all scripts.
 * If we have a covariance then we also save the upper triangular part to file so we can calculate NEES values.
 */
class Recorder
{
public:
    /**
     * @brief Default constructor that will open the specified file on disk.
     * If the output directory does not exist, this will also create the directory path to this file.
     * @param filename Desired file we want to "record" into
     */
    Recorder(const std::string &filename)
    {

        // Create folder path to this location if not exists
        boost::filesystem::path dir(filename.c_str());
        if (boost::filesystem::create_directories(dir.parent_path())) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Created folder path to output file.");
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Path: %s", dir.parent_path().c_str());
        }

        // If it exists, then delete it
        if (boost::filesystem::exists(filename)) {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Output file exists, deleting old file....");
            boost::filesystem::remove(filename);
        }

        // Open this file we want to write to
        outfile.open(filename.c_str());
        if (outfile.fail()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to open output file!!");
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Path: %s", filename.c_str());
            std::exit(EXIT_FAILURE);
        }

        outfile << "# timestamp(s) tx ty tz qx qy qz qw Pr11 Pr12 Pr13 Pr22 Pr23 Pr33 Pt11 Pt12 Pt13 Pt22 Pt23 Pt33" << std::endl;

        // Set initial state values
        timestamp = -1;
        q_ItoG << 0, 0, 0, 1;
        p_IinG = Eigen::Vector3d::Zero();
        cov_rot = Eigen::Matrix<double, 3, 3>::Zero();
        cov_pos = Eigen::Matrix<double, 3, 3>::Zero();
        has_covariance = false;
    }
  /**
   * @brief Callback for nav_msgs::Odometry message types.
   *
   * Note that covariance is in the order (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis).
   * http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovariance.html
   *
   * @param msg New message
   */
    void callback_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;  // Convert to seconds
        q_ItoG << msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
                  msg->pose.pose.orientation.z, msg->pose.pose.orientation.w;
        p_IinG << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;

        // Extract covariance matrix for position and rotation
        cov_pos << msg->pose.covariance[0], msg->pose.covariance[1], msg->pose.covariance[2],
                   msg->pose.covariance[6], msg->pose.covariance[7], msg->pose.covariance[8],
                   msg->pose.covariance[12], msg->pose.covariance[13], msg->pose.covariance[14];

        cov_rot << msg->pose.covariance[21], msg->pose.covariance[22], msg->pose.covariance[23],
                   msg->pose.covariance[27], msg->pose.covariance[28], msg->pose.covariance[29],
                   msg->pose.covariance[33], msg->pose.covariance[34], msg->pose.covariance[35];

        has_covariance = true;

        // Call a function to handle the data, e.g., writing it to a file
        write();
    }
  /**
   * @brief Callback for geometry_msgs::PoseStamped message types
   * @param msg New message
   */
    void callback_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;  // Convert to seconds
        q_ItoG << msg->pose.orientation.x, msg->pose.orientation.y,
                  msg->pose.orientation.z, msg->pose.orientation.w;
        p_IinG << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

        // Call a function to handle the data (e.g., writing it to a file)
        write();
    }
  /**
   * @brief Callback for geometry_msgs::PoseWithCovarianceStamped message types.
   *
   * Note that covariance is in the order (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis).
   * http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovariance.html
   *
   * @param msg New message
   */
    void callback_posecovariance(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // Extract timestamp from the message
        timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;  // Convert to seconds

        // Extract quaternion orientation
        q_ItoG << msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
                  msg->pose.pose.orientation.z, msg->pose.pose.orientation.w;

        // Extract position (translation)
        p_IinG << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;

        // Extract covariance for position and rotation
        cov_pos << msg->pose.covariance[0], msg->pose.covariance[1], msg->pose.covariance[2], 
                   msg->pose.covariance[6], msg->pose.covariance[7], msg->pose.covariance[8], 
                   msg->pose.covariance[12], msg->pose.covariance[13], msg->pose.covariance[14];

        cov_rot << msg->pose.covariance[21], msg->pose.covariance[22], msg->pose.covariance[23], 
                   msg->pose.covariance[27], msg->pose.covariance[28], msg->pose.covariance[29], 
                   msg->pose.covariance[33], msg->pose.covariance[34], msg->pose.covariance[35];

        has_covariance = true;

        // Call write function (you can define your own logic)
        write();
    }

  /**
   * @brief Callback for geometry_msgs::TransformStamped message types
   * @param msg New message
   */
    void callback_transform(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
    {
        // Extract timestamp from the message
        timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;  // Convert to seconds

        // Extract quaternion orientation
        q_ItoG << msg->transform.rotation.x, msg->transform.rotation.y, 
                  msg->transform.rotation.z, msg->transform.rotation.w;

        // Extract position (translation)
        p_IinG << msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z;

        // Call write function (you can define your own logic)
        write();
    }
protected:
  /**
   * @brief This is the main write function that will save to disk.
   * This should be called after we have saved the desired pose to our class variables.
   */
  void write() {

    // timestamp
    outfile.precision(5);
    outfile.setf(std::ios::fixed, std::ios::floatfield);
    outfile << timestamp << " ";

    // pose
    outfile.precision(6);
    outfile << p_IinG.x() << " " << p_IinG.y() << " " << p_IinG.z() << " " << q_ItoG(0) << " " << q_ItoG(1) << " " << q_ItoG(2) << " "
            << q_ItoG(3);

    // output the covariance only if we have it
    if (has_covariance) {
      outfile.precision(10);
      outfile << " " << cov_rot(0, 0) << " " << cov_rot(0, 1) << " " << cov_rot(0, 2) << " " << cov_rot(1, 1) << " " << cov_rot(1, 2) << " "
              << cov_rot(2, 2) << " " << cov_pos(0, 0) << " " << cov_pos(0, 1) << " " << cov_pos(0, 2) << " " << cov_pos(1, 1) << " "
              << cov_pos(1, 2) << " " << cov_pos(2, 2) << std::endl;
    } else {
      outfile << std::endl;
    }
  }

  // Output stream file
  std::ofstream outfile;

  // Temp storage objects for our pose and its certainty
  bool has_covariance = false;
  double timestamp;
  Eigen::Vector4d q_ItoG;
  Eigen::Vector3d p_IinG;
  Eigen::Matrix<double, 3, 3> cov_rot;
  Eigen::Matrix<double, 3, 3> cov_pos;
};

} // namespace ov_eval

#endif // OV_EVAL_RECORDER_H

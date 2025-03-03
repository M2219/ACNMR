#include "hybrid_a_star/hybrid_a_star_flow.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "all_config.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

HybridAStarFlow::HybridAStarFlow() : Node("hybrid_a_star_flow") {
    double steering_angle = 15; //MAX_STEER * 180 / M_PI;
    double wheel_base = WB;

    kinodynamic_astar_searcher_ptr_ = std::make_shared<HybridAStar>(
            steering_angle, steering_angle_discrete_num, segment_length, segment_length_discrete_num, wheel_base,
            steering_penalty, reversing_penalty, steering_change_penalty, shot_distance
    );

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("searched_path", 1);
    searched_tree_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("searched_tree", 1);
    vehicle_path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("vehicle_path", 1);

    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    has_map_ = false;
}

void HybridAStarFlow::Run() {
    ReadData();
    /*
    if (goal_pose_deque_.empty())
            RCLCPP_ERROR(this->get_logger(), "goal empty.");
    else
            RCLCPP_ERROR(this->get_logger(), "goal recived.");


    if (init_pose_deque_.empty())
            RCLCPP_ERROR(this->get_logger(), "start empty.");
    else
            RCLCPP_ERROR(this->get_logger(), "start recived.");
    */

    if (!has_map_) {
        if (costmap_deque_.empty()) {
            //RCLCPP_ERROR(this->get_logger(), "map not recived.");
            return;
        }

        current_costmap_ptr_ = costmap_deque_.front();
        costmap_deque_.pop_front();

        const double map_resolution = static_cast<float>(current_costmap_ptr_->info.resolution);
        kinodynamic_astar_searcher_ptr_->Init(
                current_costmap_ptr_->info.origin.position.x,
                current_costmap_ptr_->info.width * map_resolution,
	                current_costmap_ptr_->info.origin.position.y,
                current_costmap_ptr_->info.height * map_resolution,
                1.0, map_resolution
        );

        for (unsigned int w = 0; w < current_costmap_ptr_->info.width; ++w) {
            for (unsigned int h = 0; h < current_costmap_ptr_->info.height; ++h) {
                if (current_costmap_ptr_->data[h * current_costmap_ptr_->info.width + w]) {
                    kinodynamic_astar_searcher_ptr_->SetObstacle(w, h);
                }
            }
        }
        has_map_ = true;
    }

    costmap_deque_.clear();

    while (HasStartPose() && HasGoalPose()) {

        InitPoseData();
        double start_yaw = tf2::getYaw(current_init_pose_ptr_->pose.pose.orientation);
        double goal_yaw = tf2::getYaw(current_goal_pose_ptr_->pose.orientation);

        Vec3d start_state(current_init_pose_ptr_->pose.pose.position.x,
                          current_init_pose_ptr_->pose.pose.position.y,
                          start_yaw);
        Vec3d goal_state(current_goal_pose_ptr_->pose.position.x,
                         current_goal_pose_ptr_->pose.position.y,
                         goal_yaw);

        if (kinodynamic_astar_searcher_ptr_->Search(start_state, goal_state)) {
            auto path = kinodynamic_astar_searcher_ptr_->GetPath();
            PublishPath(path);
            PublishVehiclePath(path, 4.0, 2.0, 5u);
            PublishSearchedTree(kinodynamic_astar_searcher_ptr_->GetSearchedTree());
        }

        kinodynamic_astar_searcher_ptr_->Reset();
    }
}

void HybridAStarFlow::ReadData() {
    costmap_sub_ptr_->ParseData(costmap_deque_);
    init_pose_sub_ptr_->ParseData(init_pose_deque_);
    goal_pose_sub_ptr_->ParseData(goal_pose_deque_);
}

void HybridAStarFlow::InitPoseData() {
    current_init_pose_ptr_ = init_pose_deque_.front();
    init_pose_deque_.pop_front();

    current_goal_pose_ptr_ = goal_pose_deque_.front();
    goal_pose_deque_.pop_front();
}

bool HybridAStarFlow::HasGoalPose() {
    return !goal_pose_deque_.empty();
}

bool HybridAStarFlow::HasStartPose() {
    return !init_pose_deque_.empty();
}

void HybridAStarFlow::PublishPath(const VectorVec3d &path) {
    nav_msgs::msg::Path nav_path;
    geometry_msgs::msg::PoseStamped pose_stamped;

    for (const auto &pose : path) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();

        tf2::Quaternion q;
        q.setRPY(0, 0, pose.z());  // Roll=0, Pitch=0, Yaw=pose.z()
        pose_stamped.pose.orientation = tf2::toMsg(q);  // Convert to ROS2 message format

        nav_path.poses.push_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = this->get_clock()->now();
    path_pub_->publish(nav_path);
}

void HybridAStarFlow::PublishSearchedTree(const VectorVec4d &searched_tree) {
    visualization_msgs::msg::Marker tree_list;
    tree_list.header.frame_id = "world";
    tree_list.header.stamp = this->get_clock()->now();
    tree_list.type = visualization_msgs::msg::Marker::LINE_LIST;
    tree_list.action = visualization_msgs::msg::Marker::ADD;
    tree_list.ns = "searched_tree";
    tree_list.scale.x = 0.02;

    tree_list.color.a = 1.0;
    tree_list.color.r = 0;
    tree_list.color.g = 0;
    tree_list.color.b = 0;

    tree_list.pose.orientation.w = 1.0;
    tree_list.pose.orientation.x = 0.0;
    tree_list.pose.orientation.y = 0.0;
    tree_list.pose.orientation.z = 0.0;

    geometry_msgs::msg::Point point;
    for (const auto &i: searched_tree) {
        point.x = i.x();
        point.y = i.y();
        point.z = 0.0;
        tree_list.points.push_back(point);

        point.x = i.z();
        point.y = i.w();
        point.z = 0.0;
        tree_list.points.push_back(point);
    }

    searched_tree_pub_->publish(tree_list);
}

void HybridAStarFlow::PublishVehiclePath(const VectorVec3d &path, double width,
                                         double length, unsigned int vehicle_interval) {
    visualization_msgs::msg::MarkerArray vehicle_array;

    for (unsigned int i = 0; i < path.size(); i += vehicle_interval) {
        visualization_msgs::msg::Marker vehicle;

        if (i == 0) {
            vehicle.action = visualization_msgs::msg::Marker::DELETEALL;
        }

        vehicle.header.frame_id = "world";
        vehicle.header.stamp = this->get_clock()->now();
        vehicle.type = visualization_msgs::msg::Marker::CUBE;
        vehicle.id = static_cast<int>(i / vehicle_interval);
        vehicle.scale.x = width;
        vehicle.scale.y = length;
        vehicle.scale.z = 0.01;
        vehicle.color.a = 0.1;

        vehicle.color.r = 1.0;
        vehicle.color.b = 0.0;
        vehicle.color.g = 0.0;

        vehicle.pose.position.x = path[i].x();
        vehicle.pose.position.y = path[i].y();
        vehicle.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, path[i].z());  // Roll=0, Pitch=0, Yaw=path[i].z()
        vehicle.pose.orientation = tf2::toMsg(q);

        vehicle_array.markers.push_back(vehicle);
    }

    vehicle_path_pub_->publish(vehicle_array);
}

void HybridAStarFlow::initializeSubscribers() {
    costmap_sub_ptr_ = std::make_shared<CostMapSubscriber>(shared_from_this(), "/map", 10);
    init_pose_sub_ptr_ = std::make_shared<InitPoseSubscriber2D>(shared_from_this(), "/initialpose", 10);
    goal_pose_sub_ptr_ = std::make_shared<GoalPoseSubscriber2D>(shared_from_this(), "/goal", 10);
}

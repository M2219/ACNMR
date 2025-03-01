#ifndef HYBRID_A_STAR_HYBRID_A_STAR_H_FLOW
#define HYBRID_A_STAR_HYBRID_A_STAR_H_FLOW

#include "type.h"
#include "hybrid_a_star.h"
#include "costmap_subscriber.h"
#include "init_pose_subscriber.h"
#include "goal_pose_subscriber.h"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class HybridAStarFlow : public rclcpp::Node {
public:
    HybridAStarFlow();
    void initializeSubscribers();
    void Run();

private:
    void InitPoseData();
    void ReadData();
    bool HasStartPose();
    bool HasGoalPose();

    void PublishPath(const VectorVec3d &path);
    void PublishSearchedTree(const VectorVec4d &searched_tree);
    void PublishVehiclePath(const VectorVec3d &path, double width, double length, unsigned int vehicle_interval);

    std::shared_ptr<HybridAStar> kinodynamic_astar_searcher_ptr_;
    std::shared_ptr<CostMapSubscriber> costmap_sub_ptr_;
    std::shared_ptr<InitPoseSubscriber2D> init_pose_sub_ptr_;
    std::shared_ptr<GoalPoseSubscriber2D> goal_pose_sub_ptr_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr searched_tree_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vehicle_path_pub_;

    std::deque<std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped>> init_pose_deque_;
    std::deque<std::shared_ptr<geometry_msgs::msg::PoseStamped>> goal_pose_deque_;
    std::deque<std::shared_ptr<nav_msgs::msg::OccupancyGrid>> costmap_deque_;

    std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> current_init_pose_ptr_;
    std::shared_ptr<geometry_msgs::msg::PoseStamped> current_goal_pose_ptr_;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> current_costmap_ptr_;

    rclcpp::Time timestamp_;
    bool has_map_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
};

#endif // HYBRID_A_STAR_HYBRID_A_STAR_FLOW_H

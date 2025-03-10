#ifndef __STEREO_SLAM_NODE_HPP__
#define __STEREO_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <ros_utils.hpp>

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using ImageMsg = sensor_msgs::msg::Image;
using PcdMsg = sensor_msgs::msg::PointCloud2;
using PoseMsg = geometry_msgs::msg::PoseStamped;
using OdomMsg = nav_msgs::msg::Odometry;

class StereoSlamNode : public rclcpp::Node
{
public:
    StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify);

    ~StereoSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;

    void GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight);

    ORB_SLAM3::System* m_SLAM;

    bool doRectify;
    cv::Mat M1l,M2l,M1r,M2r;
    Eigen::Vector3f Wwb = Eigen::Vector3f::Zero();
    Eigen::Vector3f Vwb = Eigen::Vector3f::Zero();
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    cv_bridge::CvImageConstPtr cv_ptrRight;

    std::shared_ptr<message_filters::Subscriber<ImageMsg> > left_sub;
    std::shared_ptr<message_filters::Subscriber<ImageMsg> > right_sub;

    rclcpp::Publisher<OdomMsg>::SharedPtr pubodo_;
    rclcpp::Publisher<PoseMsg>::SharedPtr pubPose_;
    rclcpp::Publisher<PcdMsg>::SharedPtr pubPcd_;
    rclcpp::Publisher<ImageMsg>::SharedPtr pubTrackImage_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;
};

#endif

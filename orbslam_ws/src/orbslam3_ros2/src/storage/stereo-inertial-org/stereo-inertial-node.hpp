#ifndef __STEREO_INERTIAL_NODE_HPP__
#define __STEREO_INERTIAL_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;
using PcdMsg = sensor_msgs::msg::PointCloud2;
using PoseMsg = geometry_msgs::msg::PoseStamped;
using OdomMsg = nav_msgs::msg::Odometry;

class StereoInertialNode : public rclcpp::Node
{
public:
    StereoInertialNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual);
    ~StereoInertialNode();

private:
    void GrabImu(const ImuMsg::SharedPtr msg);
    void GrabImageLeft(const ImageMsg::SharedPtr msgLeft);
    void GrabImageRight(const ImageMsg::SharedPtr msgRight);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    void SyncWithImu();

    rclcpp::Subscription<ImuMsg>::SharedPtr   subImu_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgLeft_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgRight_;

    rclcpp::Publisher<PoseMsg>::SharedPtr pubPose_;
    rclcpp::Publisher<OdomMsg>::SharedPtr pubOdom_;
    rclcpp::Publisher<PcdMsg>::SharedPtr pubPcd_;
    rclcpp::Publisher<ImageMsg>::SharedPtr pubTrackImage_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    ORB_SLAM3::System *SLAM_;
    std::thread *syncThread_;

    // IMU
    queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutex_;

    // Image
    queue<ImageMsg::SharedPtr> imgLeftBuf_, imgRightBuf_;
    std::mutex bufMutexLeft_, bufMutexRight_;

    bool doRectify_;
    bool doEqual_;
    cv::Mat M1l_, M2l_, M1r_, M2r_;

    bool bClahe_;
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
};

#endif

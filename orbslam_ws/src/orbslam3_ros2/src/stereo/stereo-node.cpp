/**
 * @file stereod-slam-node.cpp
 * @brief Implementation of the StereoSlamNode Wrapper class.
 * @author Suchetan R S (rssuchetan@gmail.com)
 */
#include "stereo-node.hpp"

#include <opencv2/core/core.hpp>

namespace ORB_SLAM3_Wrapper
{
    StereoSlamNode::StereoSlamNode(const std::string &strVocFile,
                               const std::string &strSettingsFile,
                               ORB_SLAM3::System::eSensor sensor)
        : Node("ORB_SLAM3_STEREO_ROS2")
    {
        // Declare parameters (topic names)
        this->declare_parameter("left_image_topic_name", rclcpp::ParameterValue("/left/image_rect"));
        this->declare_parameter("right_image_topic_name", rclcpp::ParameterValue("/right/image_rect"));
        this->declare_parameter("odom_topic_name", rclcpp::ParameterValue("/odom"));

        // ROS Subscribers


        std::string left_topic = this->get_parameter("left_image_topic_name").as_string();
        std::string right_topic = this->get_parameter("right_image_topic_name").as_string();

        // Wait for the topics to be published
        while (rclcpp::ok() && (!topic_exists(left_topic) || !topic_exists(right_topic))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for stereo camera topics: %s and %s...", left_topic.c_str(), right_topic.c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        RCLCPP_INFO(this->get_logger(), "Topic detected!");

        imgLSub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, left_topic);
        imgRSub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, right_topic);

        syncApproximate_ = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(10), *imgLSub_, *imgRSub_);
        syncApproximate_->registerCallback(&StereoSlamNode::StereoCallback, this);

        odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(this->get_parameter("odom_topic_name").as_string(), 1000, std::bind(&StereoSlamNode::OdomCallback, this, std::placeholders::_1));

        // ROS Publishers
        //---- the following is published when a service is called
        mapPointsPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_points", 10);
        visibleLandmarksPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("visible_landmarks", 10);
        visibleLandmarksPose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("visible_landmarks_pose", 10);
        slamInfoPub_ = this->create_publisher<slam_msgs::msg::SlamInfo>("slam_info", 10);
        //---- the following is published continously
        mapDataPub_ = this->create_publisher<slam_msgs::msg::MapData>("map_data", 10);
        robotPoseMapFrame_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("robot_pose_slam", 10);

        // Services
        getMapDataService_ = this->create_service<slam_msgs::srv::GetMap>("orb_slam3/get_map_data", std::bind(&StereoSlamNode::getMapServer, this,
                                                                                                              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        pointsInViewCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        getMapPointsService_ = this->create_service<slam_msgs::srv::GetLandmarksInView>("orb_slam3/get_landmarks_in_view", std::bind(&StereoSlamNode::getMapPointsInViewServer, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rmw_qos_profile_services_default, pointsInViewCallbackGroup_);
        mapPointsCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        mapPointsService_ = this->create_service<slam_msgs::srv::GetAllLandmarksInMap>("orb_slam3/get_all_landmarks_in_map", std::bind(&StereoSlamNode::publishMapPointCloud, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rmw_qos_profile_services_default, mapPointsCallbackGroup_);
        resetLocalMapSrv_ = this->create_service<std_srvs::srv::SetBool>("orb_slam3/reset_mapping", std::bind(&StereoSlamNode::resetActiveMapSrv, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rmw_qos_profile_services_default, mapPointsCallbackGroup_);

        // TF
        tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

        bool bUseViewer;
        this->declare_parameter("visualization", rclcpp::ParameterValue(true));
        this->get_parameter("visualization", bUseViewer);

        this->declare_parameter("robot_base_frame", "base_footprint");
        this->get_parameter("robot_base_frame", robot_base_frame_id_);

        this->declare_parameter("global_frame", "map");
        this->get_parameter("global_frame", global_frame_);

        this->declare_parameter("odom_frame", "odom");
        this->get_parameter("odom_frame", odom_frame_id_);

        this->declare_parameter("robot_x", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_x", robot_x_);

        this->declare_parameter("robot_y", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_y", robot_y_);

        this->declare_parameter("robot_z", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_z", robot_z_);

        // Declare and get the quaternion components
        this->declare_parameter("robot_qx", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_qx", robot_qx_);

        this->declare_parameter("robot_qy", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_qy", robot_qy_);

        this->declare_parameter("robot_qz", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_qz", robot_qz_);

        this->declare_parameter("robot_qw", rclcpp::ParameterValue(1.0));
        this->get_parameter("robot_qw", robot_qw_);

        // Create and populate the Pose message
        geometry_msgs::msg::Pose initial_pose;
        initial_pose.position.x = robot_x_;
        initial_pose.position.y = robot_y_;
        initial_pose.position.z = robot_z_;
        initial_pose.orientation.x = robot_qx_;
        initial_pose.orientation.y = robot_qy_;
        initial_pose.orientation.z = robot_qz_;
        initial_pose.orientation.w = robot_qw_;

        this->declare_parameter("odometry_mode", rclcpp::ParameterValue(false));
        this->get_parameter("odometry_mode", odometry_mode_);

        this->declare_parameter("publish_tf", rclcpp::ParameterValue(true));
        this->get_parameter("publish_tf", publish_tf_);


        this->declare_parameter("map_data_publish_frequency", rclcpp::ParameterValue(1000));
        this->get_parameter("map_data_publish_frequency", map_data_publish_frequency_);

        this->declare_parameter("do_loop_closing", rclcpp::ParameterValue(false));
        this->get_parameter("do_loop_closing", do_loop_closing_);

        // Timers
        mapDataCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        mapDataTimer_ = this->create_wall_timer(std::chrono::milliseconds(map_data_publish_frequency_), std::bind(&StereoSlamNode::publishMapData, this), mapDataCallbackGroup_);

        interface_ = std::make_shared<ORB_SLAM3_Wrapper::ORBSLAM3Interface>(strVocFile, strSettingsFile,
                                                                            sensor, bUseViewer, do_loop_closing_, initial_pose, global_frame_, odom_frame_id_,
                                                                            robot_base_frame_id_);

        frequency_tracker_count_ = 0;
        frequency_tracker_clock_ = std::chrono::high_resolution_clock::now();

        RCLCPP_INFO(this->get_logger(), "CONSTRUCTOR END!");
    }

    StereoSlamNode::~StereoSlamNode()
    {
        imgLSub_.reset();
        imgRSub_.reset();
        odomSub_.reset();
        interface_.reset();

        RCLCPP_INFO(this->get_logger(), "DESTRUCTOR!");
    }

    bool StereoSlamNode::topic_exists(const std::string &topic_name) {
        auto topic_names_and_types = this->get_topic_names_and_types();
        return topic_names_and_types.find(topic_name) != topic_names_and_types.end();
    }

    void StereoSlamNode::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msgOdom)
    {
        if (odometry_mode_)
        {   // populate map to odom tf if odometry is being used
            RCLCPP_DEBUG_STREAM(this->get_logger(), "OdomCallback");
            interface_->getMapToOdomTF(msgOdom, tfMapOdom_);
        }
        else
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 4000, "Odometry msg recorded but no odometry mode is true, set to false to use this odometry");
    }

    void StereoSlamNode::StereoCallback(const sensor_msgs::msg::Image::SharedPtr msgL, const sensor_msgs::msg::Image::SharedPtr msgR)
    {
        if (!msgL || !msgR) {
            RCLCPP_ERROR(this->get_logger(), "StereoCallback: One or both images are NULL!");
            return;
        }

        Sophus::SE3f Tcw;
        bool track_result = interface_->trackStereo(msgL, msgR, Tcw);

        if (track_result)
        {
            isTracked_ = true;
            if (publish_tf_)
            {
                // populate map to base_footprint tf if odometry is not being used
                if (!odometry_mode_)
                {
                    tfMapOdom_ = geometry_msgs::msg::TransformStamped();
                    tfMapOdom_.header.stamp = msgL->header.stamp;
                    tfMapOdom_.header.frame_id = global_frame_;
                    tfMapOdom_.child_frame_id = odom_frame_id_;
                    tfBroadcaster_->sendTransform(tfMapOdom_);
                    interface_->getDirectOdomToRobotTF(msgL->header, tfMapOdom_);
                }
                // publish the tf if publish_tf_ is true
                tfBroadcaster_->sendTransform(tfMapOdom_);
            }
            geometry_msgs::msg::PoseStamped pose;
            interface_->getRobotPose(pose);
            pose.header.stamp = msgL->header.stamp;
            robotPoseMapFrame_->publish(pose);

            ++frequency_tracker_count_;
        }
    }

    void StereoSlamNode::publishMapPointCloud(std::shared_ptr<rmw_request_id_t> request_header,
                                            std::shared_ptr<slam_msgs::srv::GetAllLandmarksInMap::Request> request,
                                            std::shared_ptr<slam_msgs::srv::GetAllLandmarksInMap::Response> response)
    {
        if (isTracked_)
        {
            // Using high resolution clock to measure time
            auto start = std::chrono::high_resolution_clock::now();

            sensor_msgs::msg::PointCloud2 mapPCL;

            auto t1 = std::chrono::high_resolution_clock::now();
            auto time_create_mapPCL = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - start).count();
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Time to create mapPCL object: " << time_create_mapPCL << " seconds");

            interface_->getCurrentMapPoints(mapPCL);

            if (mapPCL.data.size() == 0)
                return;

            auto t2 = std::chrono::high_resolution_clock::now();
            auto time_get_map_points = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Time to get current map points: " << time_get_map_points << " seconds");

            mapPointsPub_->publish(mapPCL);
            auto t3 = std::chrono::high_resolution_clock::now();
            auto time_publish_map_points = std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count();
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Time to publish map points: " << time_publish_map_points << " seconds");
            RCLCPP_DEBUG_STREAM(this->get_logger(), "=======================");

            // Calculate the time taken for each line

            // Print the time taken for each line
            response->landmarks = mapPCL;
        }
    }

    void StereoSlamNode::resetActiveMapSrv(std::shared_ptr<rmw_request_id_t> request_header,
                           std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                           std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        interface_->resetLocalMapping();
    }

    void StereoSlamNode::publishMapData()
    {
        if (isTracked_)
        {
            auto start = std::chrono::high_resolution_clock::now();
            slam_msgs::msg::SlamInfo slamInfoMsg;
            //RCLCPP_DEBUG_STREAM(this->get_logger(), "Publishing map data");
            double tracking_freq = frequency_tracker_count_ / std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - frequency_tracker_clock_).count();
            //RCLCPP_INFO_STREAM(this->get_logger(), "Current ORB-SLAM3 tracking frequency: " << tracking_freq << " frames / sec");
            frequency_tracker_clock_ = std::chrono::high_resolution_clock::now();
            frequency_tracker_count_ = 0;
            // publish the map data (current active keyframes etc)
            slam_msgs::msg::MapData mapDataMsg;
            interface_->mapDataToMsg(mapDataMsg, true, false);
            //mapDataPub_->publish(mapDataMsg);
            slamInfoMsg.num_maps = interface_->getNumberOfMaps();
            slamInfoMsg.num_keyframes_in_current_map = mapDataMsg.graph.poses_id.size();
            slamInfoMsg.tracking_frequency = tracking_freq;
            auto t1 = std::chrono::high_resolution_clock::now();
            auto time_publishMapData = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - start).count();
            //RCLCPP_DEBUG_STREAM(this->get_logger(), "Time to create mapdata: " << time_publishMapData << " seconds");
            //RCLCPP_INFO_STREAM(this->get_logger(), "*************************");
            slamInfoPub_->publish(slamInfoMsg);
        }
    }

    void StereoSlamNode::getMapServer(std::shared_ptr<rmw_request_id_t> request_header,
                                    std::shared_ptr<slam_msgs::srv::GetMap::Request> request,
                                    std::shared_ptr<slam_msgs::srv::GetMap::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "GetMap2 service called.");
        slam_msgs::msg::MapData mapDataMsg;
        interface_->mapDataToMsg(mapDataMsg, false, request->tracked_points, request->kf_id_for_landmarks);
        response->data = mapDataMsg;
    }

    void StereoSlamNode::getMapPointsInViewServer(std::shared_ptr<rmw_request_id_t> request_header,
                                                std::shared_ptr<slam_msgs::srv::GetLandmarksInView::Request> request,
                                                std::shared_ptr<slam_msgs::srv::GetLandmarksInView::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "GetMapPointsInView service called.");
        std::vector<slam_msgs::msg::MapPoint> landmarks;
        std::vector<ORB_SLAM3::MapPoint*> points;
        interface_->mapPointsVisibleFromPose(request->pose, points, 1000, request->max_dist_pose_observation, request->max_angle_pose_observation);
        auto affineMapToPos = interface_->getTypeConversionPtr()->poseToAffine(request->pose);
        auto affinePosToMap = affineMapToPos.inverse();
        // Populate the pose of the points vector into the ros message
        for (const auto& point : points) {
            slam_msgs::msg::MapPoint landmark;
            Eigen::Vector3f landmark_position = point->GetWorldPos();
            auto position = interface_->getTypeConversionPtr()->vector3fORBToROS(landmark_position);
            position = interface_->getTypeConversionPtr()->transformPointWithReference<Eigen::Vector3f>(affinePosToMap, position);
            // RCLCPP_INFO_STREAM(this->get_logger(), "x: " << position.x() << " y: " << position.y() << " z: " << position.z());
            landmark.position.x = position.x();
            landmark.position.y = position.y();
            landmark.position.z = position.z();
            landmarks.push_back(landmark);
        }
        response->map_points = landmarks;
        auto cloud = interface_->getTypeConversionPtr()->MapPointsToPCL(points);
        visibleLandmarksPub_->publish(cloud);

        // Convert the pose in request to PoseStamped and publish
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = this->now();
        pose_stamped.header.frame_id = "map"; // Assuming the frame is "map", adjust if needed
        pose_stamped.pose = request->pose;

        // Publish the PoseStamped
        visibleLandmarksPose_->publish(pose_stamped);
    }
}

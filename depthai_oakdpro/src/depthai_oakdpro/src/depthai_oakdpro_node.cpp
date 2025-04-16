#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "sensor_msgs/msg/imu.hpp"

#include "depthai/depthai.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("depthai_oakdpro_node");

    // Initialize pipeline
    std::shared_ptr<dai::Pipeline> pipeline;
    pipeline = std::make_shared<dai::Pipeline>();

    // IMU
    auto imu = pipeline->create<dai::node::IMU>();
    auto xoutImu = pipeline->create<dai::node::XLinkOut>();
    xoutImu->setStreamName("imu");
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 400);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(1);  // Get one message only for now.
    imu->out.link(xoutImu->input);

    // Create mono cameras
    auto camLeft = pipeline->create<dai::node::MonoCamera>();
    auto camRight = pipeline->create<dai::node::MonoCamera>();
    auto stereo = pipeline->create<dai::node::StereoDepth>();

    auto xoutLeft = pipeline->create<dai::node::XLinkOut>();
    auto xoutRight = pipeline->create<dai::node::XLinkOut>();

    auto xoutRectifL = pipeline->create<dai::node::XLinkOut>();
    auto xoutRectifR = pipeline->create<dai::node::XLinkOut>();

    // Set camera properties
    camLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    camLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    camLeft->setFps(20.0);
    //camLeft.setSyncMode(True)

    camRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    camRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    camRight->setFps(20.0);
    //camRight.setSyncMode(True)

    stereo->setRectifyEdgeFillColor(0);
    camLeft->out.link(stereo->left);
    camRight->out.link(stereo->right);
    stereo->syncedLeft.link(xoutLeft->input);
    stereo->syncedRight.link(xoutRight->input);

    // Set XLinkOut stream names
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");

    xoutRectifL->setStreamName("rectified_left");
    xoutRectifR->setStreamName("rectified_right");

    stereo->rectifiedLeft.link(xoutRectifL->input);
    stereo->rectifiedRight.link(xoutRectifR->input);

    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>(*pipeline);
    auto calibrationHandler = device->readCalibration();

    auto leftQueue = device->getOutputQueue("left", 30, false);
    auto rightQueue = device->getOutputQueue("right", 30, false);

    auto rectifLeftQueue = device->getOutputQueue("rectified_left", 30, false);
    auto rectifRightQueue = device->getOutputQueue("rectified_right", 30, false);

    // Image converters
    dai::rosBridge::ImageConverter imageConverterLeft("oak_left_camera_optical_frame", true);
    //imageConverterLeft.setUpdateRosBaseTimeOnToRosMsg(true);

    dai::rosBridge::ImageConverter imageConverterRight("oak_right_camera_optical_frame", true);
    //imageConverterRight.setUpdateRosBaseTimeOnToRosMsg(true);

    dai::rosBridge::ImageConverter imageConverterLeftRect("oak_left_camera_optical_frame", true);
    //imageConverterLeftRect.setUpdateRosBaseTimeOnToRosMsg(true);

    dai::rosBridge::ImageConverter imageConverterRightRect("oak_right_camera_optical_frame", true);
    //imageConverterRightRect.setUpdateRosBaseTimeOnToRosMsg(true);


    const std::string leftPubName =  std::string("left/image_raw");
    const std::string rightPubName = std::string("right/image_raw");

    const std::string leftRectPubName =  std::string("left/image_rect");
    const std::string rightRectPubName = std::string("right/image_rect");

    int width = 1280;
    int height = 720;

    auto leftCameraInfo = imageConverterLeft.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_B, width, height);
    auto rightCameraInfo = imageConverterRight.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, width, height);

    // Bridge Publishers
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> leftPublish(
        leftQueue,
        node,
        leftPubName,
        [&](std::shared_ptr<dai::ImgFrame> frame, std::deque<sensor_msgs::msg::Image> &rosMsgs) {
        imageConverterLeft.toRosMsg(frame, rosMsgs);
        if (!rosMsgs.empty()) {
            //rosMsgs.front().header.stamp = node->now();
            rclcpp::Time dai_time = rclcpp::Time(frame->getTimestamp().time_since_epoch().count());
            rosMsgs.front().header.stamp = dai_time;

        }
        },
        30,
        leftCameraInfo,
        "left");

    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightPublish(
        rightQueue,
        node,
        rightPubName,
        [&](std::shared_ptr<dai::ImgFrame> frame, std::deque<sensor_msgs::msg::Image> &rosMsgs) {
        imageConverterRight.toRosMsg(frame, rosMsgs);
        if (!rosMsgs.empty()) {
            //rosMsgs.front().header.stamp = node->now();
            rclcpp::Time dai_time = rclcpp::Time(frame->getTimestamp().time_since_epoch().count());
            rosMsgs.front().header.stamp = dai_time;

        }
        },
        30,
        rightCameraInfo,
        "right");

    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> leftRectPublish(
        rectifLeftQueue,
        node,
        leftRectPubName,
        [&](std::shared_ptr<dai::ImgFrame> frame, std::deque<sensor_msgs::msg::Image> &rosMsgs) {
        imageConverterLeftRect.toRosMsg(frame, rosMsgs);
        if (!rosMsgs.empty()) {
            //rosMsgs.front().header.stamp = node->now();
            rclcpp::Time dai_time = rclcpp::Time(frame->getTimestamp().time_since_epoch().count());
            rosMsgs.front().header.stamp = dai_time;

        }
        },
        30,
        leftCameraInfo,
        "left");

    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightRectPublish(
        rectifRightQueue,
        node,
        rightRectPubName,
        [&](std::shared_ptr<dai::ImgFrame> frame, std::deque<sensor_msgs::msg::Image> &rosMsgs) {
        imageConverterRightRect.toRosMsg(frame, rosMsgs);
        if (!rosMsgs.empty()) {
            //rosMsgs.front().header.stamp = node->now();
            rclcpp::Time dai_time = rclcpp::Time(frame->getTimestamp().time_since_epoch().count());
            rosMsgs.front().header.stamp = dai_time;

        }
        },
        30,
        rightCameraInfo,
        "right");

    rightPublish.addPublisherCallback();
    leftPublish.addPublisherCallback();

    rightRectPublish.addPublisherCallback();
    leftRectPublish.addPublisherCallback();

    // RGB camera
    //auto imgQueue = device->getOutputQueue("rgb", 30, false);

    // imu
    auto imuQueue = device->getOutputQueue("imu", 30, false);

    double angularVelCovariance = 0, linearAccelCovariance = 0;
    dai::ros::ImuSyncMethod imuMode = dai::ros::ImuSyncMethod::COPY;
    dai::rosBridge::ImuConverter imuConverter("oak_imu_frame", imuMode, linearAccelCovariance, angularVelCovariance);
    //imuConverter.setUpdateRosBaseTimeOnToRosMsg(true);
    /*
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData> imuPublish(
        imuQueue,
        node,
        std::string("/oak/imu"),
        std::bind(&dai::rosBridge::ImuConverter::toRosMsg, &imuConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        "",
        "imu");
    */
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData> imuPublish(
        imuQueue,
        node,
        std::string("/oak/imu"),
        [&](std::shared_ptr<dai::IMUData> imuData, std::deque<sensor_msgs::msg::Imu> &rosMsgs) {
        imuConverter.toRosMsg(imuData, rosMsgs);
        if (!rosMsgs.empty()) {
            //rosMsgs.front().header.stamp = node->now();
            rclcpp::Time dai_time = rclcpp::Time(imuData->getTimestamp().time_since_epoch().count());
            rosMsgs.front().header.stamp = dai_time;
        }
        },
        30,
        "",
        "imu");

    imuPublish.addPublisherCallback();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

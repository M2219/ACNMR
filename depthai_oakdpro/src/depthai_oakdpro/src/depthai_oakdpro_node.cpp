#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "sensor_msgs/msg/imu.hpp"

#include "depthai/depthai.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("depthai_oakdpro_node");

    // Initialize pipeline
    std::shared_ptr<dai::Pipeline> pipeline;
    pipeline = std::make_shared<dai::Pipeline>();

    // RGB camera
    auto camRgb = pipeline->create<dai::node::ColorCamera>();
    auto xoutRgb = pipeline->create<dai::node::XLinkOut>();
    xoutRgb->setStreamName("rgb");
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    dai::node::ColorCamera::Properties::SensorResolution rgbResolution = dai::node::ColorCamera::Properties::SensorResolution::THE_4_K;
    int rgbWidth; // 1/3 1280 x 720
    int rgbHeight;

    rgbWidth = 3840;
    rgbHeight = 2160;

    camRgb->setResolution(rgbResolution);

    rgbWidth = rgbWidth * 1 / 3;
    rgbHeight = rgbHeight * 1 / 3;

    camRgb->setIspScale(1, 3);
    camRgb->isp.link(xoutRgb->input);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);

    // IMU
    auto imu = pipeline->create<dai::node::IMU>();
    auto xoutImu = pipeline->create<dai::node::XLinkOut>();
    xoutImu->setStreamName("imu");
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 200);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 200);
    imu->setBatchReportThreshold(5);
    imu->setMaxBatchReports(20);  // Get one message only for now.
    imu->out.link(xoutImu->input);
    // Create mono cameras
    auto camLeft = pipeline->create<dai::node::MonoCamera>();
    auto camRight = pipeline->create<dai::node::MonoCamera>();

    auto xoutLeft = pipeline->create<dai::node::XLinkOut>();
    auto xoutRight = pipeline->create<dai::node::XLinkOut>();

    // Set camera properties
    camLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    camLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    camLeft->setFps(30.0);

    camRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    camRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    camRight->setFps(30.0);

    // Set XLinkOut stream names
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");

    camLeft->out.link(xoutLeft->input);
    camRight->out.link(xoutRight->input);

    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>(*pipeline);
    auto calibrationHandler = device->readCalibration();

    auto leftQueue = device->getOutputQueue("left", 30, false);
    auto rightQueue = device->getOutputQueue("right", 30, false);

    // Image converters
    dai::rosBridge::ImageConverter imageConverterLeft("oak_left_camera_optical_frame", true);
    imageConverterLeft.setUpdateRosBaseTimeOnToRosMsg();

    dai::rosBridge::ImageConverter imageConverterRight("oak_right_camera_optical_frame", true);
    imageConverterRight.setUpdateRosBaseTimeOnToRosMsg();


    const std::string leftPubName =  std::string("left/image_raw");
    const std::string rightPubName = std::string("right/image_raw");

    int width = 1280;
    int height = 720;

    auto leftCameraInfo = imageConverterLeft.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_B, width, height);
    auto rightCameraInfo = imageConverterRight.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, width, height);


    // Bridge Publishers
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> leftPublish(
        leftQueue,
        node,
        leftPubName,
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &imageConverterLeft, std::placeholders::_1, std::placeholders::_2),
        30,
        leftCameraInfo,
        "left");

    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightPublish(
        rightQueue,
        node,
        rightPubName,
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &imageConverterRight, std::placeholders::_1, std::placeholders::_2),
        30,
        rightCameraInfo,
        "right");

    rightPublish.addPublisherCallback();
    leftPublish.addPublisherCallback();

    // RGB camera
    auto imgQueue = device->getOutputQueue("rgb", 30, false);

    dai::rosBridge::ImageConverter rgbConverter("oak_rgb_camera_optical_frame", false);
    rgbConverter.setUpdateRosBaseTimeOnToRosMsg();

    auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, width, height);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(
        imgQueue,
        node,
        std::string("color/image"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        rgbCameraInfo,
        "color");

    rgbPublish.addPublisherCallback();

    // imu
    auto imuQueue = device->getOutputQueue("imu", 30, false);

    double angularVelCovariance = 0, linearAccelCovariance = 0;
    dai::ros::ImuSyncMethod imuMode = dai::ros::ImuSyncMethod::COPY;
    dai::rosBridge::ImuConverter imuConverter("oak_imu_frame", imuMode, linearAccelCovariance, angularVelCovariance);
    imuConverter.setUpdateRosBaseTimeOnToRosMsg();

    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData> imuPublish(
        imuQueue,
        node,
        std::string("/oak/imu"),
        std::bind(&dai::rosBridge::ImuConverter::toRosMsg, &imuConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        "",
        "imu");

    imuPublish.addPublisherCallback();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

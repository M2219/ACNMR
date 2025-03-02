#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

#include <thread>
#include <mutex>
#include <cmath>
#include <Eigen/Dense>
#include <chrono>
#include <vector>

#include "OsqpEigen/OsqpEigen.h"
#include "mpc_controller.hpp"
#include "mpc_utils.hpp"
#include "all_config.hpp"

class MPCNode : public rclcpp::Node {
public:
    MPCNode() : Node("mpc_control_node") {
        odom_updated = false;
        path_updated = false;
        path_initialized = false;
        mpcWindow = 30;

        // Publishers
        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        target_x_pub = this->create_publisher<std_msgs::msg::Float64>("/target_x", 10);
        target_y_pub = this->create_publisher<std_msgs::msg::Float64>("/target_y", 10);
        target_yaw_pub = this->create_publisher<std_msgs::msg::Float64>("/target_yaw", 10);
        steering_pub = this->create_publisher<std_msgs::msg::Float64>("/steering", 10);
        target_time_pub = this->create_publisher<std_msgs::msg::Float64>("/target_time", 10);
        error_pub = this->create_publisher<std_msgs::msg::Float64>("/target_error", 10);
        footprint_pub = this->create_publisher<geometry_msgs::msg::PolygonStamped>("/robot_footprint", 10);
        path_pub = this->create_publisher<nav_msgs::msg::Path>("/smoothed_path", 10);
        custom_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/custom_pose", 10);

        goal_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal", 10);

        custom_pose_msg.header.stamp = this->get_clock()->now();
        custom_pose_msg.header.frame_id = "map";

        // Subscribers
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 1, std::bind(&MPCNode::odomCallback, this, std::placeholders::_1));
        global_path_sub = this->create_subscription<nav_msgs::msg::Path>(
            "/searched_path", 1, std::bind(&MPCNode::pathCallback, this, std::placeholders::_1));

        x0.setZero();
        sendInitialPose(x0(0), x0(1), x0(3), 0.5);

        goal_x = GOAL_X;
        goal_y = GOAL_Y;
        goal_yaw = GOAL_Z;

        sendGoalPose(goal_x, goal_y, goal_yaw);

        ctr << 0.0, 0.0;

        Eigen::MatrixXd xRef(4, mpcWindow + 1);
        for (int t = 0; t <= mpcWindow; t++) { //check for inside the while
            xRef(0, t) = x0(0);
            xRef(1, t) = x0(1);
            xRef(2, t) = x0(2);
            xRef(3, t) = x0(3);
        }

        yaw_comp = 0;
        dl = 0.02;

        // set MPC problem quantities for direct linearized model
        //double v = x0(2);
        //double phi = x0(3);
        //double delta = 0;
        //setDynamicsMatrices(a, b, c, v, phi, delta);

        setDynamicsMatrices(a, b, c, x0, ctr, DT);
        setInequalityConstraints(xMax, xMin, uMax, uMin);
        setWeightMatrices(Q, R);

        // cast the MPC problem as QP problem
        castMPCToQPHessian(Q, R, mpcWindow, hessian);
        castMPCToQPGradient(Q, xRef, mpcWindow, gradient);
        castMPCToQPConstraintMatrix(a, b, mpcWindow, linearMatrix);
        castMPCToQPConstraintVectors(xMax, xMin, uMax, uMin, x0, mpcWindow, lowerBound, upperBound);

        // settings
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);

        // set the initial data of the QP solver
        solver.data()->setNumberOfVariables(4 * (mpcWindow + 1) + 2 * mpcWindow);
        solver.data()->setNumberOfConstraints(2 * 4 * (mpcWindow + 1) + 2 * mpcWindow);
        solver.data()->setHessianMatrix(hessian);
        solver.data()->setGradient(gradient);
        solver.data()->setLinearConstraintsMatrix(linearMatrix);
        solver.data()->setLowerBound(lowerBound);
        solver.data()->setUpperBound(upperBound);

        // instantiate the solver
        solver.initSolver();
        // Start control thread
        control_thread = std::thread(&MPCNode::controlLoop, this);
    }

    ~MPCNode() {
        if (control_thread.joinable()) {
            control_thread.join();
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_x_pub, target_y_pub, target_yaw_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_time_pub, error_pub, steering_pub;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr custom_pose_pub, goal_pub;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub;

    std::thread control_thread;
    std::mutex odom_mutex;
    nav_msgs::msg::Path path_msg;
    geometry_msgs::msg::PoseStamped custom_pose_msg;

    // State Variables
    Eigen::Matrix<double, 4, 1> x0;
    Eigen::Matrix<double, 2, 1> ctr;
    Eigen::Matrix<double, 4, 4> a;
    Eigen::Matrix<double, 4, 2> b;
    Eigen::Matrix<double, 4, 1> c;
    Eigen::Matrix<double, 4, 1> xMax, xMin;
    Eigen::Matrix<double, 2, 1> uMax, uMin;
    Eigen::DiagonalMatrix<double, 4> Q;
    Eigen::DiagonalMatrix<double, 2> R;
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound, upperBound;
    OsqpEigen::Solver solver;
    Eigen::VectorXd QPSolution;
    bool odom_updated, path_updated, path_initialized;

    // Reference Trajectory
    std::vector<double> ccx, ccy, ccyaw, ccx_s, ccy_s, ccyaw_s, ccdir, sp, ck;
    double dl;
    double goal_x;
    double goal_y;
    double goal_yaw;
    int target_ind;
    int mpcWindow;
    double yaw_comp;
    double x_odo;
    double y_odo;
    double v_odo;
    double roll_odo, pitch_odo, yaw_odo;


    void sendInitialPose(double x, double y, double yaw, double variance) {
        auto pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        rclcpp::sleep_for(std::chrono::seconds(1));

        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";
        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        msg.pose.pose.orientation = tf2::toMsg(q);
        msg.pose.covariance[0] = msg.pose.covariance[7] = variance;
        msg.pose.covariance[35] = (M_PI / 12.0) * (M_PI / 12.0);

        pub->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    void sendGoalPose(double x, double y, double yaw) {
        rclcpp::sleep_for(std::chrono::seconds(1));

        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";
        msg.pose.position.x = x;
        msg.pose.position.y = y;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        msg.pose.orientation = tf2::toMsg(q);

        goal_pub->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    void sendCustomPoint(double x, double y, double yaw) {
        custom_pose_msg.header.stamp = this->get_clock()->now();
        custom_pose_msg.header.frame_id = "map";

        custom_pose_msg.pose.position.x = x;
        custom_pose_msg.pose.position.y = y;
        custom_pose_msg.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        custom_pose_msg.pose.orientation = tf2::toMsg(q);

        custom_pose_pub->publish(custom_pose_msg);
    }
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex);
        x0(0) = msg->pose.pose.position.x;
        x0(1) = msg->pose.pose.position.y;
        x0(2) = msg->twist.twist.linear.x;
        //std::cout << "odome" << x0 << std::endl;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        m.getRPY(x0(3), yaw_comp, yaw_comp);
        x0(3) += yaw_comp;

        odom_updated = true;
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex);

        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received an empty path.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received Path with %lu waypoints", msg->poses.size());

        ccx.clear();
        ccy.clear();
        ccyaw.clear();
        ccdir.clear();

        for (size_t i = 0; i < msg->poses.size(); i++) {
            const auto &pose = msg->poses[i].pose;
            double theta = tf2::getYaw(pose.orientation);

            ccx.push_back(pose.position.x);
            ccy.push_back(pose.position.y);
            ccyaw.push_back(theta);

            if (i == 0) {
                ccdir.push_back(1);
            } else {
                double dx = ccx[i] - ccx[i - 1];
                double dy = ccy[i] - ccy[i - 1];
                double motion_angle = atan2(dy, dx);

                double angle_diff = ccyaw[i] - motion_angle;
                while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
                while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

                ccdir.push_back((std::abs(angle_diff) > M_PI / 2) ? -1 : 1);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Stored Path with %lu waypoints.", ccx.size());

        path_updated = true;
        global_path_sub.reset();  // Receive path only once in a static environment
    }

    void publishTrajectory(double x, double y, double steering, double yaw, double ttime, double error_traj) {
        std_msgs::msg::Float64 msg_x, msg_y, msg_yaw, msg_time, msg_error, msg_steering;

        msg_x.data = x;
        msg_y.data = y;
        msg_yaw.data = yaw;
        msg_steering.data = steering;
        msg_time.data = ttime;
        msg_error.data = error_traj;

        target_x_pub->publish(msg_x);
        target_y_pub->publish(msg_y);
        target_yaw_pub->publish(msg_yaw);
        steering_pub->publish(msg_steering);
        target_time_pub->publish(msg_time);
        error_pub->publish(msg_error);
    }

    void publishFootprint() {
        geometry_msgs::msg::PolygonStamped footprint_msg;
        footprint_msg.header.stamp = this->get_clock()->now();
        footprint_msg.header.frame_id = "base_link";

        std::vector<std::vector<float>> footprint = {
            {-0.49, -0.3725}, {-0.49, 0.3725}, {0.49, 0.3725}, {0.49, -0.3725}
        };

        for (const auto &point : footprint) {
            geometry_msgs::msg::Point32 p;
            p.x = point[0];
            p.y = point[1];
            p.z = 0;
            footprint_msg.polygon.points.push_back(p);
        }

        footprint_pub->publish(footprint_msg);
    }

    void publishPath() {
        if (ccx_s.size() != ccy_s.size() || ccx_s.empty()) {
            RCLCPP_WARN(this->get_logger(), "Path vectors are empty or not the same size.");
            return;
        }

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = "map";

        for (size_t i = 0; i < ccx_s.size(); ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->get_clock()->now();
            pose.header.frame_id = "map";

            pose.pose.position.x = ccx_s[i];
            pose.pose.position.y = ccy_s[i];
            pose.pose.position.z = 0.0;

            path_msg.poses.push_back(pose);
        }

        path_pub->publish(path_msg);
    }

    void controlLoop() {
        rclcpp::Rate rate(50);
        double ttime = 0;
        while (rclcpp::ok()) {
            std::lock_guard<std::mutex> lock(odom_mutex);

            if (!odom_updated) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for odometry data...");
                //rate.sleep();
                continue;
            }

            if (!path_updated) {

                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for global path...");
                //rate.sleep();
                continue;
            }

            if (!path_initialized) {
                target_ind = 0;
                getStraightCourse(dl, ccx_s, ccy_s, ccyaw_s, ck, ccx, ccy);
                publishPath();
                sp = calcSpeedProfile(ccx_s, ccy_s, ccyaw_s, TARGET_SPEED);
                auto [target_ind, mind] = calcNearestIndex(x0, ccx_s, ccy_s, ccyaw_s, 0, N_IND_SEARCH);
                path_initialized = true;
            }

            setDynamicsMatrices(a, b, c, x0, ctr, DT);

            auto [xRef, target_ind_new, dref] = calcRefTrajectory(
                x0, ccx_s, ccy_s, ccyaw_s, sp, dl, target_ind, mpcWindow, NX, N_IND_SEARCH, DT
            );

            target_ind = target_ind_new;

            castMPCToQPGradient(Q, xRef, mpcWindow, gradient);
            solver.updateGradient(gradient);

            castMPCToQPConstraintMatrix(a, b, mpcWindow, linearMatrix);
            solver.updateLinearConstraintsMatrix(linearMatrix);

            castMPCToQPConstraintVectors(xMax, xMin, uMax, uMin, x0, mpcWindow, lowerBound, upperBound);
            updateConstraintVectors(x0, lowerBound, upperBound);
            solver.updateBounds(lowerBound, upperBound);

            if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
                RCLCPP_ERROR(this->get_logger(), "Solver Error");

            QPSolution = solver.getSolution();
            ctr = QPSolution.block(4 * (mpcWindow + 1), 0, 2, 1);

            double speed = x0(2) + ctr(0) * DT;
            x0(0) = x_odo;
            x0(1) = y_odo;
            x0(2) = v_odo;
            x0(3) = yaw_odo;

            double steering = ctr(1);

            //x0(3) += (speed / WB) * tan(ctr(1)) * DT;
            //x0(0) += speed * cos(x0(3)) * DT;
            //x0(1) += speed * sin(x0(3)) * DT;
            //x0(2) = speed;

            //x0 = a * x0 + b * ctr + c; linearized model

            // for using in direct linearized model without nonlinearity
            //v = x0(2);
            //phi = x0(3);
            //delta = ctr(1);

            if (std::hypot(x0(0) - goal_x, x0(1) - goal_y) <= GOAL_DIS && std::abs(x0(2)) < STOP_SPEED) {
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                break;
            }


            // Limit velocity and steering angle
            //double speed = state.v + dt * acceleration;
            //if (speed > max_linear_speed) speed = max_linear_speed;
            //if (speed < -max_linear_speed) speed = -max_linear_speed;

            //if (steering > max_steer_angle) steering = max_steer_angle;
            //if (steering < -max_steer_angle) steering = -max_steer_angle;

            geometry_msgs::msg::Twist cmd_msg;
            cmd_msg.linear.x = speed;
            cmd_msg.angular.z = steering;
            cmd_vel_pub->publish(cmd_msg);

            ttime += DT;
            double error_traj = std::hypot(x0(0) - ccx_s[target_ind], x0(1) - ccy_s[target_ind]);

            sendCustomPoint(x0(0), x0(1), x0(3));
            publishTrajectory(ccx_s[target_ind], ccy_s[target_ind], steering, ccyaw_s[target_ind], ttime, error_traj);
            publishFootprint();

            rate.sleep();
        }
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MPCNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <thread>
#include <mutex>
#include <cmath>
#include <Eigen/Dense>
#include <chrono>
#include <vector>
#include <cmath>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "OsqpEigen/OsqpEigen.h"
#include "mpc_controller.hpp"
#include "mpc_utils.hpp"
#include "all_config.hpp"

class MPCNode {
public:
    MPCNode() {
        odom_updated = false;
        path_updated = false;
        path_initialized = false;
        // set the preview window
        mpcWindow = 20;

        // Publishers
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        target_x_pub = nh.advertise<std_msgs::Float64>("/target_x", 1);
        target_y_pub = nh.advertise<std_msgs::Float64>("/target_y", 1);
        target_yaw_pub = nh.advertise<std_msgs::Float64>("/target_yaw", 1);
        steering_pub = nh.advertise<std_msgs::Float64>("/steering", 1);
        target_time_pub = nh.advertise<std_msgs::Float64>("/target_time", 1);
        error_pub = nh.advertise<std_msgs::Float64>("/target_error", 1);

        // Subscribers
        odom_sub = nh.subscribe("/odom", 1, &MPCNode::odomCallback, this);
        std::cout << "Starting MPC Simulation..." << std::endl;

        // allocate the initial and the reference state space
        x0 << 0, 0, 0, 0;

        global_path_sub = nh.subscribe("/run_hybrid_astar/searched_path", 10, &MPCNode::pathCallback, this);
        sendInitialPose(nh, x0(0), x0(1), x0(3), 0.5);
        goal_x = GOAL_X;
        goal_y = GOAL_Y;
        goal_yaw = GOAL_Z;
        sendGoalPose(nh, goal_x, goal_y, goal_yaw);

        ctr << 0, 0;

        Eigen::MatrixXd xRef(4, mpcWindow + 1);
        for (int t = 0; t <= mpcWindow; t++) { //check for inside the while
            xRef(0, t) = x0(0);
            xRef(1, t) = x0(1);
            xRef(2, t) = x0(2);
            xRef(3, t) = x0(3);
        }

        yaw_comp = 0; // check to if necceary
        dl = 0.05; // path ticks

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
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub, target_x_pub, target_y_pub, target_yaw_pub, target_time_pub, error_pub, steering_pub;
    ros::Subscriber odom_sub, global_path_sub;
    std::thread control_thread;
    std::mutex odom_mutex;

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

    bool odom_updated;
    bool path_updated;
    bool path_initialized;

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

    void sendInitialPose(ros::NodeHandle &nh, double x, double y, double yaw, double variance) {
        ros::Publisher init_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

        ros::Duration(1.0).sleep();
        geometry_msgs::PoseWithCovarianceStamped init_pose_msg;

        init_pose_msg.header.stamp = ros::Time::now();
        init_pose_msg.header.frame_id = "map";

        init_pose_msg.pose.pose.position.x = x;
        init_pose_msg.pose.pose.position.y = y;
        init_pose_msg.pose.pose.position.z = 0.0;

        init_pose_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        // Set covariance (optional, mainly for localization uncertainty)
        for (int i = 0; i < 36; i++) {
            init_pose_msg.pose.covariance[i] = 0.0;
        }
        init_pose_msg.pose.covariance[0] = variance;
        init_pose_msg.pose.covariance[7] = variance;
        init_pose_msg.pose.covariance[35] = (M_PI / 12.0) * (M_PI / 12.0);
        ROS_INFO("Sending Initial Pose: x=%.2f, y=%.2f, yaw=%.2f (variance=%.2f)", x, y, yaw, variance);
        init_pose_pub.publish(init_pose_msg);

        ros::Duration(1.0).sleep();
    }

    void sendGoalPose(ros::NodeHandle &nh, double x, double y, double yaw) {
        ros::Publisher goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

        ros::Duration(1.0).sleep();

        geometry_msgs::PoseStamped goal_msg;

        goal_msg.header.stamp = ros::Time::now();
        goal_msg.header.frame_id = "map";

        goal_msg.pose.position.x = x;
        goal_msg.pose.position.y = y;
        goal_msg.pose.position.z = 0.0;

        goal_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        ROS_INFO("Sending Goal Pose: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);
        goal_pose_pub.publish(goal_msg);

        ros::Duration(1.0).sleep();
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

        x_odo = msg->pose.pose.position.x;
        y_odo = msg->pose.pose.position.y;
        v_odo = msg->twist.twist.linear.x;

        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf::Matrix3x3 m(q);
        m.getRPY(roll_odo, pitch_odo, yaw_odo);
        yaw_odo = yaw_odo + yaw_comp;

        odom_updated = true;
    }

    // Callback function to store path data
    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {

    if (msg->poses.empty()) {
        ROS_WARN("Received an empty path.");
        return;
    }
        ROS_INFO("Received Path with %lu waypoints", msg->poses.size());

        ccx.clear();
        ccy.clear();
        ccyaw.clear();
        ccdir.clear();

        for (size_t i = 0; i < msg->poses.size(); i++) {
            const auto& pose = msg->poses[i].pose;
            double theta = tf::getYaw(pose.orientation);

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

        ROS_INFO("Stored Path:");
        for (size_t i = 0; i < ccx.size(); i++) {
            ROS_INFO("Waypoint %lu: x=%f, y=%f, yaw=%f, dir=%f", i, ccx[i], ccy[i], ccyaw[i], ccdir[i]);
        }

        ROS_INFO("Path stored successfully! Total waypoints: %lu", ccx.size());

        path_updated = true;
        global_path_sub.shutdown();  //recieve once for static environment
    }

    void publishTrajectory(double x, double y, double steering, double yaw, double ttime, double error_traj) {
        std_msgs::Float64 msg_x, msg_y, msg_yaw, msg_time, msg_error, msg_steering;
        msg_x.data = x;
        msg_y.data = y;
        msg_yaw.data = yaw;
        msg_steering.data = steering;
        msg_time.data = ttime;
        msg_error.data = error_traj;

        target_x_pub.publish(msg_x);
        target_y_pub.publish(msg_y);
        target_yaw_pub.publish(msg_yaw);
        steering_pub.publish(msg_steering);
        target_time_pub.publish(msg_time);
        error_pub.publish(msg_error);
    }

    void controlLoop() {
        ros::Rate rate(50);  // 100 Hz for sim 50 for real

        double ttime = 0;
        while (ros::ok()) {

            std::lock_guard<std::mutex> lock(odom_mutex);

            if (!odom_updated) {
                ROS_WARN_THROTTLE(1, "Waiting for odometry data...");
                rate.sleep();
                continue;
            }

            if (!path_updated) {
                ROS_WARN_THROTTLE(1, "Waiting for global path ...");
                rate.sleep();
                continue;
            }

            if (!path_initialized) {

                target_ind = 0;
                getStraightCourse(dl, ccx_s, ccy_s, ccyaw_s, ck, ccx, ccy);
                sp = calcSpeedProfile(ccx_s, ccy_s, ccyaw_s, TARGET_SPEED);
                auto [target_ind, mind] = calcNearestIndex(x0, ccx_s, ccy_s, ccyaw_s, 0, N_IND_SEARCH);
                //smoothYaw(ccyaw);
                //smoothYawMovingAverage(cyaw);
                //smoothYawKalman(cyaw);
                //smoothYawSavitzkyGolay(cyaw);
                path_initialized = true;

            }

            // Get reference trajectory for MPC
            // direct linearization
            //setDynamicsMatrices(a, b, c, v, phi, delta);

            //RK4
            setDynamicsMatrices(a, b, c, x0, ctr, DT);

            auto [xRef, target_ind_new, dref] = calcRefTrajectory(x0, ccx_s, ccy_s, ccyaw_s, sp, dl, target_ind, mpcWindow, NX, N_IND_SEARCH, DT);
            target_ind = target_ind_new;

            castMPCToQPGradient(Q, xRef, mpcWindow, gradient);
            solver.updateGradient(gradient);

            castMPCToQPConstraintMatrix(a, b, mpcWindow, linearMatrix);
            solver.updateLinearConstraintsMatrix(linearMatrix);

            castMPCToQPConstraintVectors(xMax, xMin, uMax, uMin, x0, mpcWindow, lowerBound, upperBound);
            updateConstraintVectors(x0, lowerBound, upperBound);
            solver.updateBounds(lowerBound, upperBound);

            // solve the QP problem
            if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
                std::cout << "Solver Error" << std::endl;

            // get the controller in	put
            QPSolution = solver.getSolution();
            ctr = QPSolution.block(4 * (mpcWindow + 1), 0, 2, 1);

            // propagate the model
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

            // break command
            if (std::hypot(x0(0) - goal_x, x0(1) - goal_y) <= GOAL_DIS && std::abs(x0(2)) < STOP_SPEED) {
                std::cout << "Goal reached!" << std::endl;
                break;
            }


            // Limit velocity and steering angle
            //double speed = state.v + dt * acceleration;
            //if (speed > max_linear_speed) speed = max_linear_speed;
            //if (speed < -max_linear_speed) speed = -max_linear_speed;

            //if (steering > max_steer_angle) steering = max_steer_angle;
            //if (steering < -max_steer_angle) steering = -max_steer_angle;

            // Publish control command
            geometry_msgs::Twist cmd_msg;
            cmd_msg.linear.x = speed;
            cmd_msg.angular.z = steering; // -steering for the real mobile
            cmd_vel_pub.publish(cmd_msg);

            // Publish trajectory point
            ttime = ttime + DT;
            double error_traj = sqrt((x0(0) - ccx_s[target_ind])*(x0(0) - ccx_s[target_ind]) + (x0(1) - ccy_s[target_ind]) * (x0(1) - ccy_s[target_ind]));
            //ROS_INFO("Control loop using odometry: x=%f, y=%f, yaw=%f, v=%f, e=%f", x_odo, y_odo, yaw_odo, v_odo, error_traj);

            publishTrajectory(ccx_s[target_ind], ccy_s[target_ind], steering, ccyaw_s[target_ind], ttime, error_traj);

            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_control_node");
    MPCNode node;
    ros::spin();
    return 0;
}


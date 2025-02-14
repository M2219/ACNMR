#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
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
#include "cubic_planner.hpp"
#include "mpc_utils.hpp"

class MPCNode {
public:
    MPCNode() {
        // Initialize ROS NodeHandle
        odom_updated = false;
        target_ind = 0;
        // Publishers
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        target_x_pub = nh.advertise<std_msgs::Float64>("/target_x", 1);
        target_y_pub = nh.advertise<std_msgs::Float64>("/target_y", 1);
        target_yaw_pub = nh.advertise<std_msgs::Float64>("/target_yaw", 1);
        target_time_pub = nh.advertise<std_msgs::Float64>("/target_time", 1);

        // Subscribers
        odom_sub = nh.subscribe("/odom", 1, &MPCNode::odomCallback, this);

        std::cout << "Starting MPC Simulation..." << std::endl;

        // allocate the initial and the reference state space
        x0 << 0, 0, 0, 0;
        ctr << 0, 0;
        dl = 0.01;  // Course tick

        //getStraightCourse(dl, cx, cy, cyaw, ck);
        //getStraightCourse2(dl, cx, cy, cyaw, ck);
        getStraightCourse3(dl, cx, cy, cyaw, ck);
        //getForwardCourse(dl, cx, cy, cyaw, ck);
        //getSwitchBackCourse(dl, cx, cy, cyaw, ck);
        //x0 << 2, 0, 0, 0; // start and start is the same

        sp = calcSpeedProfile(cx, cy, cyaw, TARGET_SPEED);

        // set the preview window
        mpcWindow = 5;

        // set the initial and the desired states
        auto [target_ind, mind] = calcNearestIndex(x0, cx, cy, cyaw, 0, N_IND_SEARCH);

        yaw_comp = 0;
        if (x0(3) - cyaw[0] >= M_PI) {
           x0(3) -= 2 * M_PI;
           yaw_comp = -M_PI * 2.0;
        } else if (x0(3) - cyaw[0] <= -M_PI) {
           x0(3) += 2 * M_PI;
           yaw_comp = M_PI * 2.0;
        } else {
           yaw_comp = 0;
        }

        double goal_x = cx.back(), goal_y = cy.back();

        smoothYaw(cyaw);
        //smoothYawMovingAverage(cyaw);
        //smoothYawKalman(cyaw);
        //smoothYawSavitzkyGolay(cyaw);

        Eigen::MatrixXd xRef(4, mpcWindow + 1);
        for (int t = 0; t <= mpcWindow; t++) {
            xRef(0, t) = cx[t];
            xRef(1, t) = cy[t];
            xRef(2, t) = sp[t];
            xRef(3, t) = cyaw[t];
        }
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
    ros::Publisher cmd_vel_pub, target_x_pub, target_y_pub, target_yaw_pub, target_time_pub;
    ros::Subscriber odom_sub;
    ros::NodeHandle nh;
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

    // Reference Trajectory
    std::vector<double> cx, cy, cyaw, ck, sp;
    double dl;
    int target_ind;
    int mpcWindow;
    double yaw_comp;
    double x_odo;
    double y_odo;
    double v_odo;
    double roll_odo, pitch_odo, yaw_odo;

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

    void publishTrajectory(double x, double y, double yaw, double ttime) {
        std_msgs::Float64 msg_x, msg_y, msg_yaw, msg_time;
        msg_x.data = x;
        msg_y.data = y;
        msg_yaw.data = yaw;
        msg_time.data = ttime;

        target_x_pub.publish(msg_x);
        target_y_pub.publish(msg_y);
        target_yaw_pub.publish(msg_yaw);
        target_time_pub.publish(msg_time);
    }

    void controlLoop() {
        ros::Rate rate(50);  // 50 Hz
        double ttime = 0;
        while (ros::ok()) {

            std::lock_guard<std::mutex> lock(odom_mutex);

            if (!odom_updated) {
                ROS_WARN_THROTTLE(1, "Waiting for odometry data...");
                rate.sleep();
                continue;
            }

            // Get reference trajectory for MPC
            // direct linearization
            //setDynamicsMatrices(a, b, c, v, phi, delta);

            //RK4
            setDynamicsMatrices(a, b, c, x0, ctr, DT);

            auto [xRef, target_ind_new, dref] = calcRefTrajectory(x0, cx, cy, cyaw, ck, sp, dl, target_ind, mpcWindow, NX, N_IND_SEARCH, DT);
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

            ROS_INFO("Control loop using odometry: x=%f, y=%f, yaw=%f, v=%f", x_odo, y_odo, yaw_odo, v_odo);

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
            //if (std::hypot(x0(0) - goal_x, x0(1) - goal_y) <= GOAL_DIS && std::abs(x0(2)) < STOP_SPEED) {
            //    std::cout << "Goal reached!" << std::endl;
            //    break;
            //}


            // Limit velocity and steering angle
            //double speed = state.v + dt * acceleration;
            //if (speed > max_linear_speed) speed = max_linear_speed;
            //if (speed < -max_linear_speed) speed = -max_linear_speed;

            //if (steering > max_steer_angle) steering = max_steer_angle;
            //if (steering < -max_steer_angle) steering = -max_steer_angle;

            // Publish control command
            geometry_msgs::Twist cmd_msg;
            cmd_msg.linear.x = speed;
            cmd_msg.angular.z = steering; // -steering for the real robot
            cmd_vel_pub.publish(cmd_msg);

            // Publish trajectory point
            ttime = ttime + DT;
            publishTrajectory(cx[target_ind], cy[target_ind], steering, ttime);

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

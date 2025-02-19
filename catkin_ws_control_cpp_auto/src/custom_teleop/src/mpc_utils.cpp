#include <tuple>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <Eigen/Dense>

#include "mpc_utils.hpp"
#include "mpc_controller.hpp"
#include "cubic_planner.hpp"

void getStraightCourse(double dl, std::vector<double>& cx, std::vector<double>& cy,
                       std::vector<double>& cyaw, std::vector<double>& ck, std::vector<double> xx, std::vector<double> yy) {

    std::vector<double> s_values;
    std::tie(cx, cy, cyaw, ck, s_values) = calcSplineCourse(xx, yy, dl);

}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>>
calcSplineCourse(const std::vector<double>& x, const std::vector<double>& y, double ds) {
    CubicSpline2D sp(x, y);

    double s_max = sp.s.back();

    std::vector<double> s_values;
    for (double s = 0.0; s <= s_max; s += ds) {
        s_values.push_back(s);
    }

    std::vector<double> rx, ry, ryaw, rk;


    for (double s : s_values) {
        auto [ix, iy] = sp.calcPosition(s);
        double yaw = sp.calcYaw(s);
        double curvature = sp.calcCurvature(s);

        rx.push_back(ix);
        ry.push_back(iy);
        ryaw.push_back(yaw);
        rk.push_back(curvature);
    }

    return {rx, ry, ryaw, rk, s_values};
}

double angleMod(double x, bool zero_2_2pi, bool degree) {
    degree = 0;
    zero_2_2pi = 0;
    if (degree) {
        x = x * M_PI / 180.0;
    }

    double mod_angle;
    if (zero_2_2pi) {
        mod_angle = fmod(x, 2 * M_PI);
        if (mod_angle < 0) mod_angle += 2 * M_PI;
    } else {
        mod_angle = fmod(x + M_PI, 2 * M_PI) - M_PI;
    }

    if (degree) {
        mod_angle = mod_angle * 180.0 / M_PI;
    }

    return mod_angle;
}

double pi2pi(double angle) {
    return angleMod(angle);
}

// Generate a speed profile based on a given trajectory
std::vector<double> calcSpeedProfile(const std::vector<double>& cx, const std::vector<double>& cy,
                                     const std::vector<double>& cyaw, double target_speed) {
    std::vector<double> speed_profile(cx.size(), target_speed);
    double direction = 1.0;

    for (size_t i = 0; i < cx.size() - 1; ++i) {
        double dx = cx[i + 1] - cx[i];
        double dy = cy[i + 1] - cy[i];

        double move_direction = atan2(dy, dx);
        double dangle = std::abs(pi2pi(move_direction - cyaw[i]));

        if (dx != 0.0 && dy != 0.0) {
            direction = (dangle >= M_PI / 4.0) ? -1.0 : 1.0;
        }

        speed_profile[i] = direction * target_speed;
    }

    speed_profile.back() = 0.0; // Stop at the final point
    return speed_profile;
}
/*
// Smooth yaw values to avoid discontinuities
void smoothYaw(std::vector<double>& yaw) {
    for (size_t i = 0; i < yaw.size() - 1; ++i) {
        double dyaw = yaw[i + 1] - yaw[i];

        while (dyaw >= M_PI) {
            yaw[i + 1] -= 2.0 * M_PI;
            dyaw = yaw[i + 1] - yaw[i];
        }

        while (dyaw <= -M_PI) {
            yaw[i + 1] += 2.0 * M_PI;
            dyaw = yaw[i + 1] - yaw[i];
        }
    }
}
*/
void smoothYaw(std::vector<double>& yaw) {
    if (yaw.empty()) return;

    double prev_yaw = yaw[0];
    for (size_t i = 1; i < yaw.size(); ++i) {
        double delta_yaw = yaw[i] - prev_yaw;

        while (delta_yaw > M_PI) {
            yaw[i] -= 2.0 * M_PI;
            delta_yaw = yaw[i] - prev_yaw;
        }
        while (delta_yaw < -M_PI) {
            yaw[i] += 2.0 * M_PI;
            delta_yaw = yaw[i] - prev_yaw;
        }

        prev_yaw = yaw[i];
    }
}

void smoothYawMovingAverage(std::vector<double>& yaw, int window_size) {
    if (yaw.size() < window_size) return;

    std::vector<double> smoothed_yaw(yaw.size());

    for (size_t i = 0; i < yaw.size(); ++i) {
        double sum = 0.0;
        int count = 0;

        for (int j = -window_size / 2; j <= window_size / 2; ++j) {
            int idx = i + j;
            if (idx >= 0 && idx < yaw.size()) {
                sum += yaw[idx];
                count++;
            }
        }
        smoothed_yaw[i] = sum / count;
    }

    yaw = smoothed_yaw;  // Apply smoothed values back to original vector
}

void smoothYawKalman(std::vector<double>& yaw, double process_noise, double measurement_noise) {
    if (yaw.empty()) return;

    double estimated_yaw = yaw[0];
    double error_cov = 1.0; // Initial estimate error covariance

    for (size_t i = 1; i < yaw.size(); ++i) {
        // Prediction step
        estimated_yaw = estimated_yaw;  // Assume no external control input
        error_cov += process_noise;

        // Correction step
        double kalman_gain = error_cov / (error_cov + measurement_noise);
        estimated_yaw += kalman_gain * (yaw[i] - estimated_yaw);
        error_cov *= (1 - kalman_gain);

        yaw[i] = estimated_yaw;  // Update the yaw value
    }
}

void smoothYawSavitzkyGolay(std::vector<double>& yaw, int window_size, int poly_order) {
    if (yaw.size() < window_size || poly_order >= window_size) return;

    std::vector<double> smoothed_yaw(yaw.size());
    Eigen::VectorXd coeffs = Eigen::VectorXd::Zero(poly_order + 1);

    for (size_t i = 0; i < yaw.size(); ++i) {
        int half_window = window_size / 2;
        int start = std::max(0, static_cast<int>(i) - half_window);
        int end = std::min(static_cast<int>(yaw.size()) - 1, static_cast<int>(i) + half_window);

        Eigen::MatrixXd A(end - start + 1, poly_order + 1);
        Eigen::VectorXd Y(end - start + 1);

        for (int j = start; j <= end; ++j) {
            for (int p = 0; p <= poly_order; ++p) {
                A(j - start, p) = std::pow(j - i, p);
            }
            Y(j - start) = yaw[j];
        }

        Eigen::VectorXd X = (A.transpose() * A).ldlt().solve(A.transpose() * Y);
        smoothed_yaw[i] = X(0);
    }

    yaw = smoothed_yaw;  // Apply smoothed values back
}

// Check if the vehicle has reached the goal
bool checkGoal(double x, double y, double v, double goal_x, double goal_y, double GOAL_DIS, double STOP_SPEED) {
    double dx = x - goal_x;
    double dy = y - goal_y;
    double d = hypot(dx, dy);

    bool is_goal = (d <= GOAL_DIS);
    bool is_stop = (std::abs(v) <= STOP_SPEED);

    return is_goal && is_stop;
}

// Find the nearest index in the trajectory to the vehicle
std::pair<int, double> calcNearestIndex(const Eigen::Matrix<double, 4, 1>& x0,
                                        const std::vector<double>& cx,
                                        const std::vector<double>& cy,
                                        const std::vector<double>& cyaw,
                                        int start_index, int search_range) {
    int end_index = std::min(start_index + search_range, static_cast<int>(cx.size()));
    // Compute squared distances
    std::vector<double> d(end_index - start_index);

    for (int i = start_index; i < end_index; ++i) {
        double dx = x0(0) - cx[i];
        double dy = x0(1) - cy[i];
        d[i - start_index] = dx * dx + dy * dy;
    }

    // Find the index of the minimum distance
    auto min_it = std::min_element(d.begin(), d.end());
    int nearest_index = std::distance(d.begin(), min_it) + start_index;
    double mind = std::sqrt(*min_it);

    // Compute angle correction
    double dxl = cx[nearest_index] - x0(0);
    double dyl = cy[nearest_index] - x0(1);
    double angle = pi2pi(cyaw[nearest_index] - std::atan2(dyl, dxl));

    // Apply sign correction
    if (angle < 0) {
        mind *= -1;
    }

    return {nearest_index, mind};
}


std::tuple<Eigen::MatrixXd, int, Eigen::MatrixXd>
calcRefTrajectory(const Eigen::Matrix<double, 4, 1>& x0,
                  const std::vector<double>& cx,
                  const std::vector<double>& cy,
                  const std::vector<double>& cyaw,
                  const std::vector<double>& sp,
                  double dl,
                  int pind,
                  int T,
                  int NX,
                  int N_IND_SEARCH,
                  double DT) {

    int ncourse = cx.size();

    // Initialize xref (NX x (T+1)) and dref (1 x T)
    Eigen::MatrixXd xref = Eigen::MatrixXd::Zero(NX, T + 1);
    Eigen::MatrixXd dref = Eigen::MatrixXd::Zero(1, T);  // Steering operational points

    auto [ind, _] = calcNearestIndex(x0, cx, cy, cyaw, pind, N_IND_SEARCH);
    // Ensure we do not move backward
    if (pind >= ind) {
        ind = pind;
    }

    // Set initial reference state
    xref(0, 0) = cx[ind];
    xref(1, 0) = cy[ind];
    xref(2, 0) = sp[ind];
    xref(3, 0) = cyaw[ind];
    dref(0, 0) = 0.0;

    double travel = 0.0;

    for (int i = 1; i <= T; ++i) {
        travel += std::abs(x0(2)) * DT;
        int dind = static_cast<int>(std::round(travel / dl));

        if ((ind + dind) < ncourse) {
            xref(0, i) = cx[ind + dind];
            xref(1, i) = cy[ind + dind];
            xref(2, i) = sp[ind + dind];
            xref(3, i) = cyaw[ind + dind];
            dref(0, i - 1) = 0.0;
        } else {
            xref(0, i) = cx[ncourse - 1];
            xref(1, i) = cy[ncourse - 1];
            xref(2, i) = sp[ncourse - 1];
            xref(3, i) = cyaw[ncourse - 1];
            dref(0, i - 1) = 0.0;
        }
    }

    return {xref, ind, dref};
}



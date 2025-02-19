#ifndef MPC_UTILS_H
#define MPC_UTILS_H

#include <Eigen/Dense>
#include <vector>
#include <cmath>

void getStraightCourse(double dl, std::vector<double>& cx, std::vector<double>& cy,
                       std::vector<double>& cyaw, std::vector<double>& ck, std::vector<double> xx, std::vector<double> yy);

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>>
calcSplineCourse(const std::vector<double>& x, const std::vector<double>& y, double ds);
// Angle normalization functions
double angleMod(double x, bool zero_2_2pi = false, bool degree = false);
double pi2pi(double angle);

// Speed profile calculation
std::vector<double> calcSpeedProfile(const std::vector<double>& cx, const std::vector<double>& cy,
                                     const std::vector<double>& cyaw, double target_speed);

// Yaw smoothing
void smoothYaw(std::vector<double>& yaw);
void smoothYawMovingAverage(std::vector<double>& yaw, int window_size = 5);
void smoothYawKalman(std::vector<double>& yaw, double process_noise = 0.1, double measurement_noise = 0.1);
void smoothYawSavitzkyGolay(std::vector<double>& yaw, int window_size = 5, int poly_order = 2);


// Goal checking function
bool checkGoal(double x, double y, double v, double goal_x, double goal_y, double GOAL_DIS, double STOP_SPEED);

// Nearest index calculation
std::pair<int, double> calcNearestIndex(const Eigen::Matrix<double, 4, 1>& x0,
                                        const std::vector<double>& cx,
                                        const std::vector<double>& cy,
                                        const std::vector<double>& cyaw,
                                        int start_index,
                                        int search_range);

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
                  double DT);

#endif  // MPC_UTILS_H

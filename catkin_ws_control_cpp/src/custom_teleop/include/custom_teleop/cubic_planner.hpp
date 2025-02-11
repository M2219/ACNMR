#ifndef CUBIC_SPLINE_2D_HPP
#define CUBIC_SPLINE_2D_HPP

#include <vector>
#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

class CubicSpline1D {
public:
    CubicSpline1D(const std::vector<double>& x, const std::vector<double>& y);

    double calcPosition(double x);
    double calcFirstDerivative(double x);
    double calcSecondDerivative(double x);
    double calcThirdDerivative(double x);

    std::vector<double> a, b, c, d, x_values;
    int nx;

    int searchIndex(double x);
    Eigen::MatrixXd calcA(const std::vector<double>& h);
    Eigen::VectorXd calcB(const std::vector<double>& h, const std::vector<double>& a);
};


class CubicSpline2D {
public:
    CubicSpline2D(const std::vector<double>& x, const std::vector<double>& y);

    std::pair<double, double> calcPosition(double s);
    double calcCurvature(double s);
    double calcCurvatureRate(double s);
    double calcYaw(double s);

    std::vector<double> s; // Arc-length parameter
    CubicSpline1D sx, sy;

    std::vector<double> calcS(const std::vector<double>& x, const std::vector<double>& y);
};

#endif // CUBIC_SPLINE_2D_HPP

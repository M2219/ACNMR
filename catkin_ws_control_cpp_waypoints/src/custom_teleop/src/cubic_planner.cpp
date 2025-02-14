#include "cubic_planner.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include "cubic_planner.hpp"


CubicSpline1D::CubicSpline1D(const std::vector<double>& x, const std::vector<double>& y) {
    if (x.size() != y.size()) {
        throw std::invalid_argument("x and y vectors must have the same size.");
    }

    nx = x.size();
    x_values = x;
    a = y;

    std::vector<double> h(nx - 1);
    for (int i = 0; i < nx - 1; ++i) {
        h[i] = x[i + 1] - x[i];
        if (h[i] < 0) {
            throw std::invalid_argument("x coordinates must be sorted in ascending order.");
        }
    }

    // Compute coefficient c using a linear system
    Eigen::MatrixXd A = calcA(h);
    Eigen::VectorXd B = calcB(h, a);
    Eigen::VectorXd c_vec = A.colPivHouseholderQr().solve(B);

    c.assign(c_vec.data(), c_vec.data() + c_vec.size());

    // Compute coefficients b and d
    for (int i = 0; i < nx - 1; ++i) {
        double d_val = (c[i + 1] - c[i]) / (3.0 * h[i]);
        double b_val = (a[i + 1] - a[i]) / h[i] - h[i] * (2.0 * c[i] + c[i + 1]) / 3.0;
        d.push_back(d_val);
        b.push_back(b_val);
    }
}

double CubicSpline1D::calcPosition(double x) {
    if (x < x_values.front() || x > x_values.back()) {
        return NAN;
    }

    int i = searchIndex(x);
    double dx = x - x_values[i];

    return a[i] + b[i] * dx + c[i] * dx * dx + d[i] * dx * dx * dx;
}

double CubicSpline1D::calcFirstDerivative(double x) {
    if (x < x_values.front() || x > x_values.back()) {
        return NAN;
    }

    int i = searchIndex(x);
    double dx = x - x_values[i];

    return b[i] + 2.0 * c[i] * dx + 3.0 * d[i] * dx * dx;
}

double CubicSpline1D::calcSecondDerivative(double x) {
    if (x < x_values.front() || x > x_values.back()) {
        return NAN;
    }

    int i = searchIndex(x);
    double dx = x - x_values[i];

    return 2.0 * c[i] + 6.0 * d[i] * dx;
}

double CubicSpline1D::calcThirdDerivative(double x) {
    if (x < x_values.front() || x > x_values.back()) {
        return NAN;
    }

    int i = searchIndex(x);
    return 6.0 * d[i];
}

int CubicSpline1D::searchIndex(double x) {
    auto it = std::upper_bound(x_values.begin(), x_values.end(), x);
    return std::max(0, static_cast<int>(it - x_values.begin()) - 1);
}

Eigen::MatrixXd CubicSpline1D::calcA(const std::vector<double>& h) {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nx, nx);
    A(0, 0) = 1.0;
    A(nx - 1, nx - 1) = 1.0;

    for (int i = 0; i < nx - 2; ++i) {
        A(i + 1, i) = h[i];
        A(i + 1, i + 1) = 2.0 * (h[i] + h[i + 1]);
        A(i + 1, i + 2) = h[i + 1];
    }

    return A;
}

Eigen::VectorXd CubicSpline1D::calcB(const std::vector<double>& h, const std::vector<double>& a) {
    Eigen::VectorXd B = Eigen::VectorXd::Zero(nx);

    for (int i = 0; i < nx - 2; ++i) {
        B(i + 1) = 3.0 * ((a[i + 2] - a[i + 1]) / h[i + 1] - (a[i + 1] - a[i]) / h[i]);
    }

    return B;
}


CubicSpline2D::CubicSpline2D(const std::vector<double>& x, const std::vector<double>& y)
    : s(calcS(x, y)), sx(s, x), sy(s, y) {}

std::vector<double> CubicSpline2D::calcS(const std::vector<double>& x, const std::vector<double>& y) {
    std::vector<double> s_values;
    s_values.push_back(0.0);

    for (size_t i = 1; i < x.size(); ++i) {
        double dx = x[i] - x[i - 1];
        double dy = y[i] - y[i - 1];
        s_values.push_back(s_values.back() + std::hypot(dx, dy));
    }

    return s_values;
}

std::pair<double, double> CubicSpline2D::calcPosition(double s_query) {
    double x = sx.calcPosition(s_query);
    double y = sy.calcPosition(s_query);
    return {x, y};
}

double CubicSpline2D::calcCurvature(double s_query) {
    double dx = sx.calcFirstDerivative(s_query);
    double dy = sy.calcFirstDerivative(s_query);
    double ddx = sx.calcSecondDerivative(s_query);
    double ddy = sy.calcSecondDerivative(s_query);

    return (ddy * dx - ddx * dy) / std::pow(dx * dx + dy * dy, 1.5);
}

double CubicSpline2D::calcCurvatureRate(double s_query) {
    double dx = sx.calcFirstDerivative(s_query);
    double dy = sy.calcFirstDerivative(s_query);
    double ddx = sx.calcSecondDerivative(s_query);
    double ddy = sy.calcSecondDerivative(s_query);
    double dddx = sx.calcThirdDerivative(s_query);
    double dddy = sy.calcThirdDerivative(s_query);

    double a = dx * ddy - dy * ddx;
    double b = dx * dddy - dy * dddx;
    double c = dx * ddx + dy * ddy;
    double d = dx * dx + dy * dy;

    return (b * d - 3.0 * a * c) / (d * d * d);
}

double CubicSpline2D::calcYaw(double s_query) {
    double dx = sx.calcFirstDerivative(s_query);
    double dy = sy.calcFirstDerivative(s_query);
    return std::atan2(dy, dx);
}

/*
namespace plt = matplotlibcpp;

int main() {
    std::cout << "Cubic Spline 2D Test" << std::endl;

    // Define waypoints (like your Python example)
    std::vector<double> x = {-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0};
    std::vector<double> y = {0.7, -6, 5, 6.5, 0.0, 5.0, -2.0};
    double ds = 0.1; // Distance step for interpolation

    // Fit cubic spline
    CubicSpline2D sp(x, y);
    double s_max = sp.s.back(); // Length of the fitted spline

    // Generate interpolated points
    std::vector<double> rx, ry, ryaw, rk, s_values;
    for (double i_s = 0; i_s <= s_max; i_s += ds) {
        auto [ix, iy] = sp.calcPosition(i_s);
        rx.push_back(ix);
        ry.push_back(iy);
        ryaw.push_back(sp.calcYaw(i_s) * 180.0 / M_PI); // Convert to degrees
        rk.push_back(sp.calcCurvature(i_s));
        s_values.push_back(i_s);
    }

    // Plot trajectory
    plt::figure();
    plt::plot(x, y, "xb");
    plt::plot(rx, ry, "-r");
    plt::grid(true);
    plt::axis("equal");
    plt::xlabel("X [m]");
    plt::ylabel("Y [m]");
    plt::title("Cubic Spline 2D Path");

    // Plot yaw angle
    plt::figure();
    plt::plot(s_values, ryaw, "r-");
    plt::grid(true);
    plt::xlabel("Arc Length [m]");
    plt::ylabel("Yaw [deg]");
    plt::title("Yaw Along the Path");

    // Plot curvature
    plt::figure();
    plt::plot(s_values, rk, "r-");
    plt::grid(true);
    plt::xlabel("Arc Length [m]");
    plt::ylabel("Curvature [1/m]");
    plt::title("Curvature Along the Path");

    plt::show();
    return 0;
}
*/

#include <iostream>
#include <Eigen/Dense>

#include "mpc_controller.hpp"
#include "OsqpEigen/OsqpEigen.h"

void dynamics_function_(const Eigen::Matrix<double, 4, 1>& x,
                        Eigen::Matrix<double, 4, 1>& x_dot,
                        const Eigen::Matrix<double, 2, 1>& u) {
    double v = x(2);
    double phi = x(3);
    double a = u(0);
    double delta = u(1);

    x_dot(0) = v * cos(phi);
    x_dot(1) = v * sin(phi);
    x_dot(2) = a;
    x_dot(3) = (v / WB) * tan(delta);
}

void getLinearizedDynamicsContinuous(Eigen::Matrix<double, 4, 4>& F,
                                     Eigen::Matrix<double, 4, 2>& G,
                                     Eigen::Matrix<double, 4, 1>& C,
                                     const Eigen::Matrix<double, 4, 1>& x,
                                     const Eigen::Matrix<double, 2, 1>& u)
{
    double v = x(2);
    double theta = x(3);
    double delta = u(1);

    F.setZero();
    F(0, 2) = cos(theta);
    F(0, 3) = -v * sin(theta);
    F(1, 2) = sin(theta);
    F(1, 3) = v *  cos(theta);
    F(2, 2) = 0;
    F(3, 2) = tan(delta) / WB;

    G.setZero();
    G(2, 0) = 1.0;
    G(3, 1) = v / (WB * pow(cos(delta), 2)); // G(3, 1) = v / WB * (1 + delta * delta); // taylor appx

    // Compute Nonlinear Compensation Term (C)
    C.setZero();
    C(0) = v *  sin(theta) * theta;
    C(1) = -v *  cos(theta) * theta;
    C(3) = -v * delta / (WB * pow(cos(delta), 2));

}

void setDynamicsMatrices(Eigen::Matrix<double, 4, 4>& A,
                         Eigen::Matrix<double, 4, 2>& B,
                         Eigen::Matrix<double, 4, 1>& C,
                         const Eigen::Matrix<double, 4, 1>& x,
                         const Eigen::Matrix<double, 2, 1>& u,
                         double sample_period) {
    // Initialize RK4 variables
    Eigen::Matrix<double, 4, 1> x_dot, dx;
    Eigen::Matrix<double, 4, 4> F, dA1, dA2, dA3, dA4;
    Eigen::Matrix<double, 4, 2> G, dB1, dB2, dB3, dB4;
    Eigen::Matrix<double, 4, 1> C1, C2, C3, C4;

    // Identity initialization
    A.setIdentity();
    B.setZero();
    C.setZero();

    // RK4 integration step size
    double rk_delta_t = sample_period / 4.0;  // 4 RK steps

    // Step 1: Compute dynamics at x
    dynamics_function_(x, x_dot, u);  // Nonlinear state derivative
    getLinearizedDynamicsContinuous(F, G, C1, x, u);

    dx = x_dot * rk_delta_t;
    dA1 = F * A * rk_delta_t;
    dB1 = (F * B + G) * rk_delta_t;
    C1 *= rk_delta_t;

    // Step 2: Compute dynamics at x + 0.5*dx
    dynamics_function_(x + 0.5 * dx, x_dot, u);
    getLinearizedDynamicsContinuous(F, G, C2, x + 0.5 * dx, u);

    dx = x_dot * rk_delta_t;
    dA2 = F * (A + 0.5 * dA1) * rk_delta_t;
    dB2 = (F * (B + 0.5 * dB1) + G) * rk_delta_t;
    C2 *= rk_delta_t;

    // Step 3: Compute dynamics at x + 0.5*dx
    dynamics_function_(x + 0.5 * dx, x_dot, u);
    getLinearizedDynamicsContinuous(F, G, C3, x + 0.5 * dx, u);

    dx = x_dot * rk_delta_t;
    dA3 = F * (A + 0.5 * dA2) * rk_delta_t;
    dB3 = (F * (B + 0.5 * dB2) + G) * rk_delta_t;
    C3 *= rk_delta_t;

    // Step 4: Compute dynamics at x + dx
    dynamics_function_(x + dx, x_dot, u);
    getLinearizedDynamicsContinuous(F, G, C4, x + dx, u);

    dx = x_dot * rk_delta_t;
    dA4 = F * (A + dA3) * rk_delta_t;
    dB4 = (F * (B + dB3) + G) * rk_delta_t;
    C4 *= rk_delta_t;

    // Final RK4 update
    A += (dA1 + 2 * (dA2 + dA3) + dA4) / 6;
    B += (dB1 + 2 * (dB2 + dB3) + dB4) / 6;
    C += (C1 + 2 * (C2 + C3) + C4) / 6;  // Nonlinear compensation
}
/*
// Direct linearized model (faster and less accurate)
void setDynamicsMatrices(Eigen::Matrix<double, 4, 4>& a, Eigen::Matrix<double, 4, 2>& b, Eigen::Matrix<double, 4, 1>& c, double v, double phi, double delta)
{
    // use RK4 for more accuracy
    a << 1., 0., DT * cos(phi), -DT * v * sin(phi),
         0., 1., DT * sin(phi), DT * v * cos(phi),
         0., 0., 1., 0.,
         0., 0., DT * tan(delta) / WB, 1.;

    b <<  0., 0.,
          0., 0.,
          DT, 0.,
          0., DT * v / (WB * pow(cos(delta), 2));

    //c << 0.0, 0.0 , 0.0, 0.0;
    c << DT * v * sin(phi) * phi, -DT * v * cos(phi) * phi, 0., -DT * v * delta / (WB * pow(cos(delta), 2));

}
*/
void setInequalityConstraints(Eigen::Matrix<double, 4, 1>& xMax,
                              Eigen::Matrix<double, 4, 1>& xMin,
                              Eigen::Matrix<double, 2, 1>& uMax,
                              Eigen::Matrix<double, 2, 1>& uMin)
{

    double u0_acc = 0;
    double u0_steer = 0;

    // input inequality constraints
    uMin << - MAX_ACCEL + u0_acc, - MAX_STEER + u0_steer;
    uMax << MAX_ACCEL - u0_acc, MAX_STEER - u0_steer;

    // state inequality constraints
    xMin <<  - OsqpEigen::INFTY, - OsqpEigen::INFTY, MIN_SPEED, -OsqpEigen::INFTY;
    xMax <<  OsqpEigen::INFTY, OsqpEigen::INFTY, MAX_SPEED, OsqpEigen::INFTY;
}

void setWeightMatrices(Eigen::DiagonalMatrix<double, 4>& Q, Eigen::DiagonalMatrix<double, 2>& R)
{
    Q.diagonal() << 1.0, 1.0, 0.5, 0.5;
    R.diagonal() << 0.01, 0.01;
}

void castMPCToQPHessian(const Eigen::DiagonalMatrix<double, 4>& Q,
                        const Eigen::DiagonalMatrix<double, 2>& R,
                        int mpcWindow,
                        Eigen::SparseMatrix<double>& hessianMatrix)
{

    hessianMatrix.resize(4 * (mpcWindow + 1) + 2 * mpcWindow,
                         4 * (mpcWindow + 1) + 2 * mpcWindow);

    // populate hessian matrix
    for (int i = 0; i < 4 * (mpcWindow + 1) + 2 * mpcWindow; i++)
    {
        if (i < 4 * (mpcWindow + 1))
        {
            int posQ = i % 4;
            float value = Q.diagonal()[posQ];
            if (value != 0)
                hessianMatrix.insert(i, i) = value;
        } else
        {
            int posR = i % 2;
            float value = R.diagonal()[posR];
            if (value != 0)
                hessianMatrix.insert(i, i) = value;
        }
    }
}


void castMPCToQPGradient(const Eigen::DiagonalMatrix<double, 4>& Q,
                         const Eigen::MatrixXd& xRef,  // Reference trajectory [12 x (mpcWindow + 1)]
                         int mpcWindow,
                         Eigen::VectorXd& gradient)
{
    // Ensure the reference trajectory has the correct dimensions
    if (xRef.rows() != 4 || xRef.cols() != (mpcWindow + 1)) {
        throw std::invalid_argument("❌ xRef must have dimensions [12 x (mpcWindow + 1)]");
    }

    // Resize the gradient vector
    gradient = Eigen::VectorXd::Zero(4 * (mpcWindow + 1) + 2 * mpcWindow, 1);

    // Iterate over the MPC horizon
    for (int t = 0; t < mpcWindow + 1; t++) {
        Eigen::Matrix<double, 4, 1> Qx_ref = Q * (-xRef.col(t)); // Q * (-xref for timestep t)

        // Assign to gradient
        for (int i = 0; i < 4; i++) {
            int idx = t * 4 + i;  // Compute the correct index in gradient
            gradient(idx, 0) = Qx_ref(i, 0);
        }
    }
}
void castMPCToQPConstraintMatrix(const Eigen::Matrix<double, 4, 4>& dynamicMatrix,
                                 const Eigen::Matrix<double, 4, 2>& controlMatrix,
                                 int mpcWindow,
                                 Eigen::SparseMatrix<double>& constraintMatrix)
{
    constraintMatrix.resize(4 * (mpcWindow + 1) + 4 * (mpcWindow + 1) + 2 * mpcWindow,
                            4 * (mpcWindow + 1) + 2 * mpcWindow);

    // populate linear constraint matrix
    for (int i = 0; i < 4 * (mpcWindow + 1); i++)
    {
        constraintMatrix.insert(i, i) = -1;
    }

    for (int i = 0; i < mpcWindow; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++)
            {
                float value = dynamicMatrix(j, k);
                if (value != 0)
                {
                    constraintMatrix.insert(4 * (i + 1) + j, 4 * i + k) = value;
                }
            }

    for (int i = 0; i < mpcWindow; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 2; k++)
            {
                float value = controlMatrix(j, k);
                if (value != 0)
                {
                    constraintMatrix.insert(4 * (i + 1) + j, 2 * i + k + 4 * (mpcWindow + 1))
                        = value;
                }
            }

    for (int i = 0; i < 4 * (mpcWindow + 1) + 2 * mpcWindow; i++)
    {
        constraintMatrix.insert(i + (mpcWindow + 1) * 4, i) = 1;
    }
}

void castMPCToQPConstraintVectors(const Eigen::Matrix<double, 4, 1>& xMax,
                                  const Eigen::Matrix<double, 4, 1>& xMin,
                                  const Eigen::Matrix<double, 2, 1>& uMax,
                                  const Eigen::Matrix<double, 2, 1>& uMin,
                                  const Eigen::Matrix<double, 4, 1>& x0,
                                  int mpcWindow,
                                  Eigen::VectorXd& lowerBound,
                                  Eigen::VectorXd& upperBound)
{
    // evaluate the lower and the upper inequality vectors
    Eigen::VectorXd lowerInequality
        = Eigen::MatrixXd::Zero(4 * (mpcWindow + 1) + 2 * mpcWindow, 1);

    Eigen::VectorXd upperInequality
        = Eigen::MatrixXd::Zero(4 * (mpcWindow + 1) + 2 * mpcWindow, 1);
    for (int i = 0; i < mpcWindow + 1; i++)
    {
        lowerInequality.block(4 * i, 0, 4, 1) = xMin;
        upperInequality.block(4 * i, 0, 4, 1) = xMax;
    }
    for (int i = 0; i < mpcWindow; i++)
    {
        lowerInequality.block(2 * i + 4 * (mpcWindow + 1), 0, 2, 1) = uMin;
        upperInequality.block(2 * i + 4 * (mpcWindow + 1), 0, 2, 1) = uMax;
    }

    // evaluate the lower and the upper equality vectors
    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(4 * (mpcWindow + 1), 1);
    Eigen::VectorXd upperEquality;
    lowerEquality.block(0, 0, 4, 1) = -x0;
    upperEquality = lowerEquality;
    lowerEquality = lowerEquality;

    // merge inequality and equality vectors
    lowerBound = Eigen::MatrixXd::Zero(2 * 4 * (mpcWindow + 1) + 2 * mpcWindow, 1);
    lowerBound << lowerEquality, lowerInequality;

    upperBound = Eigen::MatrixXd::Zero(2 * 4 * (mpcWindow + 1) + 2 * mpcWindow, 1);
    upperBound << upperEquality, upperInequality;
}

void updateConstraintVectors(const Eigen::Matrix<double, 4, 1>& x0,
                             Eigen::VectorXd& lowerBound,
                             Eigen::VectorXd& upperBound)
{
    lowerBound.block(0, 0, 4, 1) = -x0;
    upperBound.block(0, 0, 4, 1) = -x0;
}

double getErrorNorm(const Eigen::Matrix<double, 4, 1>& x, const Eigen::Matrix<double, 4, 1>& xRef)
{
    Eigen::Matrix<double, 4, 1> error = x - xRef;
    return error.norm();
}
/*
int main()
{
    // set the preview window
    int mpcWindow = 10;

    // allocate the dynamics matrices
    Eigen::Matrix<double, 4, 4> a;
    Eigen::Matrix<double, 4, 2> b;
    Eigen::Matrix<double, 4, 1> c;

    // allocate the constraints vector
    Eigen::Matrix<double, 4, 1> xMax;
    Eigen::Matrix<double, 4, 1> xMin;
    Eigen::Matrix<double, 2, 1> uMax;
    Eigen::Matrix<double, 2, 1> uMin;

    // allocate the weight matrices
    Eigen::DiagonalMatrix<double, 4> Q;
    Eigen::DiagonalMatrix<double, 2> R;

    // allocate the initial and the reference state space
    Eigen::Matrix<double, 4, 1> x0;
    Eigen::Matrix<double, 2, 1> ctr;

    // allocate QP problem matrices and vectors
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    // set the initial and the desired states
    double radius = 20.;         // Radius of the circle

    x0 << radius, 0, 1, M_PI_2;

    Eigen::MatrixXd xRef(4, mpcWindow + 1);
    for (int t = 0; t <= mpcWindow; t++) {
        double time_ref = t * DT;
        xRef(0, t) = x0(0);
        xRef(1, t) = x0(1);
        xRef(2, t) = x0(2);
        xRef(3, t) = x0(3);
    }

    // set MPC problem quantities
    double v = x0(2);
    double phi = x0(3);
    //double delta = 0;
    double delta = atan(WB / radius);  // Steering for circular motion

    ctr << 0, delta;
    //setDynamicsMatrices(a, b, c, x0, ctr, DT);
    setDynamicsMatrices(a, b, c, v, phi, delta);
    setInequalityConstraints(xMax, xMin, uMax, uMin);
    setWeightMatrices(Q, R);

    // cast the MPC problem as QP problem
    castMPCToQPHessian(Q, R, mpcWindow, hessian);
    castMPCToQPGradient(Q, xRef, mpcWindow, gradient);
    castMPCToQPConstraintMatrix(a, b, mpcWindow, linearMatrix);
    castMPCToQPConstraintVectors(xMax, xMin, uMax, uMin, x0, mpcWindow, lowerBound, upperBound);

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    // solver.settings()->setVerbosity(false);
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

    // controller input and QPSolution vector
    //Eigen::Vector2d ctr;
    Eigen::VectorXd QPSolution;

    // number of iteration steps
    int numberOfSteps = 1000;
    std::vector<double> x_data, y_data, delta_data; // Store trajectory data
    std::vector<double> x_ref_data, y_ref_data;
    auto start_time = high_resolution_clock::now();
    for (int i = 0; i < numberOfSteps; i++)
    {
        auto loop_start = high_resolution_clock::now();
        setDynamicsMatrices(a, b, c, v, phi, delta);
        //setDynamicsMatrices(a, b, c, x0, ctr, DT);

        // trajectory
        double velocity = 1.0;       // Reference velocity
        double xc = 0.0, yc = 0.0;

        for (int t = 0; t <= mpcWindow; t++) {
            double theta =  i * DT * velocity / radius;  // Angle along the circle
            xRef(0, t) = xc + radius * cos(theta); // x_ref
            xRef(1, t) = yc + radius * sin(theta); // y_ref
            xRef(2, t) = velocity;                 // Constant velocity
            xRef(3, t) = theta + M_PI_2;           // Yaw aligned with the circle tangent
        }
        // for plot

        double theta_plot =  i * DT * velocity / radius;
        x_ref_data.push_back(xc + radius * cos(theta_plot));
        y_ref_data.push_back(yc + radius * sin(theta_plot));
        //


        castMPCToQPGradient(Q, xRef, mpcWindow, gradient);
        solver.updateGradient(gradient);

        castMPCToQPConstraintMatrix(a, b, mpcWindow, linearMatrix);
        solver.updateLinearConstraintsMatrix(linearMatrix);

        castMPCToQPConstraintVectors(xMax, xMin, uMax, uMin, x0, mpcWindow, lowerBound, upperBound);
        updateConstraintVectors(x0, lowerBound, upperBound);
        solver.updateBounds(lowerBound, upperBound);


        // solve the QP problem
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
            return 1;

        // get the controller input
        QPSolution = solver.getSolution();
        ctr = QPSolution.block(4 * (mpcWindow + 1), 0, 2, 1);

        // save data into file
        auto x0Data = x0.data();

        // propagate the model

        double v_new = x0(2) + ctr(0) * DT;

        x0(3) += (v_new / WB) * tan(ctr(1)) * DT;  // Update yaw first
        x0(0) += v_new * cos(x0(3)) * DT;  // Use new yaw
        x0(1) += v_new * sin(x0(3)) * DT;
        x0(2) = v_new;  // Finally update velocity

        //x0 = a * x0 + b * ctr;

        v = x0(2);
        phi = x0(3);
        delta = ctr(1);
        // Store x and y positions
        std::cout << "x--->" << x0(0) << std::endl;
        std::cout << "y--->" << x0(1) << std::endl;
        std::cout << "v--->" << x0(2) << std::endl;
        std::cout << "phi--->" << x0(3) << std::endl;
        std::cout << "i--->" << i << std::endl;
        x_data.push_back(x0(0)); // x-coordinate
        y_data.push_back(x0(1)); // y-coordinate
        delta_data.push_back(delta); // y-coordinate
        auto loop_end = high_resolution_clock::now();
        duration<double> loop_duration = loop_end - loop_start;
        double loop_freq = 1.0 / loop_duration.count();
        std::cout << "Loop Frequency: " << loop_freq << " Hz" << std::endl;
    }

    plt::figure();

    // First subplot: Vehicle trajectory vs. reference trajectory
    plt::subplot(2, 1, 1);  // (rows, cols, index)
    plt::named_plot("MPC Trajectory", x_data, y_data, "r-");
    plt::named_plot("Reference Circle", x_ref_data, y_ref_data, "b--");

    // Labels and formatting
    plt::xlabel("X Position");
    plt::ylabel("Y Position");
    plt::title("MPC Trajectory vs. Reference");
    plt::legend();
    plt::grid(true);

    // Second subplot: Steering angle (delta) over time
    plt::subplot(2, 1, 2);  // Second subplot for delta
    plt::plot(delta_data, "g-"); // Green line for delta

    // Labels and formatting
    plt::xlabel("Time Steps");
    plt::ylabel("Steering Angle (delta) [rad]");
    plt::title("Steering Control Over Time");
    plt::grid(true);

    // Show all subplots
    plt::show();        return 0;
}
*/

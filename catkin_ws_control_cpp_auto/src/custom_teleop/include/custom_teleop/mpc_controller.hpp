#ifndef MPC_OSQP_HPP
#define MPC_OSQP_HPP

#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include "hunter_params.hpp"

using namespace westonrobot;

static constexpr int NX = 4; // states
static constexpr int NU = 2; // input

static constexpr int N_IND_SEARCH = 10;
static constexpr double DT = 0.01; // s reduce when speeds is lower than 0.5
static constexpr double  WB = HunterV2Params::wheelbase;
static constexpr double  MAX_STEER = HunterV2Params::max_steer_angle;
static constexpr double  MAX_DSTEER = 0.52; // max ddelta rad/s -  30 deg / s
static constexpr double  MAX_SPEED = HunterV2Params::max_linear_speed;
static constexpr double  MIN_SPEED = -HunterV2Params::max_linear_speed;
static constexpr double  MAX_ACCEL = 1; // m/ss max accelration
static constexpr double GOAL_DIS = 1.5;  // goal distance m
static constexpr double STOP_SPEED = 0.125; // stop speed m/s
static constexpr double TARGET_SPEED = 0.5; // m/s

// Function Declarations

// Set system dynamics matrices A and B
//direct linearized model
//void setDynamicsMatrices(Eigen::Matrix<double, 4, 4>& a, Eigen::Matrix<double, 4, 2>& b, Eigen::Matrix<double, 4, 1>& c, double v, double phi, double delta);
void setDynamicsMatrices(Eigen::Matrix<double, 4, 4>& A,
                         Eigen::Matrix<double, 4, 2>& B,
                         Eigen::Matrix<double, 4, 1>& C,
                         const Eigen::Matrix<double, 4, 1>& x,
                         const Eigen::Matrix<double, 2, 1>& u,
                         double sample_period);

// Set inequality constraint limits for states and inputs
void setInequalityConstraints(Eigen::Matrix<double, 4, 1>& xMax,
                              Eigen::Matrix<double, 4, 1>& xMin,
                              Eigen::Matrix<double, 2, 1>& uMax,
                              Eigen::Matrix<double, 2, 1>& uMin);

// Set weight matrices for MPC optimization (Q for states, R for inputs)
void setWeightMatrices(Eigen::DiagonalMatrix<double, 4>& Q, Eigen::DiagonalMatrix<double, 2>& R);

// Convert MPC cost function to QP Hessian matrix
void castMPCToQPHessian(const Eigen::DiagonalMatrix<double, 4>& Q,
                        const Eigen::DiagonalMatrix<double, 2>& R,
                        int mpcWindow,
                        Eigen::SparseMatrix<double>& hessianMatrix);

// Convert MPC to QP gradient vector
void castMPCToQPGradient(const Eigen::DiagonalMatrix<double, 4>& Q,
                         const Eigen::MatrixXd& xRef,
                         int mpcWindow,
                         Eigen::VectorXd& gradient);

// Convert MPC constraints into QP constraint matrix
void castMPCToQPConstraintMatrix(const Eigen::Matrix<double, 4, 4>& dynamicMatrix,
                                 const Eigen::Matrix<double, 4, 2>& controlMatrix,
                                 int mpcWindow,
                                 Eigen::SparseMatrix<double>& constraintMatrix);

// Convert MPC constraints into QP constraint vectors (upper and lower bounds)
void castMPCToQPConstraintVectors(const Eigen::Matrix<double, 4, 1>& xMax,
                                  const Eigen::Matrix<double, 4, 1>& xMin,
                                  const Eigen::Matrix<double, 2, 1>& uMax,
                                  const Eigen::Matrix<double, 2, 1>& uMin,
                                  const Eigen::Matrix<double, 4, 1>& x0,
                                  int mpcWindow,
                                  Eigen::VectorXd& lowerBound,
                                  Eigen::VectorXd& upperBound);

// Update constraint vectors dynamically during MPC execution
void updateConstraintVectors(const Eigen::Matrix<double, 4, 1>& x0,
                             Eigen::VectorXd& lowerBound,
                             Eigen::VectorXd& upperBound);

// Compute error norm between current state and reference state
double getErrorNorm(const Eigen::Matrix<double, 4, 1>& x, const Eigen::Matrix<double, 4, 1>& xRef);

#endif // MPC_OSQP_HPP

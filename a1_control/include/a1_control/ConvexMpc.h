
#ifndef MPC_H
#define MPC_H

#define EIGEN_STACK_ALLOCATION_LIMIT 0

#include <vector>
#include <chrono>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>
#include "a1_control/RobotState.h"
#include "a1_control/Params.h"
#include "a1_control/Utils.h"
#include <qpOASES.hpp>

class ConvexMpc {
public:
    ConvexMpc(Eigen::VectorXd &q_weights_, Eigen::VectorXd &r_weights_);

    void reset();

    void calculate_A_c(Eigen::Vector3d root_euler);

    void calculate_B_c(double robot_mass, const Eigen::Matrix3d &trunk_inertia, Eigen::Matrix3d root_rot_mat,
                           Eigen::Matrix<double, 3, NUM_LEG> foot_pos);

    void state_space_discretization(double dt);

    void calculate_mpc_matrix(RobotStates & state);

//private:
    // parameters initialized with class Mpc
    double mu;
    double f_min;
    double f_max;
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, 1> q_weights_mpc;
    Eigen::Matrix<double, NUM_DOF * PLAN_HORIZON, 1> r_weights_mpc;

    Eigen::DiagonalMatrix<double, MPC_STATE_DIM * PLAN_HORIZON> Q;
    Eigen::DiagonalMatrix<double, NUM_DOF * PLAN_HORIZON> R;

    Eigen::MatrixXd A_mat_c = Eigen::MatrixXd::Zero(MPC_STATE_DIM, MPC_STATE_DIM);
    Eigen::MatrixXd B_mat_c = Eigen::MatrixXd::Zero(MPC_STATE_DIM, NUM_DOF);

    Eigen::MatrixXd A_mat_d = Eigen::MatrixXd::Zero(MPC_STATE_DIM, MPC_STATE_DIM);
    Eigen::MatrixXd B_mat_d = Eigen::MatrixXd::Zero(MPC_STATE_DIM, NUM_DOF);

    Eigen::MatrixXd A_qp = Eigen::MatrixXd::Zero(MPC_STATE_DIM * PLAN_HORIZON, MPC_STATE_DIM);
    Eigen::MatrixXd B_qp = Eigen::MatrixXd::Zero(MPC_STATE_DIM * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON);

    Eigen::MatrixXd B_mat_d_list = Eigen::MatrixXd::Zero(MPC_STATE_DIM * PLAN_HORIZON, NUM_DOF);


    Eigen::MatrixXd hessian; // P
    Eigen::MatrixXd C; // constraint matrix
    Eigen::SparseMatrix<double> linear_constraints; // Ac

    Eigen::SparseMatrix<double> Q_sparse;
    Eigen::SparseMatrix<double> R_sparse;

    Eigen::Matrix<double, NUM_DOF * PLAN_HORIZON, 1> gradient; // q
    Eigen::Matrix<double, MPC_CONSTRAINT_DIM * PLAN_HORIZON, 1> lb; //lower bound constraints
    Eigen::Matrix<double, MPC_CONSTRAINT_DIM * PLAN_HORIZON, 1> ub; //upper bound constraints


};

#endif 

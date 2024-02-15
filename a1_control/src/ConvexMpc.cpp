#include <a1_control/ConvexMpc.h>


ConvexMpc::ConvexMpc(Eigen::VectorXd &q_weights_, Eigen::VectorXd &r_weights_)
{
  mu = 0.5;
  f_min = 0.0;
  f_max = 0.0;

  Q_sparse = Eigen::SparseMatrix<double>(MPC_STATE_DIM * PLAN_HORIZON,MPC_STATE_DIM * PLAN_HORIZON);
  R_sparse = Eigen::SparseMatrix<double>(NUM_DOF * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON);

  q_weights_mpc.resize(MPC_STATE_DIM * PLAN_HORIZON);
  for (int i = 0; i < PLAN_HORIZON; ++i) {
      q_weights_mpc.segment(i * MPC_STATE_DIM, MPC_STATE_DIM) = q_weights_;
  }
  Q.diagonal() = q_weights_mpc;
  for (int i = 0; i < MPC_STATE_DIM*PLAN_HORIZON; ++i) {
      Q_sparse.insert(i,i) = 2*q_weights_mpc(i);
  }

  r_weights_mpc.resize(NUM_DOF * PLAN_HORIZON);
  for (int i = 0; i < PLAN_HORIZON; ++i) {
      r_weights_mpc.segment(i * NUM_DOF, NUM_DOF) = r_weights_;
  }
  R.diagonal() = 2*r_weights_mpc;
  for (int i = 0; i < NUM_DOF*PLAN_HORIZON; ++i) {
      R_sparse.insert(i,i) = r_weights_mpc(i);
  }

  linear_constraints.resize(MPC_CONSTRAINT_DIM * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON);    
  for (int i = 0; i < NUM_LEG * PLAN_HORIZON; ++i) {
      linear_constraints.insert(0 + 5 * i, 0 + 3 * i) = 1;
      linear_constraints.insert(1 + 5 * i, 0 + 3 * i) = 1;
      linear_constraints.insert(2 + 5 * i, 1 + 3 * i) = 1;
      linear_constraints.insert(3 + 5 * i, 1 + 3 * i) = 1;
      linear_constraints.insert(4 + 5 * i, 2 + 3 * i) = 1;

      linear_constraints.insert(0 + 5 * i, 2 + 3 * i) = mu;
      linear_constraints.insert(1 + 5 * i, 2 + 3 * i) = -mu;
      linear_constraints.insert(2 + 5 * i, 2 + 3 * i) = mu;
      linear_constraints.insert(3 + 5 * i, 2 + 3 * i) = -mu;
  }
  
}

void ConvexMpc::reset() 
{
  A_mat_c.setZero();
  B_mat_c.setZero();

  A_mat_d.setZero();
  B_mat_d.setZero();
  B_mat_d_list.setZero();
  A_qp.setZero();
  B_qp.setZero();
  gradient.setZero();
  lb.setZero();
  ub.setZero();

}

void ConvexMpc::calculate_A_c(Eigen::Vector3d root_euler) 
{
  double cos_yaw = cos(root_euler[2]);
  double sin_yaw = sin(root_euler[2]);

  Eigen::Matrix3d ang_vel_to_rpy_rate;

  ang_vel_to_rpy_rate << cos_yaw, sin_yaw, 0,
          -sin_yaw, cos_yaw, 0,
          0, 0, 1;

  A_mat_c.block<3, 3>(0, 6) = ang_vel_to_rpy_rate;
  A_mat_c.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
  A_mat_c(11, NUM_DOF) = 1;
}

void ConvexMpc::calculate_B_c(double robot_mass, const Eigen::Matrix3d &trunk_inertia, Eigen::Matrix3d root_rot_mat,
                                  Eigen::Matrix<double, 3, NUM_LEG> foot_pos) 
{
  Eigen::Matrix3d trunk_inertia_world;
  trunk_inertia_world = root_rot_mat * trunk_inertia * root_rot_mat.transpose();
  for (int i = 0; i < NUM_LEG; ++i) {
      B_mat_c.block<3, 3>(6, 3 * i) =
              trunk_inertia_world.inverse() * Utils::skew(foot_pos.block<3, 1>(0, i));
      B_mat_c.block<3, 3>(9, 3 * i) =
              (1 / robot_mass) * Eigen::Matrix3d::Identity();
  }
}

void ConvexMpc::state_space_discretization(double dt) // forward euler integration
{
  A_mat_d = Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>::Identity() + A_mat_c*dt;
  B_mat_d = B_mat_c*dt;
}

void ConvexMpc::calculate_mpc_matrix(RobotStates & state) //qp로 풀기
{
  f_min = 0;
  f_max = 180;

  Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> tmp_mtx;
  for (int i = 0; i < PLAN_HORIZON; ++i) {
      if (i == 0) {
          A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * i, 0) = A_mat_d;
      }
      else {
          A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * i, 0) = 
          A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * (i-1), 0)*A_mat_d;
      }
      for (int j = 0; j < i + 1; ++j) {
          if (i-j == 0) {
              B_qp.block<MPC_STATE_DIM, NUM_DOF>(MPC_STATE_DIM * i, NUM_DOF * j) =
                  B_mat_d_list.block<MPC_STATE_DIM, NUM_DOF>(j * MPC_STATE_DIM, 0);
          } else {
              B_qp.block<MPC_STATE_DIM, NUM_DOF>(MPC_STATE_DIM * i, NUM_DOF * j) =
                      A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * (i-j-1), 0) 
                      * B_mat_d_list.block<MPC_STATE_DIM, NUM_DOF>(j * MPC_STATE_DIM, 0);
          }
      }
  }

  Eigen::Matrix<double, NUM_DOF * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON> dense_hessian;

  dense_hessian = (B_qp.transpose() * Q * B_qp);
  dense_hessian += R;
  hessian = dense_hessian.sparseView();


  Eigen::Matrix<double, 13*PLAN_HORIZON, 1> tmp_vec = A_qp* state.mpc_states;
  tmp_vec -= state.mpc_states_d;
  gradient = B_qp.transpose() * Q * tmp_vec;


  Eigen::VectorXd lb_one_horizon(MPC_CONSTRAINT_DIM);
  Eigen::VectorXd ub_one_horizon(MPC_CONSTRAINT_DIM);
  for (int i = 0; i < NUM_LEG; ++i) {
      lb_one_horizon.segment<5>(i * 5) << 0,
              -qpOASES::INFTY,
              0,
              -qpOASES::INFTY,
              f_min * state.contacts[i];
      ub_one_horizon.segment<5>(i * 5) << qpOASES::INFTY,
              0,
              qpOASES::INFTY,
              0,
              f_max * state.contacts[i];
  }
  for (int i = 0; i < PLAN_HORIZON; ++i) {
      lb.segment<MPC_CONSTRAINT_DIM>(i * MPC_CONSTRAINT_DIM) = lb_one_horizon;
      ub.segment<MPC_CONSTRAINT_DIM>(i * MPC_CONSTRAINT_DIM) = ub_one_horizon;
  }
}
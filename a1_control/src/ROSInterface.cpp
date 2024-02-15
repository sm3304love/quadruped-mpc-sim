//
// Created by zixin on 11/1/21.
//

#include "a1_control/ROSInterface.h"

// constructor
QuadrupedROS::QuadrupedROS(ros::NodeHandle &_nh) {
    nh = _nh;

    // ROS publisher
    pub_joint_cmd[0] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_hip_controller/command", 1);
    pub_joint_cmd[1] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_thigh_controller/command", 1);
    pub_joint_cmd[2] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_calf_controller/command", 1);

    pub_joint_cmd[3] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_hip_controller/command", 1);
    pub_joint_cmd[4] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_thigh_controller/command", 1);
    pub_joint_cmd[5] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_calf_controller/command", 1);

    pub_joint_cmd[6] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_hip_controller/command", 1);
    pub_joint_cmd[7] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_thigh_controller/command", 1);
    pub_joint_cmd[8] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_calf_controller/command", 1);

    pub_joint_cmd[9] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_hip_controller/command", 1);
    pub_joint_cmd[10] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_thigh_controller/command", 1);
    pub_joint_cmd[11] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_calf_controller/command", 1);

    // debug estimation
    pub_estimated_pose = nh.advertise<nav_msgs::Odometry>("/gazebo_a1/estimation_body_pose", 100);

    pub_euler_d = nh.advertise<geometry_msgs::PointStamped>("a1_debug/euler_d", 100);

    // ROS register callback, call backs directly modify variables in A1CtrlStates
    sub_imu_msg = nh.subscribe("/trunk_imu", 100, &QuadrupedROS::imu_callback, this);

    sub_joint_msg[0] = nh.subscribe("/a1_gazebo/FL_hip_controller/state", 2, &QuadrupedROS::FL_hip_state_callback, this);
    sub_joint_msg[1] = nh.subscribe("/a1_gazebo/FL_thigh_controller/state", 2, &QuadrupedROS::FL_thigh_state_callback, this);
    sub_joint_msg[2] = nh.subscribe("/a1_gazebo/FL_calf_controller/state", 2, &QuadrupedROS::FL_calf_state_callback, this);

    sub_joint_msg[3] = nh.subscribe("/a1_gazebo/FR_hip_controller/state", 2, &QuadrupedROS::FR_hip_state_callback, this);
    sub_joint_msg[4] = nh.subscribe("/a1_gazebo/FR_thigh_controller/state", 2, &QuadrupedROS::FR_thigh_state_callback, this);
    sub_joint_msg[5] = nh.subscribe("/a1_gazebo/FR_calf_controller/state", 2, &QuadrupedROS::FR_calf_state_callback, this);

    sub_joint_msg[6] = nh.subscribe("/a1_gazebo/RL_hip_controller/state", 2, &QuadrupedROS::RL_hip_state_callback, this);
    sub_joint_msg[7] = nh.subscribe("/a1_gazebo/RL_thigh_controller/state", 2, &QuadrupedROS::RL_thigh_state_callback, this);
    sub_joint_msg[8] = nh.subscribe("/a1_gazebo/RL_calf_controller/state", 2, &QuadrupedROS::RL_calf_state_callback, this);

    sub_joint_msg[9] = nh.subscribe("/a1_gazebo/RR_hip_controller/state", 2, &QuadrupedROS::RR_hip_state_callback, this);
    sub_joint_msg[10] = nh.subscribe("/a1_gazebo/RR_thigh_controller/state", 2, &QuadrupedROS::RR_thigh_state_callback, this);
    sub_joint_msg[11] = nh.subscribe("/a1_gazebo/RR_calf_controller/state", 2, &QuadrupedROS::RR_calf_state_callback, this);

    sub_foot_contact_msg[0] = nh.subscribe("/visual/FL_foot_contact/the_force", 2, &QuadrupedROS::FL_foot_contact_callback, this);
    sub_foot_contact_msg[1] = nh.subscribe("/visual/FR_foot_contact/the_force", 2, &QuadrupedROS::FR_foot_contact_callback, this);
    sub_foot_contact_msg[2] = nh.subscribe("/visual/RL_foot_contact/the_force", 2, &QuadrupedROS::RL_foot_contact_callback, this);
    sub_foot_contact_msg[3] = nh.subscribe("/visual/RR_foot_contact/the_force", 2, &QuadrupedROS::RR_foot_contact_callback, this);

    // sub_joy_msg = nh.subscribe("/joy", 1000, &QuadrupedROS::joy_callback, this);
    sub_robot_cmd_msg = nh.subscribe("/a1_robot_cmd", 1000, &QuadrupedROS::Robot_command_callback, this);

    _root_control = RobotController(nh);
    robot_state.reset();
    robot_state.resetFromROSParam(nh);

    // init leg kinematics
    // set leg kinematics related parameters
    // body_to_a1_body
    p_br = Eigen::Vector3d(-0.2293, 0.0, -0.067);
    R_br = Eigen::Matrix3d::Identity();


    acc_x = MovingWindowFilter(5);
    acc_y = MovingWindowFilter(5);
    acc_z = MovingWindowFilter(5);
    gyro_x = MovingWindowFilter(5);
    gyro_y = MovingWindowFilter(5);
    gyro_z = MovingWindowFilter(5);
    quat_w = MovingWindowFilter(5);
    quat_x = MovingWindowFilter(5);
    quat_y = MovingWindowFilter(5);
    quat_z = MovingWindowFilter(5);

    robot_state.base_link = "trunk";
    robot_state.fl_foot_link = "FL_foot";
    robot_state.fr_foot_link = "FR_foot";
    robot_state.rl_foot_link = "RL_foot";
    robot_state.rr_foot_link = "RR_foot";

    robot_state.root_ang_vel_d[0] = 0.0;
    robot_state.root_ang_vel_d[1] = 0.0;
    robot_state.root_ang_vel_d[2] = 0.0;
    robot_state.root_euler_d[0] = 0.0;
    robot_state.root_euler_d[1] = 0.0;
    robot_state.root_euler_d[2] = 0.0;
    robot_state.root_pos_d[2] = 0.0;

    if (!kdl_parser::treeFromParam("robot_description", robot_state.tree)) 
    {
        ROS_ERROR("Failed to construct kdl tree");
    }

    // KDL::Chain chain;
    if (!robot_state.tree.getChain(robot_state.base_link, robot_state.fl_foot_link, robot_state.fl_chain)) 
    {
        ROS_ERROR("Failed to get chain from tree");
    }

    if (!robot_state.tree.getChain(robot_state.base_link, robot_state.fr_foot_link, robot_state.fr_chain)) 
    {
        ROS_ERROR("Failed to get chain from tree");
    }

    if (!robot_state.tree.getChain(robot_state.base_link, robot_state.rl_foot_link, robot_state.rl_chain)) 
    {
        ROS_ERROR("Failed to get chain from tree");
    }

    if (!robot_state.tree.getChain(robot_state.base_link, robot_state.rr_foot_link, robot_state.rr_chain)) 
    {
        ROS_ERROR("Failed to get chain from tree");
    }

}

bool QuadrupedROS::update_foot_forces_grf(double dt) {

    robot_state.foot_forces_grf = _root_control.compute_grf(robot_state, dt);
    return true;
}

bool QuadrupedROS::main_update(double t, double dt) {

    robot_state.root_lin_vel_d[0] = cmd_velx;
    robot_state.root_lin_vel_d[1] = cmd_vely;
    robot_state.root_lin_vel_d[2] = cmd_velz;

    cmd_body_height += cmd_velz * dt;

    if (cmd_body_height >= BODY_HEIGHT_MAX) {
        cmd_body_height = BODY_HEIGHT_MAX;
    }
    if (cmd_body_height <= BODY_HEIGHT_MIN) {
        cmd_body_height = BODY_HEIGHT_MIN;
    }

    // root_ang_vel_d is in robot frame
    robot_state.root_ang_vel_d[0] = cmd_roll_vel;
    robot_state.root_ang_vel_d[1] = cmd_pitch_vel;
    robot_state.root_ang_vel_d[2] = cmd_yaw_vel;
    robot_state.root_euler_d[0] += cmd_roll_vel * dt;
    robot_state.root_euler_d[1] += cmd_pitch_vel * dt;
    robot_state.root_euler_d[2] += cmd_yaw_vel * dt;
    robot_state.root_pos_d[2] = cmd_body_height;
 
    prev_robot_ctrl_state = robot_ctrl_state;


    // determine movement mode
    if (robot_ctrl_state == 1) {
        // walking mode, in this mode the robot should execute gait
        robot_state.movement_mode = 1;
    } else if (robot_ctrl_state == 0 && prev_robot_ctrl_state == 1) {
        // leave walking mode
        // lock current position -- for stay current position
        robot_state.movement_mode = 0;
        robot_state.root_pos_d.segment<2>(0) = robot_state.root_pos.segment<2>(0);
        robot_state.kp_linear(0) = robot_state.kp_linear_lock_x;
        robot_state.kp_linear(1) = robot_state.kp_linear_lock_y;
    } else {
        robot_state.movement_mode = 0;
    }

    // in walking mode, do position locking if no root_lin_vel_d, otherwise do not lock position
    if (robot_state.movement_mode == 1) {
        if (robot_state.root_lin_vel_d.segment<2>(0).norm() > 0.05) {
            // has nonzero velocity, keep refreshing position target, but just xy --> 속도 제어 기반이라서 위치값이 계속 바뀌는데, 그곳을 desired로 만들기
            robot_state.root_pos_d.segment<2>(0) = robot_state.root_pos.segment<2>(0);
            robot_state.kp_linear.segment<2>(0).setZero();
        } else {
            robot_state.kp_linear(0) = robot_state.kp_linear_lock_x;
            robot_state.kp_linear(1) = robot_state.kp_linear_lock_y;
        }
    }

    _root_control.update_plan(robot_state, dt);
    _root_control.generate_swing_legs_ctrl(robot_state, dt);

    // state estimation
    if (!robot_pos_estimate.is_inited()) {
        robot_pos_estimate.init_state(robot_state);
    } else {
        robot_pos_estimate.update_estimation(robot_state, dt);
    }

    


    nav_msgs::Odometry estimate_odom;
    estimate_odom.pose.pose.position.x = robot_state.estimated_root_pos(0);
    estimate_odom.pose.pose.position.y = robot_state.estimated_root_pos(1);
    estimate_odom.pose.pose.position.z = robot_state.estimated_root_pos(2);

    // make sure root_lin_vel is in world frame
    estimate_odom.twist.twist.linear.x = robot_state.estimated_root_vel(0);
    estimate_odom.twist.twist.linear.y = robot_state.estimated_root_vel(1);
    estimate_odom.twist.twist.linear.z = robot_state.estimated_root_vel(2);

    pub_estimated_pose.publish(estimate_odom);

    return true;
}

bool QuadrupedROS::send_cmd() {
    _root_control.compute_joint_torques(robot_state);

    // send control cmd to robot via ros topic
    unitree_legged_msgs::LowCmd low_cmd;

    for (int i = 0; i < 12; i++) {
        low_cmd.motorCmd[i].mode = 0x0A;
        low_cmd.motorCmd[i].q = 0;
        low_cmd.motorCmd[i].dq = 0;
        low_cmd.motorCmd[i].Kp = 0;
        low_cmd.motorCmd[i].Kd = 0;
        low_cmd.motorCmd[i].tau = robot_state.joint_torques(i, 0);
        pub_joint_cmd[i].publish(low_cmd.motorCmd[i]);
    }

    return true;
}

void QuadrupedROS::imu_callback(const sensor_msgs::Imu::ConstPtr &imu) 
{
    robot_state.root_quat = Eigen::Quaterniond(quat_w.CalculateAverage(imu->orientation.w),
                                                  quat_x.CalculateAverage(imu->orientation.x),
                                                  quat_y.CalculateAverage(imu->orientation.y),
                                                  quat_z.CalculateAverage(imu->orientation.z));

    robot_state.root_rot_mat = robot_state.root_quat.toRotationMatrix();

    robot_state.imu_acc = Eigen::Vector3d(
            acc_x.CalculateAverage(imu->linear_acceleration.x),
            acc_y.CalculateAverage(imu->linear_acceleration.y),
            acc_z.CalculateAverage(imu->linear_acceleration.z)
    );
    robot_state.imu_ang_vel = Eigen::Vector3d(
            gyro_x.CalculateAverage(imu->angular_velocity.x),
            gyro_y.CalculateAverage(imu->angular_velocity.y),
            gyro_z.CalculateAverage(imu->angular_velocity.z)
    );
    robot_state.root_ang_vel = robot_state.root_rot_mat * robot_state.imu_ang_vel;

    robot_state.root_euler = Utils::quat_to_euler(robot_state.root_quat);
    double yaw_angle = robot_state.root_euler[2];

    robot_state.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());

    KDL::Vector gravity(0, 0, -9.81);

    KDL::ChainDynParam fl_dyn_param(robot_state.fl_chain, gravity);
    KDL::ChainDynParam fr_dyn_param(robot_state.fr_chain, gravity);
    KDL::ChainDynParam rl_dyn_param(robot_state.rl_chain, gravity);
    KDL::ChainDynParam rr_dyn_param(robot_state.rr_chain, gravity);

    KDL::JntArray q_fl(NUM_DOF_PER_LEG);
    KDL::JntArray q_fr(NUM_DOF_PER_LEG);
    KDL::JntArray q_rl(NUM_DOF_PER_LEG);
    KDL::JntArray q_rr(NUM_DOF_PER_LEG);

    KDL::JntArray dq_fl(NUM_DOF_PER_LEG);
    KDL::JntArray dq_fr(NUM_DOF_PER_LEG);
    KDL::JntArray dq_rl(NUM_DOF_PER_LEG);
    KDL::JntArray dq_rr(NUM_DOF_PER_LEG);


    q_fl(0) = robot_state.joint_pos[0];
    q_fl(1) = robot_state.joint_pos[1];
    q_fl(2) = robot_state.joint_pos[2];

    q_fr(0) = robot_state.joint_pos[3];
    q_fr(1) = robot_state.joint_pos[4];
    q_fr(2) = robot_state.joint_pos[5];

    q_rl(0) = robot_state.joint_pos[6];
    q_rl(1) = robot_state.joint_pos[7];
    q_rl(2) = robot_state.joint_pos[8];

    q_rr(0) = robot_state.joint_pos[9];
    q_rr(1) = robot_state.joint_pos[10];
    q_rr(2) = robot_state.joint_pos[11];

    dq_fl(0) = robot_state.joint_vel[0];
    dq_fl(1) = robot_state.joint_vel[1];
    dq_fl(2) = robot_state.joint_vel[2];

    dq_fr(0) = robot_state.joint_vel[3];
    dq_fr(1) = robot_state.joint_vel[4];
    dq_fr(2) = robot_state.joint_vel[5];

    dq_rl(0) = robot_state.joint_vel[6];
    dq_rl(1) = robot_state.joint_vel[7];
    dq_rl(2) = robot_state.joint_vel[8];

    dq_rr(0) = robot_state.joint_vel[9];
    dq_rr(1) = robot_state.joint_vel[10];
    dq_rr(2) = robot_state.joint_vel[11];

    KDL::ChainFkSolverPos_recursive fk_solver_fl(robot_state.fl_chain);
    KDL::ChainFkSolverPos_recursive fk_solver_fr(robot_state.fr_chain);
    KDL::ChainFkSolverPos_recursive fk_solver_rl(robot_state.rl_chain);
    KDL::ChainFkSolverPos_recursive fk_solver_rr(robot_state.rr_chain);

    KDL::Frame fl_foot_position;
    fk_solver_fl.JntToCart(q_fl, fl_foot_position);
    KDL::Frame fr_foot_position;
    fk_solver_fr.JntToCart(q_fr, fr_foot_position);
    KDL::Frame rl_foot_position;
    fk_solver_rl.JntToCart(q_rl, rl_foot_position);
    KDL::Frame rr_foot_position;
    fk_solver_rr.JntToCart(q_rr, rr_foot_position);

    Eigen::Vector3d fl_foot_pos_vec(fl_foot_position.p.x(), fl_foot_position.p.y(), fl_foot_position.p.z());
    Eigen::Vector3d fr_foot_pos_vec(fr_foot_position.p.x(), fr_foot_position.p.y(), fr_foot_position.p.z());
    Eigen::Vector3d rl_foot_pos_vec(rl_foot_position.p.x(), rl_foot_position.p.y(), rl_foot_position.p.z());
    Eigen::Vector3d rr_foot_pos_vec(rr_foot_position.p.x(), rr_foot_position.p.y(), rr_foot_position.p.z());

    KDL::ChainJntToJacSolver jac_solver_fl(robot_state.fl_chain);
    KDL::ChainJntToJacSolver jac_solver_fr(robot_state.fr_chain);
    KDL::ChainJntToJacSolver jac_solver_rl(robot_state.rl_chain);
    KDL::ChainJntToJacSolver jac_solver_rr(robot_state.rr_chain);

    KDL::Jacobian fl_foot_jacobian_kdl(3);
    jac_solver_fl.JntToJac(q_fl, fl_foot_jacobian_kdl);
    KDL::Jacobian fr_foot_jacobian_kdl(3);
    jac_solver_fr.JntToJac(q_fr, fr_foot_jacobian_kdl);
    KDL::Jacobian rl_foot_jacobian_kdl(3);
    jac_solver_rl.JntToJac(q_rl, rl_foot_jacobian_kdl);
    KDL::Jacobian rr_foot_jacobian_kdl(3);
    jac_solver_rr.JntToJac(q_rr, rr_foot_jacobian_kdl);

    KDL::JntArray fl_gravity_torques(3);
    fl_dyn_param.JntToGravity(q_fl, fl_gravity_torques);
    KDL::JntArray fr_gravity_torques(3);
    fr_dyn_param.JntToGravity(q_fr, fr_gravity_torques);
    KDL::JntArray rl_gravity_torques(3);
    rl_dyn_param.JntToGravity(q_rl, rl_gravity_torques);
    KDL::JntArray rr_gravity_torques(3);
    rr_dyn_param.JntToGravity(q_rr, rr_gravity_torques);

    KDL::JntArray fl_coriolis_torques(3);
    fl_dyn_param.JntToCoriolis(q_fl, dq_fl, fl_coriolis_torques);
    KDL::JntArray fr_coriolis_torques(3);
    fr_dyn_param.JntToCoriolis(q_fr, dq_fr, fr_coriolis_torques);
    KDL::JntArray rl_coriolis_torques(3);
    rl_dyn_param.JntToCoriolis(q_rl, dq_rl, rl_coriolis_torques);
    KDL::JntArray rr_coriolis_torques(3);
    rr_dyn_param.JntToCoriolis(q_rr, dq_rr, rr_coriolis_torques);
    
    // Kdl

    robot_state.torques_gravity << fl_gravity_torques(0), fl_gravity_torques(1), fl_gravity_torques(2),
            fr_gravity_torques(0), fr_gravity_torques(1), fr_gravity_torques(2),
            rl_gravity_torques(0), rl_gravity_torques(1), rl_gravity_torques(2),
            rr_gravity_torques(0), rr_gravity_torques(1), rr_gravity_torques(2);

    robot_state.Coriolis_torques << fl_coriolis_torques(0), fl_coriolis_torques(1), fl_coriolis_torques(2),
            fr_coriolis_torques(0), fr_coriolis_torques(1), fr_coriolis_torques(2),
            rl_coriolis_torques(0), rl_coriolis_torques(1), rl_coriolis_torques(2),
            rr_coriolis_torques(0), rr_coriolis_torques(1), rr_coriolis_torques(2);


    Eigen::Matrix3d jacobian_fl;
    for (int i = 0; i < 3; ++i) 
    {
        for (int j = 0; j < 3; ++j) 
        {
            jacobian_fl(i, j) = fl_foot_jacobian_kdl(i, j);
        }
    }
    Eigen::Matrix3d jacobian_fr;
    for (int i = 0; i < 3; ++i) 
    {
        for (int j = 0; j < 3; ++j) 
        {
            jacobian_fr(i, j) = fr_foot_jacobian_kdl(i, j);
        }
    }
    Eigen::Matrix3d jacobian_rl;
    for (int i = 0; i < 3; ++i) 
    {
        for (int j = 0; j < 3; ++j) 
        {
            jacobian_rl(i, j) = rl_foot_jacobian_kdl(i, j);
        }
    }
    Eigen::Matrix3d jacobian_rr;
    for (int i = 0; i < 3; ++i) 
    {
        for (int j = 0; j < 3; ++j) 
        {
            jacobian_rr(i, j) = rr_foot_jacobian_kdl(i, j);
        }
    }

    robot_state.foot_pos_rel.block<3, 1>(0, 0) = fl_foot_pos_vec;
    robot_state.foot_pos_rel.block<3, 1>(0, 1) = fr_foot_pos_vec;
    robot_state.foot_pos_rel.block<3, 1>(0, 2) = rl_foot_pos_vec;
    robot_state.foot_pos_rel.block<3, 1>(0, 3) = rr_foot_pos_vec;

    robot_state.j_foot.block<3, 3>(3 * 0, 3 * 0) = jacobian_fl;
    robot_state.j_foot.block<3, 3>(3 * 1, 3 * 1) = jacobian_fr;
    robot_state.j_foot.block<3, 3>(3 * 2, 3 * 2) = jacobian_rl;
    robot_state.j_foot.block<3, 3>(3 * 3, 3 * 3) = jacobian_rr;

    for (int i = 0; i < NUM_LEG; ++i) 
    {
        Eigen::Matrix3d tmp_mtx = robot_state.j_foot.block<3, 3>(3 * i, 3 * i);
        Eigen::Vector3d tmp_vec = robot_state.joint_vel.segment<3>(3 * i);
        robot_state.foot_vel_rel.block<3, 1>(0, i) = tmp_mtx * tmp_vec;

        robot_state.foot_pos_abs.block<3, 1>(0, i) = robot_state.root_rot_mat * robot_state.foot_pos_rel.block<3, 1>(0, i);
        robot_state.foot_vel_abs.block<3, 1>(0, i) = robot_state.root_rot_mat * robot_state.foot_vel_rel.block<3, 1>(0, i);

        robot_state.foot_pos_world.block<3, 1>(0, i) = robot_state.foot_pos_abs.block<3, 1>(0, i) + robot_state.root_pos;
        robot_state.foot_vel_world.block<3, 1>(0, i) = robot_state.foot_vel_abs.block<3, 1>(0, i) + robot_state.root_lin_vel;
    }
}

// FL
void QuadrupedROS::FL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    robot_state.joint_pos[0] = a1_joint_state.q;
    robot_state.joint_vel[0] = a1_joint_state.dq;
}

void QuadrupedROS::FL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    robot_state.joint_pos[1] = a1_joint_state.q;
    robot_state.joint_vel[1] = a1_joint_state.dq;
}

void QuadrupedROS::FL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    robot_state.joint_pos[2] = a1_joint_state.q;
    robot_state.joint_vel[2] = a1_joint_state.dq;
}

// FR
void QuadrupedROS::FR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    robot_state.joint_pos[3] = a1_joint_state.q;
    robot_state.joint_vel[3] = a1_joint_state.dq;
}

void QuadrupedROS::FR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    robot_state.joint_pos[4] = a1_joint_state.q;
    robot_state.joint_vel[4] = a1_joint_state.dq;
}

void QuadrupedROS::FR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    robot_state.joint_pos[5] = a1_joint_state.q;
    robot_state.joint_vel[5] = a1_joint_state.dq;
}

// RL
void QuadrupedROS::RL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    robot_state.joint_pos[6] = a1_joint_state.q;
    robot_state.joint_vel[6] = a1_joint_state.dq;
}

void QuadrupedROS::RL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    robot_state.joint_pos[7] = a1_joint_state.q;
    robot_state.joint_vel[7] = a1_joint_state.dq;
}

void QuadrupedROS::RL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    robot_state.joint_pos[8] = a1_joint_state.q;
    robot_state.joint_vel[8] = a1_joint_state.dq;
}

// RR
void QuadrupedROS::RR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    robot_state.joint_pos[9] = a1_joint_state.q;
    robot_state.joint_vel[9] = a1_joint_state.dq;
}

void QuadrupedROS::RR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    robot_state.joint_pos[10] = a1_joint_state.q;
    robot_state.joint_vel[10] = a1_joint_state.dq;
}

void QuadrupedROS::RR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state) {
    robot_state.joint_pos[11] = a1_joint_state.q;
    robot_state.joint_vel[11] = a1_joint_state.dq;
}

// foot contact force
void QuadrupedROS::FL_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    robot_state.foot_force[0] = force.wrench.force.z;
}

void QuadrupedROS::FR_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    robot_state.foot_force[1] = force.wrench.force.z;
}

void QuadrupedROS::RL_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    robot_state.foot_force[2] = force.wrench.force.z;
}

void QuadrupedROS::RR_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    robot_state.foot_force[3] = force.wrench.force.z;
}


void QuadrupedROS::Robot_command_callback(const unitree_legged_msgs::RobotCmd::ConstPtr &cmd_msg)
{
    if(cmd_msg->vel_x >= CMD_VELX_MAX)
        cmd_velx = CMD_VELX_MAX;
    else
        cmd_velx = cmd_msg->vel_x;

    if(cmd_msg->vel_y >= CMD_VELY_MAX)
        cmd_vely = CMD_VELY_MAX;
    else
        cmd_vely = cmd_msg->vel_y;

    if(cmd_msg->vel_z >= CMD_BODY_HEIGHT_VEL)
        cmd_velz = CMD_BODY_HEIGHT_VEL;
    else
        cmd_velz = cmd_msg->vel_z;

    if(cmd_msg->angular_vel_roll >= CMD_ROLL_MAX)
        cmd_roll_vel = CMD_ROLL_MAX;
    else
        cmd_roll_vel = cmd_msg->angular_vel_roll;

    if(cmd_msg->angular_vel_pitch >= CMD_PITCH_MAX)
        cmd_pitch_vel = CMD_PITCH_MAX;
    else
        cmd_pitch_vel = cmd_msg->angular_vel_pitch;
    
    if(cmd_msg->angular_vel_yaw >= CMD_YAW_MAX)
        cmd_yaw_vel = CMD_YAW_MAX;
    else
        cmd_yaw_vel = cmd_msg->angular_vel_yaw;

    // robot_state.root_lin_vel_d[0] = cmd_msg->Twist.linear.x;
    // robot_state.root_lin_vel_d[1] = cmd_msg->Twist.linear.y;
    // robot_state.root_lin_vel_d[2] = cmd_msg->Twist.linear.z;

    // // root_ang_vel_d is in robot frame
    // robot_state.root_ang_vel_d[0] = cmd_msg->Twist.angular.x;   
    // robot_state.root_ang_vel_d[1] = cmd_msg->Twist.angular.y;
    // robot_state.root_ang_vel_d[2] = cmd_msg->Twist.angular.z;

    // MovingWindowFilter quat_w_d;
    // MovingWindowFilter quat_x_d;
    // MovingWindowFilter quat_y_d;
    // MovingWindowFilter quat_z_d;

    // quat_w_d = MovingWindowFilter(5);
    // quat_x_d = MovingWindowFilter(5);
    // quat_y_d = MovingWindowFilter(5);
    // quat_z_d = MovingWindowFilter(5);

    // Eigen::Quaterniond root_quat_d;

    // root_quat_d = Eigen::Quaterniond(quat_w_d.CalculateAverage(cmd_msg->Pose.orientation.w),
    //                                  quat_x_d.CalculateAverage(cmd_msg->Pose.orientation.x),
    //                                  quat_y_d.CalculateAverage(cmd_msg->Pose.orientation.y),
    //                                  quat_z_d.CalculateAverage(cmd_msg->Pose.orientation.z));


    // Eigen::Vector3d root_euler_cmd = Utils::quat_to_euler(root_quat_d);

    // robot_state.root_euler_d[0] = root_euler_cmd[0];
    // robot_state.root_euler_d[1] = root_euler_cmd[1];
    // robot_state.root_euler_d[2] = root_euler_cmd[2];
    // robot_state.root_pos_d[2] = cmd_msg->Pose.position.z;

    // std::cout << "robot_state.root_pos: " << robot_state.root_pos << std::endl;

    robot_ctrl_state = cmd_msg->mode;
}
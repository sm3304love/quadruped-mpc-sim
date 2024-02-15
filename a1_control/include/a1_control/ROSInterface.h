#ifndef ROSInterface_H
#define ROSInterface_H

// std
#include <Eigen/Dense>
#include <memory>
#include <set>
#include <chrono>
#include <map>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <fstream>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/RobotCmd.h>
// #include <gazebo_msgs/ModelStates.h>

// control parameters
#include "a1_control/Params.h"
#include "a1_control/RobotState.h"
#include "a1_control/RobotController.h"
#include "a1_control/Ekf.h"
#include "a1_control/Utils.h"
#include "a1_control/Filter.h"


class QuadrupedROS {
public:
    QuadrupedROS(ros::NodeHandle &_nh);

    bool update_foot_forces_grf(double dt);

    bool main_update(double t, double dt);

    bool send_cmd();

    void imu_callback(const sensor_msgs::Imu::ConstPtr &imu);

    void FL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void FL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void FL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void FR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void FR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void FR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void RL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void RL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void RL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void RR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void RR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void RR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void FL_foot_contact_callback(const geometry_msgs::WrenchStamped &force);

    void FR_foot_contact_callback(const geometry_msgs::WrenchStamped &force);

    void RL_foot_contact_callback(const geometry_msgs::WrenchStamped &force);

    void RR_foot_contact_callback(const geometry_msgs::WrenchStamped &force);

    // void joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg);

    void Robot_command_callback(const unitree_legged_msgs::RobotCmd::ConstPtr &cmd_msg);


private:
    ros::NodeHandle nh;

    // 0,  1,  2: FL_hip, FL_thigh, FL_calf
    // 3,  4,  5: FR_hip, FR_thigh, FR_calf
    // 6,  7,  8: RL_hip, RL_thigh, RL_calf
    // 9, 10, 11: RR_hip, RR_thigh, RR_calf
    ros::Publisher pub_joint_cmd[12];
    ros::Subscriber sub_joint_msg[12];
    ros::Publisher pub_euler_d;

    // 0, 1, 2, 3: FL, FR, RL, RR
    ros::Subscriber sub_foot_contact_msg[4];
    ros::Subscriber sub_gt_pose_msg;
    ros::Subscriber sub_imu_msg;
    ros::Subscriber sub_joy_msg;
    ros::Subscriber sub_robot_cmd_msg;

    // debug estimation
    ros::Publisher pub_estimated_pose;
    
    // following cade is also in VILEOM
    // following are some parameters that defines the transformation between IMU frame(b) and robot body frame(r)
    Eigen::Vector3d p_br;
    Eigen::Matrix3d R_br;
    // for each leg, there is an offset between the body frame and the hip motor (fx, fy)
    double leg_offset_x[4] = {};
    double leg_offset_y[4] = {};
    // for each leg, there is an offset between the body frame and the hip motor (fx, fy)
    double motor_offset[4] = {};
    double upper_leg_length[4] = {};
    double lower_leg_length[4] = {};
    std::vector<Eigen::VectorXd> rho_fix_list;
    std::vector<Eigen::VectorXd> rho_opt_list;
    // variables related to control
    RobotStates robot_state;
    RobotController _root_control;
    EKFFilter robot_pos_estimate;

    // filters
    MovingWindowFilter acc_x;
    MovingWindowFilter acc_y;
    MovingWindowFilter acc_z;
    MovingWindowFilter gyro_x;
    MovingWindowFilter gyro_y;
    MovingWindowFilter gyro_z;
    MovingWindowFilter quat_w;
    MovingWindowFilter quat_x;
    MovingWindowFilter quat_y;
    MovingWindowFilter quat_z;

    int robot_ctrl_state = 0;
    int prev_robot_ctrl_state = 0;

    float cmd_velx = 0.0;
    float cmd_vely = 0.0;
    float cmd_velz = 0.0;

    float cmd_roll_vel = 0.0;
    float cmd_pitch_vel = 0.0;
    float cmd_yaw_vel = 0.0;
    float cmd_body_height = 0.0;


};


#endif //ROSInterface_H
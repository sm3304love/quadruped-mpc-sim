
#ifndef RobotController_H
#define RobotController_H

#include <chrono>
#include <iostream>
#include <string>

// to debug
#include <geometry_msgs/PointStamped.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Dense>

#include "a1_control/ConvexMpc.h"
#include "a1_control/Filter.h"
#include "a1_control/Params.h"
#include "a1_control/RobotState.h"
#include "a1_control/Utils.h"

class RobotController
{
  public:
    RobotController();

    RobotController(ros::NodeHandle &_nh);

    void update_plan(RobotStates &state, double dt);

    void generate_swing_legs_ctrl(RobotStates &state, double dt);

    void compute_joint_torques(RobotStates &state);

    Eigen::Matrix<double, 3, NUM_LEG> compute_grf(RobotStates &state, double dt);

    Eigen::Vector3d compute_walking_surface(RobotStates &state);

  private:
    BezierUtils bezierUtils[NUM_LEG];

    Eigen::Matrix<double, 6, 1> root_acc;
    // allocate the problem weight matrices
    Eigen::DiagonalMatrix<double, 6> Q;
    double R;
    // ground friction coefficient
    double mu;
    double F_min;
    double F_max;
    // allocate QP problem matrices and vectors
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    // qpOASES::Solver solver;

    // add a number of ROS debug topics
    ros::NodeHandle nh;

    // debug topics
    //    ros::Publisher pub_root_lin_vel;
    //    ros::Publisher pub_root_lin_vel_d;
    ros::Publisher pub_terrain_angle;

    ros::Publisher pub_foot_pose_target_FL;
    ros::Publisher pub_foot_pose_target_FR;
    ros::Publisher pub_foot_pose_target_RL;
    ros::Publisher pub_foot_pose_target_RR;

    ros::Publisher pub_foot_pose_target_rel_FL;
    ros::Publisher pub_foot_pose_target_rel_FR;
    ros::Publisher pub_foot_pose_target_rel_RL;
    ros::Publisher pub_foot_pose_target_rel_RR;

    ros::Publisher pub_foot_pose_error_FL;
    ros::Publisher pub_foot_pose_error_FR;
    ros::Publisher pub_foot_pose_error_RL;
    ros::Publisher pub_foot_pose_error_RR;

    ros::Publisher pub_euler;

    // MPC does not start for the first 10 ticks to prevent uninitialized NAN goes into joint_torques
    int mpc_init_counter;

    std::string use_sim_time;

    // filters
    MovingWindowFilter terrain_angle_filter;
    MovingWindowFilter recent_contact_x_filter[NUM_LEG];
    MovingWindowFilter recent_contact_y_filter[NUM_LEG];
    MovingWindowFilter recent_contact_z_filter[NUM_LEG];
};

#endif // RobotController_H
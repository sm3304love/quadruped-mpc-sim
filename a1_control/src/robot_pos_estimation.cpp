#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <a1_control/Ekf.h>
#include <a1_control/Filter.h>
#include <nav_msgs/Odometry.h>
#include <unitree_legged_msgs/MotorState.h>

class RobotStateEstimator
{
public:
    RobotStateEstimator()
    {
        estimated_odom_pub = n.advertise<nav_msgs::Odometry>("/estimation_body_pose", 1000);
        FL_calf_sub = n.subscribe("/a1_gazebo/FL_calf_controller/state", 1000, &RobotStateEstimator::FL_calf_state_callback, this);
        FL_hip_sub = n.subscribe("/a1_gazebo/FL_hip_controller/state", 1000, &RobotStateEstimator::FL_hip_state_callback, this);
        FL_thigh_sub = n.subscribe("/a1_gazebo/FL_thigh_controller/state", 1000, &RobotStateEstimator::FL_thigh_state_callback, this);
        FR_calf_sub = n.subscribe("/a1_gazebo/FR_calf_controller/state", 1000, &RobotStateEstimator::FR_calf_state_callback, this);
        FR_hip_sub = n.subscribe("/a1_gazebo/FR_hip_controller/state", 1000, &RobotStateEstimator::FR_hip_state_callback, this);
        FR_thigh_sub = n.subscribe("/a1_gazebo/FR_thigh_controller/state", 1000, &RobotStateEstimator::FR_thigh_state_callback, this);
        RL_calf_sub = n.subscribe("/a1_gazebo/RL_calf_controller/state", 1000, &RobotStateEstimator::RL_calf_state_callback, this);
        RL_hip_sub = n.subscribe("/a1_gazebo/RL_hip_controller/state", 1000, &RobotStateEstimator::RL_hip_state_callback, this);
        RL_thigh_sub = n.subscribe("/a1_gazebo/RL_thigh_controller/state", 1000, &RobotStateEstimator::RL_thigh_state_callback, this);
        RR_calf_sub = n.subscribe("/a1_gazebo/RR_calf_controller/state", 1000, &RobotStateEstimator::RR_calf_state_callback, this);
        RR_hip_sub = n.subscribe("/a1_gazebo/RR_hip_controller/state", 1000, &RobotStateEstimator::RR_hip_state_callback, this);
        RR_thigh_sub = n.subscribe("/a1_gazebo/RR_thigh_controller/state", 1000, &RobotStateEstimator::RR_thigh_state_callback, this);

        imu_sub = n.subscribe("/trunk_imu", 1000, &RobotStateEstimator::imu_callback, this);


        robot_state.base_link = "trunk";
        robot_state.fl_foot_link = "FL_foot";
        robot_state.fr_foot_link = "FR_foot";
        robot_state.rl_foot_link = "RL_foot";
        robot_state.rr_foot_link = "RR_foot";

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

    void FL_hip_state_callback(const unitree_legged_msgs::MotorState& msg)
    {
        robot_state.joint_pos[0] = msg.q;
        robot_state.joint_vel[0] = msg.dq;
    }
    void FL_thigh_state_callback(const unitree_legged_msgs::MotorState& msg)
    {
        robot_state.joint_pos[1] = msg.q;
        robot_state.joint_vel[1] = msg.dq;
    }
    void FL_calf_state_callback(const unitree_legged_msgs::MotorState& msg)
    {
        robot_state.joint_pos[2] = msg.q;
        robot_state.joint_vel[2] = msg.dq;
    }
    void FR_hip_state_callback(const unitree_legged_msgs::MotorState& msg)
    {
        robot_state.joint_pos[3] = msg.q;
        robot_state.joint_vel[3] = msg.dq;
    }
    void FR_thigh_state_callback(const unitree_legged_msgs::MotorState& msg)
    {
        robot_state.joint_pos[4] = msg.q;
        robot_state.joint_vel[4] = msg.dq;
    }
    void FR_calf_state_callback(const unitree_legged_msgs::MotorState& msg)
    {
        robot_state.joint_pos[5] = msg.q;
        robot_state.joint_vel[5] = msg.dq;
    }
    void RL_hip_state_callback(const unitree_legged_msgs::MotorState& msg)
    {
        robot_state.joint_pos[6] = msg.q;
        robot_state.joint_vel[6] = msg.dq;
    }
    void RL_thigh_state_callback(const unitree_legged_msgs::MotorState& msg)
    {
        robot_state.joint_pos[7] = msg.q;
        robot_state.joint_vel[7] = msg.dq;
    }
    void RL_calf_state_callback(const unitree_legged_msgs::MotorState& msg)
    {
        robot_state.joint_pos[8] = msg.q;
        robot_state.joint_vel[8] = msg.dq;
    }
    void RR_hip_state_callback(const unitree_legged_msgs::MotorState& msg)
    {
        robot_state.joint_pos[9] = msg.q;
        robot_state.joint_vel[9] = msg.dq;
    }
    void RR_thigh_state_callback(const unitree_legged_msgs::MotorState& msg)
    {
        robot_state.joint_pos[10] = msg.q;
        robot_state.joint_vel[10] = msg.dq;
    }
    void RR_calf_state_callback(const unitree_legged_msgs::MotorState& msg)
    {
        robot_state.joint_pos[11] = msg.q;
        robot_state.joint_vel[11] = msg.dq;
    }

    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
    {   
        robot_state.root_quat = Eigen::Quaterniond(quat_w.CalculateAverage(msg->orientation.w),
                                                  quat_x.CalculateAverage(msg->orientation.x),
                                                  quat_y.CalculateAverage(msg->orientation.y),
                                                  quat_z.CalculateAverage(msg->orientation.z));

        robot_state.root_rot_mat = robot_state.root_quat.toRotationMatrix();

        robot_state.imu_acc = Eigen::Vector3d(
            acc_x.CalculateAverage(msg->linear_acceleration.x),
            acc_y.CalculateAverage(msg->linear_acceleration.y),
            acc_z.CalculateAverage(msg->linear_acceleration.z)
    );
        robot_state.imu_ang_vel = Eigen::Vector3d(
                gyro_x.CalculateAverage(msg->angular_velocity.x),
                gyro_y.CalculateAverage(msg->angular_velocity.y),
                gyro_z.CalculateAverage(msg->angular_velocity.z)
    );
        robot_state.root_ang_vel = robot_state.root_rot_mat * robot_state.imu_ang_vel;

        robot_state.root_euler = Utils::quat_to_euler(robot_state.root_quat);
        double yaw_angle = robot_state.root_euler[2];

        robot_state.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());
    }
    
    void run()
    {
        ros::Rate loop_rate(1000);

        while(ros::ok())
        {   
            KDL::JntArray q_fl(3);
            q_fl(0) = robot_state.joint_pos[0];
            q_fl(1) = robot_state.joint_pos[1];
            q_fl(2) = robot_state.joint_pos[2];
            KDL::JntArray q_fr(3);
            q_fr(0) = robot_state.joint_pos[3];
            q_fr(1) = robot_state.joint_pos[4];
            q_fr(2) = robot_state.joint_pos[5];
            KDL::JntArray q_rl(3);
            q_rl(0) = robot_state.joint_pos[6];
            q_rl(1) = robot_state.joint_pos[7];
            q_rl(2) = robot_state.joint_pos[8];
            KDL::JntArray q_rr(3);
            q_rr(0) = robot_state.joint_pos[9];
            q_rr(1) = robot_state.joint_pos[10];
            q_rr(2) = robot_state.joint_pos[11];

            KDL::JntArray dq_fl(3);
            dq_fl(0) = robot_state.joint_vel[0];
            dq_fl(1) = robot_state.joint_vel[1];
            dq_fl(2) = robot_state.joint_vel[2];

            KDL::JntArray dq_fr(3);
            dq_fr(0) = robot_state.joint_vel[3];
            dq_fr(1) = robot_state.joint_vel[4];
            dq_fr(2) = robot_state.joint_vel[5];

            KDL::JntArray dq_rl(3);
            dq_rl(0) = robot_state.joint_vel[6];
            dq_rl(1) = robot_state.joint_vel[7];
            dq_rl(2) = robot_state.joint_vel[8];

            KDL::JntArray dq_rr(3);
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

            if (!robot_pos_estimate.is_inited()) 
            {
                robot_pos_estimate.init_state(robot_state);
            } else {
                robot_pos_estimate.update_estimation(robot_state, 0.001); //sec
            }


            nav_msgs::Odometry estimate_odom;
            estimate_odom.pose.pose.position.x = robot_state.estimated_root_pos(0);
            estimate_odom.pose.pose.position.y = robot_state.estimated_root_pos(1);
            estimate_odom.pose.pose.position.z = robot_state.estimated_root_pos(2);
            estimate_odom.pose.pose.orientation.w = robot_state.root_quat.w();
            estimate_odom.pose.pose.orientation.x = robot_state.root_quat.x();
            estimate_odom.pose.pose.orientation.y = robot_state.root_quat.y();
            estimate_odom.pose.pose.orientation.z = robot_state.root_quat.z();

            // make sure root_lin_vel is in world frame
            estimate_odom.twist.twist.linear.x = robot_state.estimated_root_vel(0);
            estimate_odom.twist.twist.linear.y = robot_state.estimated_root_vel(1);
            estimate_odom.twist.twist.linear.z = robot_state.estimated_root_vel(2);
            estimate_odom.twist.twist.angular.x = robot_state.root_ang_vel(0);
            estimate_odom.twist.twist.angular.y = robot_state.root_ang_vel(1);
            estimate_odom.twist.twist.angular.z = robot_state.root_ang_vel(2);


            estimated_odom_pub.publish(estimate_odom);

            loop_rate.sleep();
        }
    }


private:
    ros::NodeHandle n;

    RobotStates robot_state;

    ros::Publisher estimated_odom_pub;

    ros::Subscriber FL_calf_sub;
    ros::Subscriber FL_hip_sub;
    ros::Subscriber FL_thigh_sub;

    ros::Subscriber FR_calf_sub;
    ros::Subscriber FR_hip_sub;
    ros::Subscriber FR_thigh_sub;

    ros::Subscriber RL_calf_sub;
    ros::Subscriber RL_hip_sub;
    ros::Subscriber RL_thigh_sub;

    ros::Subscriber RR_calf_sub;
    ros::Subscriber RR_hip_sub;
    ros::Subscriber RR_thigh_sub;

    ros::Subscriber imu_sub;

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
    EKFFilter robot_pos_estimate;

    // // following are some parameters that defines the transformation between IMU frame(b) and robot body frame(r)
    // Eigen::Vector3d p_br;   
    // Eigen::Matrix3d R_br;
    // // for each leg, there is an offset between the body frame and the hip motor (fx, fy)
    // double leg_offset_x[4] = {};
    // double leg_offset_y[4] = {};
    // // for each leg, there is an offset between the body frame and the hip motor (fx, fy)
    // double motor_offset[4] = {};
    // double upper_leg_length[4] = {};
    // double lower_leg_length[4] = {};
    // std::vector<Eigen::VectorXd> rho_fix_list;
    // std::vector<Eigen::VectorXd> rho_opt_list;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pos_estimation_node");

    RobotStateEstimator node;

    ros::AsyncSpinner spinner(10);
    spinner.start();

    node.run();
    
    return 0;
}
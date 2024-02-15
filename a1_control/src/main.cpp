
#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>

// control parameters
#include "a1_control/Params.h"
// A1 control
#include "a1_control/ROSInterface.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "gazebo_a1_qp_ctrl");
    ros::NodeHandle nh;

    // change ros logger
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    // make sure the ROS infra using sim time, otherwise the controller cannot run with correct time steps
    std::string use_sim_time;
    if (ros::param::get("/use_sim_time", use_sim_time)) {
        if (use_sim_time != "true") {
            std::cout << "ROS must set use_sim_time in order to use this program!" << std::endl;
            return -1;
        }
    }

    // create a1 controller
    std::unique_ptr<QuadrupedROS> robot = std::make_unique<QuadrupedROS>(nh);

    std::atomic<bool> control_execute{};
    control_execute.store(true, std::memory_order_release);

    // Thread 1: compute desired ground forces
    std::cout << "Enter thread 1: compute desired ground forces" << std::endl;
    std::thread compute_foot_forces_grf_thread([&]() {
        // prepare variables to monitor time and control the while loop
        ros::Time start = ros::Time::now();
        ros::Time prev = ros::Time::now();
        ros::Time now = ros::Time::now();  // bool res = app.exec();
        ros::Duration dt(0);

        while (control_execute.load(std::memory_order_acquire) && ros::ok()) {

            ros::Duration(GRF_UPDATE_FREQUENCY / 1000).sleep();

            // get t and dt
            now = ros::Time::now();
            dt = now - prev;
            prev = now;
            ros::Duration elapsed = now - start;

            auto t1 = std::chrono::high_resolution_clock::now();

            // compute desired ground forces
            bool running = robot->update_foot_forces_grf(dt.toSec());

            std::cout << "Thread 1 is updated in " << dt.toSec() << "s" << std::endl;

            auto t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> ms_double = t2 - t1;

            if (!running) {
                std::cout << "Thread 1 loop is terminated because of errors." << std::endl;
                ros::shutdown();
                std::terminate();
                break;
            }
        }
    });

    // Thread 2: update robot states, compute desired swing legs forces, compute desired joint torques, and send commands
    std::cout << "Enter thread 2: update robot states, compute desired swing legs forces, compute desired joint torques, and send commands"
              << std::endl;
    std::thread main_thread([&]() {
        // prepare variables to monitor time and control the while loop
        ros::Time start = ros::Time::now();
        ros::Time prev = ros::Time::now();
        ros::Time now = ros::Time::now();  // bool res = app.exec();
        ros::Duration dt(0);

        while (control_execute.load(std::memory_order_acquire) && ros::ok()) {
            auto t3 = std::chrono::high_resolution_clock::now();

            ros::Duration(MAIN_UPDATE_FREQUENCY / 1000).sleep();

            // get t and dt
            now = ros::Time::now();
            dt = now - prev;
            prev = now;
            ros::Duration elapsed = now - start;

            // compute desired ground forces
            bool main_update_running = robot->main_update(elapsed.toSec(), dt.toSec());
            bool send_cmd_running = robot->send_cmd();
            // bool send_cmd_running = 1;

            auto t4 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> ms_double = t4 - t3;
            std::cout << "Thread 2 is updated in " << ms_double.count() << "ms" << std::endl;

            if (!main_update_running || !send_cmd_running) {
                std::cout << "Thread 2 loop is terminated because of errors." << std::endl;
                ros::shutdown();
                std::terminate();
                break;
            }
        }
    });

    ros::AsyncSpinner spinner(24);
    spinner.start();

    compute_foot_forces_grf_thread.join();
    main_thread.join();

    return 0;
}
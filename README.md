# Unitree A1 Convex MPC based controller

I modified [A1-QP-MPC-Controller](https://github.com/ShuoYangRobotics/A1-QP-MPC-Controller) repository repository to use the KDL library for obtaining kinematic and dynamic parameters.

By utilizing the KDL library, kinematic and dynamic parameters can be easily obtained, making it easier to apply the algorithm to other quadruped robots in the future

## Dependencies
1. ROS Noetic
2. KDL library
3. qpOASES

## How to launch
### Launch unitree A1 gazebo simulation
```
roslaunch unitree_gazebo normal.launch rname:=a1 wname:=earth
```

### Launch A1 Convex MPC controller
```
roslaunch a1_control a1_ctrl.launch
```

### Send veloicity command to robot
```
rostopic pub /a1_robot_cmd unitree_legged_msgs/RobotCmd "{vel_x: 0.0, vel_y: 0.0, vel_z: 0.1, angular_vel_roll: 0.0, angular_vel_pitch: 0.0, angular_vel_yaw: 0.0, mode: 1.0}"
```
* mode 0 is stand mode

* mode 1 is walking mode (swing legs)

![walk](https://github.com/sm3304love/quadruped-control-sim/assets/57741032/d1d4c747-6466-4595-981a-1ed376c8bb4e)


## Reference
[A1-QP-MPC-Controller](https://github.com/ShuoYangRobotics/A1-QP-MPC-Controller)

[MIT Cheetah 3: Design and Control of a Robust, Dynamic Quadruped Robot](https://ieeexplore.ieee.org/abstract/document/8593885)

[Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control](https://ieeexplore.ieee.org/abstract/document/8594448)

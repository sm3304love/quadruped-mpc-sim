cmake_minimum_required(VERSION 3.0.2)
project(a1_control)

find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package (Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  unitree_legged_msgs
  sensor_msgs
  eigen_conversions
  nav_msgs
  visualization_msgs
)

set(qpOASES_DIR "/home/harco/qpOASES")


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES a1_control
 CATKIN_DEPENDS roscpp std_msgs unitree_legged_msgs sensor_msgs nav_msgs visualization_msgs
 DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS} 
  ${kdl_parser_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS}
  ${qpOASES_DIR}/include
  include
)

add_library(${PROJECT_NAME}
  src/Ekf.cpp
  src/Utils.cpp
  src/ConvexMpc.cpp
  src/RobotController.cpp
  src/ROSInterface.cpp
)


add_executable(robot_pos_estimation src/robot_pos_estimation.cpp)
target_link_libraries(robot_pos_estimation ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES} ${PROJECT_NAME} ${qpOASES_DIR}/bin/libqpOASES.so)
add_dependencies(robot_pos_estimation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES} ${PROJECT_NAME} ${qpOASES_DIR}/bin/libqpOASES.so)
add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
cmake_minimum_required(VERSION 3.0.2)
project(force_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_definitions(-D__LINUX__)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosmsg
  rospy
hirop_msgs

  eigen_conversions
  std_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_perception
  tf
)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include/force_bridge
/usr/local/include/hirop
  ${catkin_INCLUDE_DIRS}
)

link_directories(/usr/local/lib)



add_executable(force_bridge
        src/main.cpp
        include/force_bridge/forceService.h
        src/forceService.cpp
        )

target_link_libraries(force_bridge
    hirop_force
    hsImpenderrForce
    dl
    yaml-cpp
   ${catkin_LIBRARIES}
 )


add_executable(shakeHandJudge src/shakeHandJudge.cpp)
target_link_libraries(shakeHandJudge ${catkin_LIBRARIES})

add_executable(node_daq src/node_daq.cpp)
target_link_libraries(node_daq ${catkin_LIBRARIES}   yaml-cpp)

add_executable(motionCoorControl src/motionCoorControl.cpp)
target_link_libraries(motionCoorControl ${catkin_LIBRARIES})
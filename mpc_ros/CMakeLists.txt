cmake_minimum_required(VERSION 3.5)
project(mpc_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11 -O3)

find_package(catkin REQUIRED 
  COMPONENTS
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_msgs
    pluginlib
    move_base
    base_local_planner
    roscpp
    std_msgs
    tf
    visualization_msgs
    quintic_polynomials_planner_ros
  )
find_package(Boost REQUIRED
  COMPONENTS
    thread
    )
find_package(OpenCV REQUIRED)

include_directories( 
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/include
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/MPCPlanner.cfg
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    mpc_ros
  CATKIN_DEPENDS 
    costmap_2d 
    dynamic_reconfigure 
    geometry_msgs 
    move_base 
    roscpp 
    std_msgs 
    tf 
    visualization_msgs 
    pluginlib
    quintic_polynomials_planner_ros
)

# MPC Local planner plugin
add_library(mpc_ros 
  src/mpc_planner_ros.cpp 
  src/mpc_planner.cpp
  src/driving_state.cpp)
add_dependencies(mpc_ros mpc_ros_gencfg)

target_link_libraries(mpc_ros 
  ipopt
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  quintic_polynomial_planner
  /usr/local/lib
)


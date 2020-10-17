cmake_minimum_required(VERSION 2.8.3)
project(hydrone_aerial_mission_planner)

add_definitions(-std=c++11)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  mavros_msgs
  geographic_msgs
  trajectory_msgs
)

find_package(catkin REQUIRED COMPONENTS gazebo_msgs geometry_msgs mav_msgs roscpp sensor_msgs)

catkin_package(
  CATKIN_DEPENDS
    gazebo_msgs
    geometry_msgs
    mav_msgs
    roscpp
    sensor_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hovering_example_arena src/hovering_example_arena.cpp)
target_link_libraries(hovering_example_arena ${catkin_LIBRARIES})
add_dependencies(hovering_example_arena ${catkin_EXPORTED_TARGETS})

add_executable(waypoint_publisher_file_arena src/waypoint_publisher_file_arena.cpp)
target_link_libraries(waypoint_publisher_file_arena ${catkin_LIBRARIES})
add_dependencies(waypoint_publisher_file_arena ${catkin_EXPORTED_TARGETS})

cmake_minimum_required(VERSION 2.8.3)
project(inspection_task)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED)


catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp std_msgs cv_bridge sensor_msgs image_transport                
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

set(SOURCES src/inspection_task.cpp include/detect_number.cpp)
add_executable(${PROJECT_NAME} ${SOURCES}) 

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

target_link_libraries( inspection_task ${OpenCV_LIBS} ${catkin_LIBRARIES} )

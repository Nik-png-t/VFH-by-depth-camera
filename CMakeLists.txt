cmake_minimum_required(VERSION 3.0.2)
project(vfh_by_depth_cam)
set(CMAKE_CXX_STANDARD 17)

set(OpenCV_DIR /home/nikita/Документы/opencv/build)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  visualization_msgs
  sensor_msgs
  roscpp
  std_msgs
  pcl_ros
  cv_bridge)

include_directories(
  header
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(vfh_algorithm STATIC src/vfh_algorithm.cpp)

add_executable(vfh_node src/vfh_node.cpp)

target_link_libraries(vfh_node vfh_algorithm ${OpenCV_LIBS} ${catkin_LIBRARIES})

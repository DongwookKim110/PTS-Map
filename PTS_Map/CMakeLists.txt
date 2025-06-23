cmake_minimum_required(VERSION 3.0.2)
project(pts_map)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  geometry_msgs
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(Boost 1.54 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include
)

link_directories(
  ${PCL_INCLUDE_DIRS}
  include
)
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

add_executable(PTS_Map src/PTS_Map.cpp src/main.cpp)
target_link_libraries(PTS_Map ${catkin_LIBRARIES} ${PCL_LIBRARIES})

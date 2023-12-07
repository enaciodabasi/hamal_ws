cmake_minimum_required(VERSION 3.0.2)
project(hamal_localization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs tf2_ros tf2_geometry_msgs)
find_package(yaml-cpp REQUIRED)

catkin_package(
    INCLUDE_DIRS include/
    LIBRARIES
    CATKIN_DEPENDS roscpp geometry_msgs
    DEPENDS
)

include_directories(
    include/
)


add_executable(position_saver src/position_saver.cpp)
target_link_libraries(position_saver ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
target_include_directories(position_saver PUBLIC ${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})

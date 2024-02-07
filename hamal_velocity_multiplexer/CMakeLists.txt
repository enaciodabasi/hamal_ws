cmake_minimum_required(VERSION 3.0.2)
project(hamal_velocity_multiplexer)

find_package(catkin REQUIRED COMPONENTS

    roscpp
    geometry_msgs
    hamal_custom_interfaces
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hamal_velocity_multiplexer
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)


add_library(hamal_velocity_multiplexer SHARED src/velocity_multiplexer.cpp)
target_link_libraries(hamal_velocity_multiplexer ${catkin_LIBRARIES})
target_include_directories(hamal_velocity_multiplexer PUBLIC ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
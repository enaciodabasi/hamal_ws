cmake_minimum_required(VERSION 3.0.2)
project(hamal_utils)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    teb_local_planner
    diff_drive_controller_hamal
    dynamic_reconfigure
    hamal_custom_interfaces
)


catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES hamal_utils
   CATKIN_DEPENDS diff_drive_controller_hamal roscpp teb_local_planner dynamic_reconfigure hamal_custom_interfaces
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

add_executable(dynamic_speed_limiter src/dynamic_speed_limiter.cpp)
target_link_libraries(dynamic_speed_limiter ${catkin_LIBRARIES})
target_include_directories(dynamic_speed_limiter PUBLIC ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include/)

install(
    TARGETS dynamic_speed_limiter
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
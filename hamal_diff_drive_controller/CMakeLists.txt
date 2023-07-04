cmake_minimum_required(VERSION 3.0.2)
project(hamal_diff_drive_controller)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    controller_interface
    dynamic_reconfigure
    geometry_msgs
    nav_msgs
    hardware_interface
    realtime_tools
    pluginlib
    control_msgs
    tf
)

find_package(Boost REQUIRED)

generate_dynamic_reconfigure_options(cfg/HamalDiffDriveController.cfg)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    controller_interface
    control_msgs
    dynamic_reconfigure
    geometry_msgs
    hardware_interface
    nav_msgs
    realtime_tools
    tf
    LIBRARIES ${PROJECT_NAME}
    DEPENDS Boost
)

include_directories(
    include ${catkin_INCLUDE_DIRS}
    include ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/hamal_diff_drive_controller.cpp src/odometry.cpp src/limiter.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

install(DIRECTORY include/${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(FILES hamal_diff_drive_controller_plugings.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
cmake_minimum_required(VERSION 3.0.2)
project(hamal_control)

find_package(catkin REQUIRED COMPONENTS 
    controller_manager
    diff_drive_controller_hamal
    hamal_lifter_controller
    hardware_interface
    roscpp
    actionlib
    control_msgs
    std_msgs
    pluginlib
    tf2_geometry_msgs
    geometry_msgs
    move_base_msgs
    hamal_custom_interfaces
)

find_package(Boost REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
    CATKIN_DEPENDS controller_manager diff_drive_controller_hamal roscpp actionlib control_msgs std_msgs
    DEPENDS Boost
)

include_directories(
    include/
)


#add_dependencies(hamal_lifter_controller ${hamal_lifter_controller_EXPORTED_TARGETS})

add_executable(orientation_test src/orientation_test.cpp)
target_link_libraries(orientation_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_include_directories(orientation_test PUBLIC ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include/)


# install(TARGETS lifter_controller_client
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

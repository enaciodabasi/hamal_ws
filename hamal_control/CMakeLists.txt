cmake_minimum_required(VERSION 3.0.2)
project(hamal_control)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS 
    controller_manager
    
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
    tf2_ros
    dynamic_reconfigure
)

find_package(Boost REQUIRED)


generate_dynamic_reconfigure_options(
    cfg/MotionController.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
    CATKIN_DEPENDS controller_manager  roscpp actionlib control_msgs std_msgs tf2_ros
    DEPENDS Boost
)

include_directories(
    include/
)

#add_dependencies(hamal_lifter_controller ${hamal_lifter_controller_EXPORTED_TARGETS})

add_executable(orientation_test src/orientation_test.cpp)
target_link_libraries(orientation_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_include_directories(orientation_test PUBLIC ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include/)

add_library(velocity_smoother STATIC src/velocity_smoother/src/velocity_smoother.cpp)
target_link_libraries(velocity_smoother ${catkin_LIBRARIES})
target_include_directories(velocity_smoother PUBLIC ${catkin_INCLUDE_DIRS} include/hamal_control/velocity_smoother/include/)


add_executable(trapezoidal_profile src/trapezoid_move.cpp)
add_dependencies(trapezoidal_profile ${PROJECT_NAME}_gencfg)
target_link_libraries(trapezoidal_profile ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_include_directories(trapezoidal_profile PUBLIC ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include/)

# install(TARGETS lifter_controller_client
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS velocity_smoother
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

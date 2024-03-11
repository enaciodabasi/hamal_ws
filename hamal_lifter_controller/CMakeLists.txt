cmake_minimum_required(VERSION 3.0.2)
project(hamal_lifter_controller)

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
    hamal_custom_interfaces
)
find_package(Boost REQUIRED)
find_package(MotionProfileGenerators REQUIRED)


catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS controller_manager roscpp actionlib control_msgs std_msgs hamal_custom_interfaces
    LIBRARIES ${PROJECT_NAME}
    DEPENDS Boost
)

include_directories(
    include/
    include ${catkin_INCLUDE_DIRS}
    include ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/hamal_lifter_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${MotionProfileGenerators_LIB_INSTALL_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS} ${MotionProfileGenerators_INCLUDE_DIR})

install(DIRECTORY include/${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(FILES hamal_lifter_controller_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
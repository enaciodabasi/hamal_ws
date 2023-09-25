cmake_minimum_required(VERSION 3.0.2)
project(hamal_hardware)

set(CMAKE_CXX_STANDARD 17)

set(ethercat_interface_lib /usr/local/lib/libethercat_interface.so)
set(ethercat_interface_lib_INCLUDE_DIRS /home/grass/ethercat_interface/include/)
#set(ethercat_interface_lib_INCLUDE_DIRS /home/naci/Projects/igh_master/include)
set(etherlab /usr/local/lib/libethercat.so)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)


find_package(catkin REQUIRED COMPONENTS 
    roscpp
    hardware_interface
    geometry_msgs
    nav_msgs
    sensor_msgs
    controller_manager
    class_loader
    diff_drive_controller
    hamal_custom_interfaces
    actionlib
)

find_package(yaml-cpp REQUIRED) 

# find_library(${ethercat_interface_lib} REQUIRED)
# if(NOT ethercat_interface_lib)
#   message(FATAL_ERROR "EtherCAT Interface library not found")
# endif()
# find_library(${etherlab} REQUIRED) 
# if(NOT etherlab)
#   message(FATAL_ERROR "EtherCAT library not found")
# endif()

catkin_package(
    INCLUDE_DIRS
    include
    ${ethercat_interface_lib_INCLUDE_DIRS}
    CATKIN_DEPENDS
    roscpp
    hardware_interface
    geometry_msgs
    nav_msgs
    sensor_msgs
    controller_manager
    diff_drive_controller
    class_loader
    DEPENDS
    ethercat_interface_lib
    etherlab
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ethercat_interface_lib_INCLUDE_DIRS}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

add_executable(hamal_hardware 
    src/hamal_hardware_interface.cpp
    src/ethercat_controller.cpp
)
target_link_libraries(hamal_hardware ${catkin_LIBRARIES} ${ethercat_interface_lib} ${etherlab} ${YAML_CPP_LIBRARIES} Threads::Threads)
target_link_directories(hamal_hardware PUBLIC ${catkin_INCLUDE_DIRS} ${ethercat_interface_lib_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})

add_executable(lifter_hardware 
    src/lifter_hwi.cpp
)
target_link_libraries(lifter_hardware ${catkin_LIBRARIES} ${ethercat_interface_lib} ${etherlab} ${YAML_CPP_LIBRARIES} Threads::Threads)
target_link_directories(lifter_hardware PUBLIC ${catkin_INCLUDE_DIRS} ${ethercat_interface_lib_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})


install(TARGETS hamal_hardware
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS lifter_hardware RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
cmake_minimum_required(VERSION 3.0.2)
project(hamal_utils)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    teb_local_planner
    diff_drive_controller_hamal
    dynamic_reconfigure
    hamal_custom_interfaces
    nav_msgs
    std_msgs
    sensor_msgs
)

find_package(Qt5 COMPONENTS Widgets Core REQUIRED)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES hamal_utils
   CATKIN_DEPENDS diff_drive_controller_hamal roscpp teb_local_planner dynamic_reconfigure hamal_custom_interfaces nav_msgs sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

add_executable(dynamic_speed_limiter src/dynamic_speed_limiter.cpp)
target_link_libraries(dynamic_speed_limiter ${catkin_LIBRARIES})
target_include_directories(dynamic_speed_limiter PUBLIC ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include/)

add_library(emg_stop_client SHARED src/emg_stop_client.cpp)
target_link_libraries(emg_stop_client ${catkin_LIBRARIES})
target_include_directories(emg_stop_client PUBLIC ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(imu_test src/imu_test.cpp)
target_link_libraries(imu_test ${catkin_LIBRARIES})
target_include_directories(imu_test PUBLIC ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include/)

add_executable(emg_stop_gui 
    src/emg_stop_gui.cpp
)
target_link_libraries(emg_stop_gui ${catkin_LIBRARIES} Qt5::Core Qt5::Widgets emg_stop_client)
target_include_directories(emg_stop_gui PUBLIC ${catkin_INCLUDE_DIRS} include/)
install(
    TARGETS dynamic_speed_limiter
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
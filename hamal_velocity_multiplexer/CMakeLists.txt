cmake_minimum_required(VERSION 3.0.2)
project(hamal_velocity_multiplexer)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    hamal_custom_interfaces
)

find_package(yaml-cpp REQUIRED)
set(CMAKE_CXX_STANDARD 17)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hamal_velocity_multiplexer
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
    include
# ${catkin_INCLUDE_DIRS}
)


add_executable(hamal_velocity_multiplexer src/velocity_multiplexer.cpp)
target_link_libraries(hamal_velocity_multiplexer ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
target_include_directories(hamal_velocity_multiplexer PUBLIC ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include/ ${YAML_CPP_INCLUDE_DIRS})

install(TARGETS hamal_velocity_multiplexer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
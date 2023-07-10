cmake_minimum_required(VERSION 3.0.2)
project(hamal_custom_interfaces)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

add_message_files(
    FILES
    LifterCommand.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(

)

include_directories(

)


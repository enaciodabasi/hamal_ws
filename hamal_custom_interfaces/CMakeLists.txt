cmake_minimum_required(VERSION 3.0.2)
project(hamal_custom_interfaces)

find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs actionlib_msgs message_generation)

add_message_files(
    FILES
    LifterCommand.msg
    HardwareInfo.msg
    HardwareInfoArray.msg
    HomingInfo.msg
    ManualMoveCommand.msg
    BmsStatus.msg
    SpeedLimitRatio.msg
    HardwareStatus.msg
    ImuTestDebugInfo.msg
    ProfileCommand.msg
    MotionProfileCommand.msg
)

add_action_files(
    DIRECTORY
    action
    FILES
    LifterOperation.action
    HomingOperation.action
    MotionProfileOperation.action
)

add_service_files(
    DIRECTORY
    srv
    FILES
    EmergencyStop.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    actionlib_msgs
    geometry_msgs    
)

catkin_package(

)

include_directories(

)


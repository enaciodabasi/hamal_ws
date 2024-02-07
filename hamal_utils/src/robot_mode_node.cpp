/**
 * @file robot_mode_node.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-02-07
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <ros/ros.h>
#include <hamal_custom_interfaces/ChangeRobotMode.h>
#include <hamal_custom_interfaces/GetRobotMode.h>

using namespace hamal_custom_interfaces;

enum class RobotMode : unsigned int
{
    Manual = 0,
    Autonomous = 1,
    Hybrid = 2
};

static RobotMode currentMode{0};

bool changeRobotModeCb(ChangeRobotMode::Request& req, ChangeRobotMode::Response& rep)
{
    RobotMode newMode{req.mode};
    currentMode = newMode;
    rep.change_successful = true;
    return true;
}

bool getRobotModeCb(GetRobotMode::Request& req, GetRobotMode::Response& rep)
{
    rep.mode = static_cast<std::underlying_type<RobotMode>::type>(currentMode);
    return true;
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "robot_mode_services");
    ros::NodeHandle nh;



    ros::ServiceServer changeRobotModeService = nh.advertiseService(
        "/change_robot_mode",
        changeRobotModeCb
    );

    ros::ServiceServer getRobotModeService = nh.advertiseService(
        "/get_robot_mode",
        getRobotModeCb
    );

    /* auto changeRobotModeService = nh.advertiseService<hamal_custom_interfaces::ChangeRobotMode>(
        "/change_robot_mode",
        [](auto& req, auto& rep)
        {
            RobotMode newMode{req.mode};
        currentMode = newMode;
        rep.change_successful = true;
        return true;
        }
    );

    auto getRobotModeService = nh.advertiseService<hamal_custom_interfaces::GetRobotMode>(
        "/get_robot_mode",
        [](auto& req, auto& rep)
        {
            rep.mode = static_cast<std::underlying_type<RobotMode>::type>(currentMode);
            
            return true;
        }
    ); */

    ros::Rate rate(10.0);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
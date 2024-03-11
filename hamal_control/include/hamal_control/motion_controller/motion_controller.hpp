/**
 * @file motion_controller.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-02-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef MOTION_CONTROLLER_HPP_
#define MOTION_CONTROLLER_HPP_

#include "motion_profile_generators/motion_profile_controller.hpp"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <hamal_custom_interfaces/MotionProfileCommand.h>
#include <hamal_custom_interfaces/MotionProfileOperationAction.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>

#include <thread>
#include <queue>
#include <future>
#include <functional>
#include <chrono>

double quaternionToDegree(const geometry_msgs::Quaternion& orientation_quaternion);

struct VelParams
{
    double max_vel_x;
    double min_vel_x;

    double max_acc_x;
    double min_acc_x;
    
    double max_jerk_x;
    double min_jerk_x;
    
    double max_vel_z;
    double min_vel_z;
    
    double max_acc_z;
    double min_acc_z;
    
    double max_jerk_z;
    double min_jerk_z;

};

#endif // MOTION_CONTROLLER_HPP_
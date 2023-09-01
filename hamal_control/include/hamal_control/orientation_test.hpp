/**
 * @file orientation_test.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef ORIENTATION_TEST_HPP_
#define ORIENTATION_TEST_HPP_

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

inline const double degToRad(const double& reg)
{
    return reg * (M_PI / 180.0);
}

inline const double radToDeg(const double& rad)
{
    return rad * (180.0 / M_PI);
}

void getInputDegree(const std_msgs::Float64::ConstPtr& degree_command);

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

#endif 

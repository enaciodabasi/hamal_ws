/**
 * @file dynamic_speed_limiter.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef DYNAMIC_SPEED_LIMITER_HPP_
#define DYNAMIC_SPEED_LIMITER_HPP_

#include <memory>
#include <functional>

#include <ros/ros.h>
#include <dynamic_reconfigure/client.h>

// Dynamic Reconfigurable Params Includes:
#include <diff_drive_controller_hamal/DiffDriveControllerHamalConfig.h>
#include <teb_local_planner/TebLocalPlannerReconfigureConfig.h>
#include <hamal_custom_interfaces/SpeedLimitRatio.h>

using SpeedLimitRatio = hamal_custom_interfaces::SpeedLimitRatio;

void speedLimitRatioCallback(const SpeedLimitRatio::ConstPtr& speed_limit_ratio);

void syncTebParams(ros::NodeHandle& nh, teb_local_planner::TebLocalPlannerReconfigureConfig& teb_config);

void syncControllerParams(ros::NodeHandle& nh, diff_drive_controller_hamal::DiffDriveControllerHamalConfig& controller_config);

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

#endif // DYNAMIC_SPEED_LIMITER_HPP_

/**
 * @file trapezoidal_profile.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef TRAPEZOIDAL_PROFILE_HPP_
#define TRAPEZOIDAL_PROFILE_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <hamal_custom_interfaces/ProfileCommand.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/TwistStamped.h>

#include <iostream>
#include <array>
#include <queue>
#include <optional>

class PID
{
    public:

    PID()
    {

    }

    ~PID()
    {

    }

    double control(double current_value, double control_value, const ros::Time& current_time)
    {
        const double dt = (current_time - m_PreviousTime).toSec();
        const double error = control_value - current_value;
        std::cout << "Error: " << error << std::endl;

        m_SumOfErrors += error * dt;

        const double rateOfError = (m_PreviousError - error) / dt;
        m_PreviousError = error;
        m_PreviousTime = current_time;
        auto output = ((m_Kp * error) + (m_Ki * m_SumOfErrors) + (m_Kd * rateOfError));
        return m_Kp*error; 

    }

    /* double errorFunction(const double actual_value, const double control_value)
    {

    }
 */
    void setParams(double kp, double ki, double kd)
    {
        m_Kp = kp;
        m_Ki = ki;
        m_Kd = kd;
    }

    void reset()
    {
        m_Error = 0.0;
        m_PreviousError = 0.0;
        m_SumOfErrors = 0.0;
    }

    private:

    double m_Kp;
    double m_Ki;
    double m_Kd;

    double m_Error;
    double m_PreviousError;
    double m_SumOfErrors;

    ros::Time m_PreviousTime;

};

enum class MotionType
{
    Linear,
    Angular
};

struct
{
    double max_vel;
    double max_acc;
    double target_position;
    double ta, tc, td;
    double pos_ref;
    double vel_ref;
    double angular_vel_ref;
    MotionType motion_type;
    void reset(){
        target_position = 0.0;
        ta, tc, td = 0.0;
        pos_ref, vel_ref = 0.0;
        angular_vel_ref = 0.0;

    }
}goalInfo;

double quaternionToDegree(const geometry_msgs::Quaternion& orientation_quaternion);

#endif // TRAPEZOIDAL_PROFILE_HPP_
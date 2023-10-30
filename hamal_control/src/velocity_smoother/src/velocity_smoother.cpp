/**
 * @file velocity_smoother.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "velocity_smoother/velocity_smoother.hpp"

namespace vel_smoother
{
    Limits::Limits()
        : max(0.0), min(0.0)
    {
        
    }

    VelocitySmoother::VelocitySmoother()
    {
        m_CommandTimeout = ros::Duration(0.5);
        m_LastCommandReceiveTime = ros::Time(0.0);
    }

    VelocitySmoother::~VelocitySmoother()
    {

    }

    void VelocitySmoother::smooth(
        geometry_msgs::Twist& twist_cmd,
        const geometry_msgs::Twist& curr_twist,
        const double& dt
    )
    {

        if((ros::Time::now() - m_LastCommandReceiveTime) > m_CommandTimeout){
            twist_cmd = geometry_msgs::Twist();
        }

        applyAbsoluteVelLimits(twist_cmd);

        const double scaleFactor = calculateScaleFactor(twist_cmd, curr_twist, dt);

        twist_cmd.linear.x = curr_twist.linear.x + limitVel(
            (twist_cmd.linear.x - curr_twist.linear.x), 
            VelocitySmoother::findMinMaxVelDiff(
                twist_cmd.linear.x, curr_twist.linear.x,
                dt,
                m_AccLimitsX.max,
                m_AccLimitsX.min
            ),
            scaleFactor
        );

        twist_cmd.angular.z = curr_twist.angular.z + limitVel(
            (twist_cmd.angular.z - curr_twist.angular.z),
            VelocitySmoother::findMinMaxVelDiff(
                twist_cmd.angular.z, curr_twist.angular.z,
                dt,
                m_AccLimitsZ.max,
                m_AccLimitsZ.min
            ),
            scaleFactor
        );

        m_LastCommandReceiveTime = ros::Time::now();
        
    }

    void VelocitySmoother::applyAbsoluteVelLimits(geometry_msgs::Twist& twist)
    {
        twist.linear.x = std::clamp(twist.linear.x, m_AbsoluteVelLimitsX.min, m_AbsoluteVelLimitsX.max);
        twist.angular.z = std::clamp(twist.angular.z, m_AbsoluteVelLimitsZ.min, m_AbsoluteVelLimitsZ.max);
    }

    double VelocitySmoother::calculateScaleFactor(
            const geometry_msgs::Twist& twist,
            const geometry_msgs::Twist& curr_twist,
            const double& dt
    )
    {
        auto findScaleFactor = [&dt, this](
            const double& v_cmd, 
            const double& v_curr,
            const double& acc,
            const double& decc) -> double {

                //double dv_max, dv_min = 0.0;

                auto [dv_min, dv_max] = VelocitySmoother::findMinMaxVelDiff(v_cmd, v_curr, dt, acc, decc);

                const double dV = v_cmd - v_curr;
                if(dV > dv_max){
                    return dv_max / dV;
                }
                else if(dV < dv_min){
                    return dv_min / dV;
                }

                return -1.0;

        };

        double scaleFactor = 1.0;
        double currScaleFactor = -1.0;

        currScaleFactor = findScaleFactor(
            twist.linear.x,
            curr_twist.linear.x,
            m_AccLimitsX.max,
            m_AccLimitsX.min
        );
        if(currScaleFactor > 0.0 && (std::fabs(1.0 - currScaleFactor) > std::fabs(1.0 - scaleFactor))){
            scaleFactor = currScaleFactor;
        }


        currScaleFactor = findScaleFactor(
            twist.angular.z,
            curr_twist.angular.z,
            m_AccLimitsZ.max,
            m_AccLimitsZ.min
        );
        if(currScaleFactor > 0.0 && (std::fabs(1.0 - currScaleFactor) > std::fabs(1.0 - scaleFactor))){
            scaleFactor = currScaleFactor;
        }


        return scaleFactor;
    }
}
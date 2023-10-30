/**
 * @file velocity_smoother.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef VELOCITY_SMOOTHER_HPP_
#define VELOCITY_SMOOTHER_HPP_

#include <algorithm>
#include <geometry_msgs/Twist.h>

//template<typename T>
//struct Commands
//{
//    Commands();
//
//    T velocity; 
//
//};

namespace vel_smoother
{
 
    struct Limits
    {
        double max;
        double min;

        Limits();
        ~Limits(){}
    };


    class VelocitySmoother
    {

        private:

        /**
         * @brief first: min | second: max
         * 
         */
        using MinMaxVelDiff = std::pair<double, double>;

        public:

        VelocitySmoother();

        ~VelocitySmoother();

        void setLimitsX(const Limits& vel_limits, const Limits& acc_limits)
        {
            m_AbsoluteVelLimitsX = vel_limits;
            m_AccLimitsX = acc_limits;
        }

        void setLimitsZ(const Limits& vel_limits, const Limits& acc_limits)
        {
            m_AbsoluteVelLimitsZ = vel_limits;
            m_AccLimitsZ = acc_limits;
        }

        void smooth(
            geometry_msgs::Twist& twist_cmd,
            const geometry_msgs::Twist& curr_twist,
            const double& dt
        );

        private:

        Limits m_AbsoluteVelLimitsX;
        Limits m_AbsoluteVelLimitsZ;
        
        Limits m_AccLimitsX;
        Limits m_AccLimitsZ;

        double m_DeadbandX = 0.0;

        double m_DeadbandZ = 0.0;

        geometry_msgs::Twist m_LastTwist;

        void applyAbsoluteVelLimits(geometry_msgs::Twist& twist);

        double calculateScaleFactor(
            const geometry_msgs::Twist& twist,
            const geometry_msgs::Twist& curr_twist,
            const double& dt
        );
        
        inline static const MinMaxVelDiff findMinMaxVelDiff(const double& v_cmd, const double& v_curr, const double& dt, const double& acc, const double& decc)
        {
            const double dV = v_cmd - v_curr;

            double dv_max, dv_min = 0.0;
            if(std::fabs(v_cmd) >= std::fabs(v_curr) && v_cmd * v_curr >= 0.0){
                dv_max = acc / dt;
                dv_min = -acc / dt; 
            }
            else{
                dv_max = -decc / dt;
                dv_min = decc / dt;
            }

            return std::make_pair(dv_min, dv_max);
        }

        inline double limitVel(const double& dv, const MinMaxVelDiff& vel_diffs, const double& eta = 1.0)
        {
            return std::clamp(eta*dv, vel_diffs.first, vel_diffs.second);
        }

    }; // VelocitySmoother

} // end of namespace vel_smoother

#endif // VELOCITY_SMOOTHER_HPP_
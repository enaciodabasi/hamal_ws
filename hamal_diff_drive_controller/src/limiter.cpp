/**
 * @file limiter.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hamal_diff_drive_controller/limiter.hpp"

namespace hamal_diff_drive_controller 
{
    
    Limiter::Limiter(
        bool limit_vel,
        bool limit_accel,
        bool limit_jerk
    )   : m_HasVelLimit(limit_vel), m_HasAccelLimit(limit_accel), m_HasJerkLimit(limit_jerk)
    {

    }

    double Limiter::limit(double& v, double v0, double v1, double dt)
    {
        const double temp = v;

        limit_jerk(v, v0, v1, dt);
        limit_accel(v, v0, dt);
        limit_vel(v);

        if(temp == 0.0)
        {
            return 1.0;
        }

        return v / temp;
    }

    double Limiter::limit_vel(double& v)
    {
        const double temp = v;

        if(m_HasVelLimit)
        {
            v = clamp(v, m_VelLimit.min, m_VelLimit.max);
        }

        if(temp == 0.0)
        {
            return 1.0;
        }

        return v  / temp;
    }

    double Limiter::limit_accel(double& v, double v0, double dt)
    {
        const double temp = v;

        if(m_HasAccelLimit)
        {
            const double dv_min = m_AccelLimit.min * dt;
            const double dv_max = m_AccelLimit.max * dt;
            // limit the acceleration:
            const double dv = clamp(
                v - v0,
                dv_min,
                dv_max
            );

            v = v0 + dv;
        }

        if(temp == 0.0)
        {
            return 1.0;
        }

        return v / temp;
    }

    double Limiter::limit_jerk(double& v, double v0, double v1, double dt)
    {
        const double temp = v;

        if(m_HasJerkLimit)
        {
            const double dv = v - v0;
            const double dv0 = v0 - v1;

            const double dt2 = 2.0 * dt * dt;

            const double da_min = m_JerkLimit.min * dt2;
            const double da_max = m_JerkLimit.max * dt2;

            const double da = clamp(dv - dv0, da_min, da_max);

            v = v0 + dv0 + da; 
        } 

        if(temp == 0.0)
        {
            return 1.0;
        }

        return v / temp;
    }
    

} // hamal_diff_drive_controller

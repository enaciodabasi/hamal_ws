/**
 * @file limiter.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hamal_diff_drive_controller/defs.hpp"

#include <algorithm>

namespace hamal_diff_drive_controller
{
    struct Limiter
    {
        public:

        Limiter(
            bool limit_vel,
            bool limit_accel,
            bool limit_jerk
        );

        double limit(
            double &v,
            double v0,
            double v1,
            double dt
        );

        inline void setVelLimits(double min, double max)
        {
            m_VelLimit.min = min;
            m_VelLimit.max = max;
        }

        inline void setVelLimits(const Limit& vel_limit)
        {
            m_VelLimit = vel_limit;
        }

        inline void setAccelLimits(double min, double max)
        {
            m_AccelLimit.min = min;
            m_AccelLimit.max = max;
        }

        inline void setAccelLimits(const Limit& accel_limit)
        {
            m_AccelLimit = accel_limit;
        }

        inline void setJerkLimits(double min, double max)
        {
            m_JerkLimit.min = min;
            m_JerkLimit.max = max;
        }

        inline void setJerkLimits(const Limit& jerk_limit)
        {
            m_JerkLimit = jerk_limit;
        }

        private:

        bool m_HasVelLimit = true;

        bool m_HasAccelLimit = true;

        bool m_HasJerkLimit = false;

        Limit m_VelLimit;

        Limit m_AccelLimit;

        Limit m_JerkLimit;

        double limit_vel(double& v);

        double limit_accel(double& v, double v0, double dt);

        double limit_jerk(double& v, double v0, double v1, double dt);

    }; 
}
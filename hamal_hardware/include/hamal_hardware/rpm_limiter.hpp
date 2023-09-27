/**
 * @file rpm_limiter.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-09-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef RPM_LIMITER_HPP_
#define RPM_LIMITER_HPP_

#include <algorithm>

struct rpm_limiter
{
    rpm_limiter() : maxVel(0.0), minVel(0.0), maxAcc(0.0), minAcc(0.0), previousVel(0.0) {}
    rpm_limiter(
        const double& max_vel,
        const double& min_vel,
        const double& max_acc,
        const double& min_acc
    ) : maxVel(max_vel), minVel(min_vel), maxAcc(max_acc), minAcc(min_acc), previousVel(0.0) {}

    void limit(double& rpm, const double& dt)
    {

        double tempRPM = rpm;

        // Limit Acceleration:
        const double minVelDiff = minAcc * dt;
        const double maxVelDiff = maxAcc * dt;


        const double idealAcc = std::clamp(
            tempRPM - previousVel, minVelDiff, maxVelDiff
        );

        tempRPM = previousVel + idealAcc;

        // Limit Velocity:

        tempRPM = std::clamp(tempRPM, minVel, maxVel);

        rpm = tempRPM;

    }

    double maxVel;
    double minVel;
    double maxAcc;
    double minAcc;

    double previousVel;

};


#endif // RPM_LIMITER_HPP_

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
    rpm_limiter();
    rpm_limiter(
        const double& max_vel,
        const double& min_vel,
        const double& max_acc,
        const double& min_acc
    );

    void limit(double& rpm, const double& dt);

    double maxVel;
    double minVel;
    double maxAcc;
    double minAcc;

    double previousVel;

};


#endif // RPM_LIMITER_HPP_

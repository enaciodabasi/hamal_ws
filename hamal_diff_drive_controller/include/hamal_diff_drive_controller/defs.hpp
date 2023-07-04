/**
 * @file defs.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef HAMAL_CONTROL_DEFS_HPP_
#define HAMAL_CONTROL_DEFS_HPP_

struct WheelParams
{
    double wheel_radius;
    double wheel_separation;
};

struct Pose
{   
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
};

struct Velocity
{
    double linear = 0.0;
    double angular = 0.0;
};

struct Limit
{
    double min = 0.0;
    double max = 0.0;
};

#endif // HAMAL_CONTROL_DEFS_HPP_
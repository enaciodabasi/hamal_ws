/**
 * @file odometry.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hamal_diff_drive_controller/odometry.hpp"

namespace hamal_diff_drive_controller
{
    Odometry::Odometry(const WheelParams& wheel_params, std::size_t velocity_rolling_window_size)
        : 
        m_WheelParams{wheel_params}, 
        m_RollingWindowSize(velocity_rolling_window_size),
        m_PreviousUpdateTime(0.0),
        m_LinearVelAccumulator(bacc::tag::rolling_window::window_size = m_RollingWindowSize),
        m_AngularVelAccumulator(bacc::tag::rolling_window::window_size = m_RollingWindowSize)
    {
        /* m_LinearVelAccumulator = boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::rolling_mean>>(
            boost::accumulators::tag::rolling_window::window_size = m_RollingWindowSize
        );
        m_AngularVelAccumulator = boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::rolling_mean>>(
            boost::accumulators::tag::rolling_window::window_size = m_RollingWindowSize
        ); */
    }

    void Odometry::init(const ros::Time& time)
    {
        m_PreviousUpdateTime = time;
    }

    const Pose Odometry::integrate(const double linear_vel, const double angular_vel)
    {
        if(std::fabs(angular_vel) < 1e-6)
        {
            return rungeKutta(linear_vel, angular_vel);
        }

        return exactIntegration(linear_vel, angular_vel);
    }

    const Pose Odometry::rungeKutta(const double linear_vel, const double angular_vel)
    {
        double heading;

        heading = m_Pose.heading;
        const double dir = heading + (angular_vel * 0.5);
        const double x = linear_vel * std::cos(dir);
        const double y = linear_vel * std::sin(dir);

        return {x, y, angular_vel};

    }

    const Pose Odometry::exactIntegration(const double linear_vel, const double angular_vel)
    {      
        double oldHeading;

        oldHeading = m_Pose.heading;

        const double dir = oldHeading + angular_vel;

        const double ratio = linear_vel / angular_vel;

        const double x = ratio * (std::sin(dir) - std::sin(oldHeading));
        const double y = ratio * (std::cos(dir) - std::cos(oldHeading));

        return {x, y, angular_vel};
    }

    VelocityOdometry::VelocityOdometry(const WheelParams& wheel_params, std::size_t velocity_rolling_window_size)
        : Odometry(wheel_params, velocity_rolling_window_size)
    {

    }

    bool VelocityOdometry::update(double left, double right, const ros::Time& time)
    {
        const double dt = time.toSec() - m_PreviousUpdateTime.toSec();

        const double linear_vel = (left + right) / 2.0;
        const double angular_vel = (right - left) / m_WheelParams.wheel_separation;

        const Pose newPose = integrate(linear_vel, angular_vel);

        m_Pose.x += newPose.x;
        m_Pose.y += newPose.heading;
        m_Pose.heading += newPose.heading;

        m_PreviousUpdateTime = time;

        if(dt < 1e-3)
        {
            return false;
        }

        m_LinearVelAccumulator(linear_vel / dt);
        m_AngularVelAccumulator(angular_vel / dt);

        m_EstimatedVelocity.linear = boost::accumulators::rolling_mean(m_LinearVelAccumulator);
        m_EstimatedVelocity.angular = boost::accumulators::rolling_mean(m_AngularVelAccumulator);

        return true;
    }

    PositionOdometry::PositionOdometry(const WheelParams& wheel_params, std::size_t velocity_rolling_window_size)
        : Odometry(wheel_params, velocity_rolling_window_size)
    {

    }

    bool PositionOdometry::update(double left, double right, const ros::Time& time)
    {
        const double left_wheel_curr_pos = left * m_WheelParams.wheel_radius;
        const double right_wheel_curr_pos = right * m_WheelParams.wheel_radius;

        const double left_wheel_vel = left_wheel_curr_pos - m_LeftWheelPreviousPos;
        const double right_wheel_vel = right_wheel_curr_pos  - m_RightWheelPreviousPos;
    
        m_LeftWheelPreviousPos = left_wheel_curr_pos;
        m_RightWheelPreviousPos = right_wheel_curr_pos;

        const double linear_vel = (left_wheel_vel + right_wheel_vel) / 2.0;
        const double angular_vel = (right_wheel_vel - left_wheel_vel) / m_WheelParams.wheel_separation;

        const Pose newPose = integrate(linear_vel, angular_vel);

        m_Pose.x += newPose.x;
        m_Pose.y += newPose.y;
        m_Pose.heading += newPose.heading;

        const double dt = time.toSec() - m_PreviousUpdateTime.toSec();
        if(dt < 1e-3)
        {
            return false;
        }

        m_PreviousUpdateTime = time;

        m_LinearVelAccumulator(linear_vel / dt);
        m_AngularVelAccumulator(angular_vel / dt);

        m_EstimatedVelocity.linear = boost::accumulators::rolling_mean(m_LinearVelAccumulator);
        m_EstimatedVelocity.angular = boost::accumulators::rolling_mean(m_AngularVelAccumulator);

        return true;
    }


}
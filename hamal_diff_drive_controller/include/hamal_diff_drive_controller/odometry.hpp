/**
 * @file odometry.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef ODOMETRY_HPP_
#define ODOMETRY_HPP_

#include <ros/time.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

#include "hamal_diff_drive_controller/defs.hpp"

namespace hamal_diff_drive_controller
{

    namespace bacc = boost::accumulators;

    class Odometry
    {
        public:

        Odometry(const WheelParams& wheel_params, std::size_t velocity_rolling_window_size);

        ~Odometry();

        void init(const ros::Time& time);

        virtual bool update(double left, double right, const ros::Time& time) = 0;

        inline const Pose getPose() const
        {
            return m_Pose;
        }

        inline const Velocity getVelocity() const
        {
            return m_EstimatedVelocity;
        }


        protected:
        
        ros::Time m_PreviousUpdateTime;

        Pose m_Pose;

        Velocity m_EstimatedVelocity;

        WheelParams m_WheelParams;

        std::size_t m_RollingWindowSize;

        bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean>> m_LinearVelAccumulator;
        bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean>> m_AngularVelAccumulator;

        const Pose integrate(const double linear_vel, const double angular_vel);

        const Pose rungeKutta(const double linear_vel, const double angular_vel);

        const Pose exactIntegration(const double linear_vel, const double angular_vel);
        

    };

    class VelocityOdometry : public Odometry
    {

        public:

        VelocityOdometry(const WheelParams& wheel_params, std::size_t velocity_rolling_window_size);

        bool update(double left, double right, const ros::Time& time) override;

        private:



    };

    class PositionOdometry : public Odometry
    {
        public:

        PositionOdometry(const WheelParams& wheel_params, std::size_t velocity_rolling_window_size);

        bool update(double left, double right, const ros::Time& time) override;

        private:

        double m_LeftWheelPreviousPos = 0.0;
        
        double m_RightWheelPreviousPos = 0.0;

    };

    /* class Odometry
    {
        public:

        Odometry(const WheelParams& wheel_params, std::size_t velocity_rolling_window_size = 10);

        void init(const ros::Time& time);

        bool updateFromPosition(
            double left_wheel_pos,
            double right_wheel_pos,
            const ros::Time& time
        );

        bool updateFromVelocity(
            double left_wheel_vel,
            double right_wheel_vel,
            const ros::Time& time
        );

        const Pose getPoseFromVelocity() const
        {
            return m_PoseFromVelocity;
        }

        const Pose getPoseFromPosition() const
        {
            return m_PoseFromPosition;
        }

        private:

        ros::Time m_PreviousUpdateTimePos;
        ros::Time m_PreviousUpdateTimeVel;

        Pose m_PoseFromPosition;

        Pose m_PoseFromVelocity;

        struct
        {
            double linear = 0.0;
            double angular = 0.0;
        } m_EstVelocityFromVel;

        struct
        {
            double linear = 0.0;
            double angular = 0.0;
        } m_EstVelocityFromPos;

        WheelParams m_WheelParams;

        double m_LeftWheelPreviousPos = 0.0;
        double m_RightWheelPreviousPos = 0.0;

        std::size_t m_VelocityRollingWindowSize;

        boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::rolling_mean>> m_LinearVelocityAccumulatorPos;
        boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::rolling_mean>> m_AngularVelocityAccumulatorPos;

        boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::rolling_mean>> m_LinearVelocityAccumulatorVel;
        boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::rolling_mean>> m_AngularVelocityAccumulatorVel;

        Pose integrate(double linear_vel, double angular_vel, bool position = true);

        Pose rungeKutta(double linear_vel, double angular_vel, bool position = true);

        Pose exactIntegration(double linear_vel, double angular_vel, bool position = true);
        

    }; */
}

#endif // ODOMETRY_HPP_
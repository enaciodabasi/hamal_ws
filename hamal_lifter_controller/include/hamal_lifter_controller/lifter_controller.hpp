/**
 * @file lifter_controller.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-07-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef HAMAL_LIFTER_CONTROLLER_HPP_
#define HAMAL_LIFTER_CONTROLLER_HPP_

#include <controller_interface/controller.h>
#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <actionlib/server/simple_action_server.h>
#include <pluginlib/class_list_macros.h>

#include "hamal_custom_interfaces/LifterOperationAction.h"

#include <boost/bind/bind.hpp>

#include <iostream>
#include <memory>
#include <optional>

using LifterAction = hamal_custom_interfaces::LifterOperationAction;
using LifterResult = hamal_custom_interfaces::LifterOperationResult;
using LifterFeedback = hamal_custom_interfaces::LifterOperationFeedback;

struct FifthOrderCoeffs
{
    double a0;
    double a1;
    double a2;
    double a3;
    double a4;
    double a5;

    FifthOrderCoeffs()
    {
        a1 = 0;
        a2 = 0;
        a3 = 0;
        a4 = 0;
        a5 = 0;
        a0 = 0;
    }
};

struct Commands
{
    double position;
    double velocity;
    double accel;

    Commands()
    {
        position = 0.0;
        velocity = 0.0;
        accel = 0.0;
    }
};

namespace hamal_lifter_controller
{
    class HamalLifterController : public controller_interface::Controller<hardware_interface::PosVelAccJointInterface>
    {
        public:

        HamalLifterController();

        ~HamalLifterController();

        bool init(
            hardware_interface::PosVelAccJointInterface* pos_vel_acc_interface, 
            ros::NodeHandle& base_nh, 
            ros::NodeHandle& controller_nh
        );

        void update(const ros::Time& time, const ros::Duration& period);

        void starting(const ros::Time &time);

        void stopping(const ros::Time& time);

        void setTargetPosition(double target_position)
        {
            m_TargetPosition = target_position;
        }

        private:

        std::string m_ControllerName;

        std::string m_LifterJointName;

        double m_ControllerRate;
        ros::Duration m_ControllerPeriod;

        hardware_interface::PosVelAccJointHandle m_LifterJointHandle;

        double m_TargetPosition = 0.0;

        double m_TargetVelocity = 0.0;

        double m_TargetAccel = 0.0;

        double m_TargetTime = 0.0;

        ros::Time m_StartTime;
        ros::Time m_PreviousTime;

        double m_MaxPos = 0.0;
        double m_MaxVel = 0.0;
        double m_MaxAccel = 0.0;

        std::shared_ptr<actionlib::SimpleActionServer<LifterAction>> m_LifterActionServer;

        bool m_IsGoalNew = true;

        Commands computeCommands(
            const double& current_time
        );

        const double calculateRequiredOperationDuration();

        void checkLimits(Commands& commands);

        void lifterActionGoalCallback();

    };


    const FifthOrderCoeffs fifthOrderPolyCoeffs(
            const double& start_pos,
            const double& final_pos,
            const double& start_vel,
            const double& final_vel,
            const double& start_acc,
            const double& final_acc,
            const double& target_time,
            const double& current_time
    );
}
#endif // LIFTER_CONTROLLER_HPP_
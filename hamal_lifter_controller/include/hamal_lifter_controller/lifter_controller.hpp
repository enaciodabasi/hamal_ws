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
#include <std_msgs/Float64.h>

#include "hamal_lifter_controller/controllers.hpp"
#include "hamal_custom_interfaces/LifterOperationAction.h"

#include <boost/bind/bind.hpp>

#include <iostream>
#include <memory>
#include <optional>

using LifterOperationActionServer = actionlib::SimpleActionServer<hamal_custom_interfaces::LifterOperationAction>;

using LifterAction = hamal_custom_interfaces::LifterOperationAction;
using LifterResult = hamal_custom_interfaces::LifterOperationResult;
using LifterFeedback = hamal_custom_interfaces::LifterOperationFeedback;

constexpr auto ProfileTimeErrorStr = "Elapsed time is longer than the desired profile time. Aborting...";
constexpr auto OperationSuccessStr = "Target successfuly reached.";
template<typename T>
bool inRange(const T& check_val, const T& target_val, const T& range)
{
    return (check_val < (target_val + range)) && (check_val > (target_val - range));
}

namespace hamal_lifter_controller
{

    struct PID
    {

        double Kp;
        double Ki;
        double Kd;

        double prevError;
        double sumOfErrors;

        ros::Time m_UpdateTime;

        void setParams(
            const double kp,
            const double ki,
            const double kd
        )
        {
            Kp = kp;
            Ki = ki;
            Kd = kd;
        }

        template<typename T>
        T pid(const ros::Time& current_time, T actual, T target);

        void init(
            ros::Time init_time
        );

        void reset()
        {
            prevError = 0.0;
            sumOfErrors = 0.0;
            m_UpdateTime = ros::Time(0.0);
        }

        PID()
        {
            Kp = 0;
            Ki = 0;
            Kd = 0;
            m_UpdateTime = ros::Time(0.0);
        }
    }; 

    class HamalLifterController : public controller_interface::Controller<hardware_interface::PosVelAccJointInterface>
    {
        private:

        struct Coefficients{
            double a0;
            double a1;
            double a2;
            double a3;
            double a4;
            double a5;

            Coefficients()
            {
                a0 = 0.0;
                a1 = 0.0;
                a2 = 0.0;
                a3 = 0.0;
                a4 = 0.0;
                a5 = 0.0;
            }

            void reset()
            {
                a0 = 0.0;
                a1 = 0.0;
                a2 = 0.0;
                a3 = 0.0;
                a4 = 0.0;
                a5 = 0.0;
            }
        };

        public:

        HamalLifterController();

        ~HamalLifterController();

        bool init(
            hardware_interface::PosVelAccJointInterface* lifter_control_interface,
            ros::NodeHandle& base_nh,
            ros::NodeHandle& controller_nh
        );

        void starting(const ros::Time& time);

        void update(const ros::Time& time, const ros::Duration& period);

        void stopping(const ros::Time& time);


        void setLimits(double pos_limit, double vel_limit, double accel_limit)
        {
            m_Limits.posLimit = pos_limit;
            m_Limits.velLimit = vel_limit;
            m_Limits.accelLimit = accel_limit;
        }

        void updateUpdateTime(){
            m_PreviousUpdateTime = ros::Time::now();
        }

        private:

        std::unique_ptr<LifterOperationActionServer> m_LifterOperationServer;

        double m_PrevPosTrajCmd = 0.0; 

    /*     std::optional<Commands> getCommands(double current_pos, double current_vel);    
     */

        std::string m_LifterJointName;

        hardware_interface::PosVelAccJointHandle m_LifterJointHandle;

        bool m_NewGoalArrived = false;
        bool m_GoalActive = false;
        bool m_GoalDone = false;
        bool m_GoalError = false;
        Coefficients m_GoalCoefficients;    
        hamal_custom_interfaces::LifterOperationGoal* m_CurrentActionGoalPtr;

        PID m_PositionPID;

        double positionTracker = 0.0;

        struct{ 
            double posLimit = 0.0;
            double velLimit = 0.0;
            double accelLimit = 0.0;
        } m_Limits;

        double m_PositionTolerance;

        double m_ControllerFrequency;
        std::unique_ptr<ros::Rate> m_ControllerRate;

        ros::Time m_PreviousUpdateTime;

        ros::Duration m_MaxProfileTime;

        const Coefficients calculateControlParams(
            const double& initial_position,
            const double& initial_velocity,
            const double& initial_acceleration,
            const double& target_position,
            const double& target_vel = 0.0,
            const double& target_acc = 0.0
        );

        const Coefficients calculateControlParams(
            double profile_time,
            const double& initial_position,
            const double& initial_velocity,
            const double& initial_acceleration,
            const double& target_position,
            const double& target_vel = 0.0,
            const double& target_acc = 0.0
        );

        void cleanup();

        void lifterActionCallback();

        double linearDistanceToLifterRotation(const double& lifter_rotation)
        {

        }

        double lifterRotationToLinearDistance(const double& linear_distance)
        {

        }


    };

    }

#endif // LIFTER_CONTROLLER_HPP_
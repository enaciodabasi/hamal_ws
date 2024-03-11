/**
 * @file hamal_lifter_controller.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-02-19
 * 
 * @copyright Copyright (c) 2024
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
#include "hamal_custom_interfaces/LifterOperationAction.h"

#include "motion_profile_generators/motion_profile_controller.hpp"
#include <hamal_custom_interfaces/MotionProfileCommand.h>

#include <queue>
#include <variant>
#include <chrono>

using LifterOperationActionServer = actionlib::SimpleActionServer<hamal_custom_interfaces::LifterOperationAction>;
using LifterAction = hamal_custom_interfaces::LifterOperationAction;
using LifterResult = hamal_custom_interfaces::LifterOperationResult;
using LifterFeedback = hamal_custom_interfaces::LifterOperationFeedback;

/* using Tolerance = std::variant<double, int32_t>; */

using namespace motion_profile_generators;



namespace hamal_lifter_controller
{
    template<typename T>
bool inRange(const T check_val, const T boundry, const T target)
{
    return (
        (
            (abs(check_val) + boundry >= abs(target)) 
            || 
            (abs(check_val) - boundry <= abs(target))
        )
    );
}
    class HamalLifterController : public controller_interface::Controller<hardware_interface::PosVelAccJointInterface>
    {
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

        private:
        
        std::string m_LifterJointName;

        hardware_interface::PosVelAccJointHandle m_LifterJointHandle;
        
        std::unique_ptr<LifterOperationActionServer> m_LifterOperationServer;

        std::queue<hamal_custom_interfaces::MotionProfileCommand> m_CommandQueue;

        MotionProfileController<double, double> m_LifterMotionProfileController;

        std::chrono::time_point<std::chrono::high_resolution_clock> m_ProfileStartTime = {};

        motion_profile_generators::triangular_profile::TriangularProfileTimes<double> m_CurrentTimePoints;

        MotionConstraints<double> m_MotionConstraints; 

        bool isGoalCurrentlyActive = false;

        void lifterActionCallback();

        double m_LifterPositionTolerance;

    };
}
#endif // HAMAL_LIFTER_CONTROLLER_HPP_
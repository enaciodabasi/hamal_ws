/**
 * @file lifter_controller.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-07-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hamal_lifter_controller/lifter_controller.hpp"

namespace hamal_lifter_controller
{
    HamalLifterController::HamalLifterController()
    {

    }

    HamalLifterController::~HamalLifterController()
    {

    }

    bool HamalLifterController::init(
        hardware_interface::PosVelAccJointInterface* pos_vel_acc_interface,
        ros::NodeHandle& base_nh,
        ros::NodeHandle& controller_nh
    )
    {


        const std::string complete_ns = controller_nh.getNamespace();
        std::size_t id = complete_ns.find_last_of("/");
        m_ControllerName = complete_ns.substr(id + 1);

        if(controller_nh.hasParam("lifter_joint_name"))
        {
            controller_nh.getParam("lifter_joint_name", m_LifterJointName);
        }

        controller_nh.param("controller_rate", m_ControllerRate, 250.0);
        m_ControllerPeriod = ros::Duration(1.0 / m_ControllerRate);

        controller_nh.param("max_velocity", m_MaxVel, 0.5);
        controller_nh.param("max_accel", m_MaxAccel, 1.5);

        m_LifterActionServer = std::make_shared<actionlib::SimpleActionServer<LifterAction>>(base_nh, "lifter_action_server", false);
        m_LifterActionServer->registerGoalCallback(boost::bind(&HamalLifterController::lifterActionGoalCallback, this));

        return true;
    }   

    void HamalLifterController::update(const ros::Time& time, const ros::Duration& period)
    {   

        if(m_LifterActionServer->isActive())
        {
            const auto currentTime = time;   
            const auto currentPosition = m_LifterJointHandle.getPosition();
            if(currentPosition == m_TargetPosition)
            {
                LifterResult res;
                res.target_reached = true;
                ROS_INFO("Goal reached.");
                m_LifterActionServer->setSucceeded(res);
                return;    
            }
            // If the goal is new
            if(m_IsGoalNew)
            {
                m_StartTime = currentTime;
                m_TargetTime = calculateRequiredOperationDuration();
                m_IsGoalNew = false;
            }

            const double time = (currentTime - m_StartTime).toSec();

            if(time > m_TargetTime)
            {
                LifterResult res;
                res.target_reached =false;
                ROS_ERROR("Elapsed time is bigger than target profile time. Aborting...");
                m_LifterActionServer->setAborted(res);
                return;
            }

            Commands newCommands;

            newCommands = computeCommands(
                time
            );

            checkLimits(newCommands);

            LifterFeedback feedback;
            feedback.target_command = newCommands.position;
            feedback.current_position = currentPosition;

            m_LifterActionServer->publishFeedback(feedback);

            m_LifterJointHandle.setCommandPosition(newCommands.position);
            m_LifterJointHandle.setCommandVelocity(newCommands.velocity);
            m_LifterJointHandle.setCommandAcceleration(newCommands.accel);

            m_PreviousTime = currentTime;

        }
        else
        {
            m_TargetPosition = 0.0;
            m_TargetTime = 0.0;
            m_IsGoalNew = true;
        }
    }

    void HamalLifterController::starting(const ros::Time& time)
    {
        m_StartTime = ros::Time::now();
        m_PreviousTime = m_StartTime;

        m_LifterActionServer->start();
    }

    void HamalLifterController::stopping(const ros::Time& time)
    {
        m_LifterActionServer->shutdown();
    }

    void HamalLifterController::lifterActionGoalCallback()
    {
        const auto newGoal = m_LifterActionServer->acceptNewGoal();
        m_TargetPosition = newGoal->target_position;
        m_IsGoalNew = true;
    }

    const double HamalLifterController::calculateRequiredOperationDuration()
    {
        const double current_position = m_LifterJointHandle.getPosition();

        double duration = std::fmax(
            (15.0 * std::fabs(m_TargetPosition - current_position)) / (8.0 * m_MaxVel),
            (std::sqrt((10.0 * std::fabs(m_TargetPosition - current_position)) / (m_MaxAccel * std::sqrt(3.0))))
        );

        return duration;
    }

    Commands HamalLifterController::computeCommands(
        const double& current_time
    )
    {
        Commands cmds;

        const double startPos = m_LifterJointHandle.getPosition();
        const double startVel = m_LifterJointHandle.getVelocity();
        const double startAccel = 0.0;

        const auto coeffs = fifthOrderPolyCoeffs(
            startPos,
            m_TargetPosition,
            startVel,
            m_TargetVelocity,
            startAccel,
            m_TargetAccel,
            m_TargetTime,
            current_time
        );

        cmds.position = (
            coeffs.a0   +
            coeffs.a1 * current_time    +
            coeffs.a2 * std::pow(current_time, 2)   +
            coeffs.a3 * std::pow(current_time, 3)   +
            coeffs.a4 * std::pow(current_time, 4)   +
            coeffs.a5 * std::pow(current_time, 5)
        );

        cmds.velocity = (
            coeffs.a1 * current_time    +
            coeffs.a2 * std::pow(current_time, 2)   +
            coeffs.a3 * std::pow(current_time, 3)   +
            coeffs.a4 * std::pow(current_time, 4)   +
            coeffs.a5 * std::pow(current_time, 5)
        );

        cmds.accel = (
            coeffs.a2 * std::pow(current_time, 2)   +
            coeffs.a3 * std::pow(current_time, 3)   +
            coeffs.a4 * std::pow(current_time, 4)   +
            coeffs.a5 * std::pow(current_time, 5)
        );

        return cmds;

    }

    const FifthOrderCoeffs fifthOrderPolyCoeffs(
        const double& start_pos,
        const double& final_pos,
        const double& start_vel,
        const double& final_vel,
        const double& start_acc,
        const double& final_acc,
        const double& target_time,
        const double& current_time
    )
    {

        FifthOrderCoeffs coeffs;
        coeffs.a0 = start_pos;
        coeffs.a1 = start_vel;
        coeffs.a2 = start_acc / 2.0;

        coeffs.a3 = (
            (1.0 / 2.0 * std::pow(target_time, 3)) *
            (
                (20.0 * (final_pos - start_pos)) -
                (((8.0 * start_vel) + (12.0 * final_vel)) * target_time) -
                (((3.0 * start_acc) - (final_acc)) * std::pow(target_time, 2))
            )
        );

        coeffs.a4 = (
            (1.0 / 2.0 * std::pow(target_time, 4)) *
            (
                (30.0 * (final_pos - start_pos)) +
                (((14.0 * start_vel) + (16.0 * final_vel)) * target_time) +
                (((3.0 * start_acc) - (2.0 * final_acc)) * std::pow(target_time, 2))
            )
        );

        coeffs.a5 = (
            (1.0 / 2.0 * std::pow(target_time, 5)) *
            (
                (12.0 * (final_pos - start_pos)) +
                (((6.0 * start_vel) + (6.0 * final_vel)) * target_time) +
                (((1.0 * start_acc) - (1.0 * final_acc)) * std::pow(target_time, 2))
            )
        );

        return coeffs;

    }

    void HamalLifterController::checkLimits(Commands& commands)
    {
        if(std::abs(commands.velocity) > m_MaxVel)
        {
            if(commands.velocity < 0) {commands.velocity = -m_MaxVel;}
            else {commands.velocity = m_MaxVel;}
        }

        if(std::abs(commands.accel) > m_MaxAccel)
        {
            if(commands.accel < 0) {commands.accel = -m_MaxAccel;}
            else {commands.accel = m_MaxAccel;}
        }
    }

}

PLUGINLIB_EXPORT_CLASS(
    hamal_lifter_controller::HamalLifterController, controller_interface::ControllerBase
);
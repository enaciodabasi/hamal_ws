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

#include <math.h>
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

        m_LifterJointHandle = pos_vel_acc_interface->getHandle(m_LifterJointName);

        controller_nh.param("controller_rate", m_ControllerRate, 250.0);
        m_ControllerPeriod = ros::Duration(1.0 / m_ControllerRate);

        controller_nh.param("max_velocity", m_MaxVel, 0.5);
        controller_nh.param("max_accel", m_MaxAccel, 1.5);

        controller_nh.param("tolerance", m_Tolerance, 0.05);

        double kp, ki, kd = 0.0;
        controller_nh.param("pid/kp", kp, kp);
        controller_nh.param("pid/ki", ki, ki);
        controller_nh.param("pid/ki", kd, kd);

        double Kffv = 0.0;
        controller_nh.param("feed_forward/Kffv", Kffv, Kffv);
        m_KffV = Kffv;

        m_PidParams.Kp = kp;
        m_PidParams.Ki = ki;
        m_PidParams.Kd = kd;

        m_LifterActionServer = std::make_shared<actionlib::SimpleActionServer<LifterAction>>(base_nh, "lifter_action_server", false);
        m_LifterActionServer->registerGoalCallback(boost::bind(&HamalLifterController::lifterActionGoalCallback, this));
        m_PointPublisher = base_nh.advertise<std_msgs::Float64>("/traj_points", 10);
        m_CommandVelPublisher = base_nh.advertise<std_msgs::Float64>("/command_vel", 10);

        return true;
    }   

    void HamalLifterController::update(const ros::Time& time, const ros::Duration& period)
    {   
        const auto currentPosition = m_LifterJointHandle.getPosition();
        const auto currentVelocity = m_LifterJointHandle.getVelocity();
        ROS_INFO("Current Position: %f", currentPosition);
        const auto currentTime = time; 
        if(m_LifterActionServer->isActive())
        {  
            
            if(inRange(currentPosition, m_TargetPosition, m_Tolerance))
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
                m_StartPosition = m_LifterJointHandle.getPosition();
                m_IsGoalNew = false;
            }

            ROS_INFO("Profile Time: %f", m_TargetTime);

            const double time = (currentTime - m_StartTime).toSec();

            if(time > m_TargetTime)
            {
                LifterResult res;
                res.target_reached =false;
                ROS_ERROR("Elapsed time is bigger than target profile time. Aborting...");
                m_LifterActionServer->setAborted(res);
                
                m_LifterJointHandle.setCommandPosition(0.0);
                m_LifterJointHandle.setCommandVelocity(0.0);
                m_LifterJointHandle.setCommandAcceleration(0.0);
                m_TargetPosition = 0.0;
                m_TargetTime = 0.0;
                m_IsGoalNew = true;
                return;
            }

            Commands newCommands;

            newCommands = computeCommands(
                time
            );

/*             newCommands.velocity += m_KffV * newCommands.velocity;
 */            const auto pidVal = m_VelocityController.pid(newCommands.position, currentPosition, currentTime);
/*             newCommands.velocity *= pidVal;
 */
/*             newCommands.velocity = pidVal + (m_KffV * newCommands.velocity);
 */ 
            checkLimits(newCommands);

            LifterFeedback feedback;
            feedback.target_command = newCommands.velocity;
            feedback.current_position = currentPosition;
            std_msgs::Float64 p;
            p.data = newCommands.position;
            m_PointPublisher.publish(p);

            p.data = newCommands.velocity *(((60.0/M_PI*2.0)) * 24.685);
            m_CommandVelPublisher.publish(p);

            m_LifterActionServer->publishFeedback(feedback);            

            std::cout << "After PID and FF: " << newCommands.velocity << std::endl;
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
        m_VelocityController.initController(m_StartTime, m_PidParams.Kp, m_PidParams.Ki, m_PidParams.Kd);
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

       /*  double duration = std::fmax(
            (15.0 * std::fabs(m_TargetPosition - current_position)) / (8.0 * m_MaxVel),
            (std::sqrt((10.0 * std::fabs(m_TargetPosition - current_position)) / (m_MaxAccel * std::sqrt(3.0))))
        ); */

        /* double duration = fmax((15.0 * (fabs(m_TargetPosition - current_position) / 2.0)) / (8.0 * m_MaxVel), sqrt((10.0 * (fabs(m_TargetPosition - current_position) / 2.0)) / (m_MaxAccel * sqrt(3.0)))); */
        double duration = fmax((15.0 * (fabs(m_TargetPosition - current_position))) / (8.0 * m_MaxVel), sqrt((10.0 * (fabs(m_TargetPosition - current_position))) / (m_MaxAccel * sqrt(3.0))));
        return duration;
    }

    Commands HamalLifterController::computeCommands(
        const double& current_time
    )
    {
        Commands cmds;

        double startPos = m_LifterJointHandle.getPosition();
        double startVel = m_LifterJointHandle.getVelocity();
        double startAccel = 0.0;
        m_TargetVelocity = 0.0;

        //startPos = m_StartPosition;
        auto coeffs = fifthOrderPolyCoeffs(
            startPos,
            m_TargetPosition,
            startVel,
            m_TargetVelocity,
            startAccel,
            m_TargetAccel,
            m_TargetTime,
            current_time
        );

        coeffs.a0 = 0.0;
        coeffs.a1 = 0.0;
        coeffs.a2 = 0.0;
        coeffs.a3 = (20.0*m_TargetPosition - 20.0*startPos) / (2.0 * std::pow(m_TargetTime, 3));
        coeffs.a4 = (30.0*m_TargetPosition - 30.0*startPos) / (2.0 * std::pow(m_TargetTime, 4));
        coeffs.a5 = (12.0*m_TargetPosition - 12.0*startPos) / (2.0 * std::pow(m_TargetTime, 5));

        cmds.position = (
            coeffs.a0   +
            coeffs.a1 * current_time    +
            coeffs.a2 * std::pow(current_time, 2)   +
            coeffs.a3 * std::pow(current_time, 3)   +
            coeffs.a4 * std::pow(current_time, 4)   +
            coeffs.a5 * std::pow(current_time, 5)
        );

        cmds.velocity = (
            coeffs.a1    +
            2.0 * coeffs.a2 * std::pow(current_time, 1)   +
            3.0 * coeffs.a3 * std::pow(current_time, 2)   +
            4.0 * coeffs.a4 * std::pow(current_time, 3)   +
            5.0* coeffs.a5 * std::pow(current_time, 4)
        );

        cmds.accel = (
            2.0 * coeffs.a2   +
            6.0 * coeffs.a3 * std::pow(current_time, 1)   +
            12.0 * coeffs.a4 * std::pow(current_time, 2)   +
            20.0 * coeffs.a5 * std::pow(current_time, 3)
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
        coeffs.a0 = 0.0;
        coeffs.a1 = 0.0;
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

        /* if(commands.velocity < 0.436 && commands.velocity > 0)
            commands.velocity = 0.436;
        else if(commands.velocity > -0.436 && commands.velocity < 0)
            commands.velocity = -0.436; */

            
    }

}

PLUGINLIB_EXPORT_CLASS(
    hamal_lifter_controller::HamalLifterController, controller_interface::ControllerBase
);
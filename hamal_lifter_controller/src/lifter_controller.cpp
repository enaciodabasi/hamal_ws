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
        : m_CurrentActionGoalPtr(nullptr)
    {

    }

    HamalLifterController::~HamalLifterController()
    {
        delete m_CurrentActionGoalPtr;
    }

    bool HamalLifterController::init(hardware_interface::PosVelAccJointInterface* lifter_control_interface, ros::NodeHandle &base_nh, ros::NodeHandle &controller_nh)
    {
        /* const std::string complete_ns = controller_nh.getNamespace();
        std::size_t id = complete_ns.find_last_of("/");
        m_ControllerName = complete_ns.substr(id + 1); */

        if(controller_nh.hasParam("lifter_joint_name"))
        {
            controller_nh.getParam("lifter_joint_name", m_LifterJointName);
        }

        m_LifterJointHandle = lifter_control_interface->getHandle(m_LifterJointName);

        controller_nh.param("controller_frequency", m_ControllerFrequency, 50.0);
        m_ControllerRate = std::make_unique<ros::Rate>(m_ControllerFrequency);

        controller_nh.param("max_position", m_Limits.posLimit, 1.0);
        controller_nh.param("max_velocity", m_Limits.velLimit, 3.0);
        controller_nh.param("max_accel", m_Limits.accelLimit, 6.0);

        controller_nh.param("tolerance", m_PositionTolerance, 0.0);
        
        double kp, ki, kd = 0.0;
        controller_nh.param("pid/kp", kp, kp);
        controller_nh.param("pid/ki", ki, ki);
        controller_nh.param("pid/ki", kd, kd);

        m_PositionPID = PID();
        m_PositionPID.setParams(kp, ki, kd);

        //double Kffv = 0.0;
        //controller_nh.param("feed_forward/Kffv", Kffv, Kffv);
        //m_KffV = Kffv;

        m_LifterOperationServer = std::make_unique<LifterOperationActionServer>(base_nh, "lifter_action_server", false);
        m_LifterOperationServer->registerGoalCallback(boost::bind(&HamalLifterController::lifterActionCallback, this));

        return true;
    }

    void HamalLifterController::starting(const ros::Time &time)
    {
        m_PreviousUpdateTime = time;
        m_LifterOperationServer->start();
    }

    void HamalLifterController::update(const ros::Time &time, const ros::Duration &period)
    {
        if(m_CurrentActionGoalPtr){
            if(m_LifterJointHandle.getPosition() == m_CurrentActionGoalPtr->target_position){
                LifterResult res;
                res.target_reached = true;
                res.status_str = OperationSuccessStr;
                m_GoalActive = false;

                m_LifterOperationServer->setSucceeded(res);
            }
        }
        static double prevPosPoint = 0.0;
        // If a new goal is arrived set up the controller.
        if(m_NewGoalArrived){
            
            if(m_CurrentActionGoalPtr){
                
                if(m_CurrentActionGoalPtr->profile_time == 0.0){
                    m_GoalCoefficients = calculateControlParams(
                        m_LifterJointHandle.getPosition(),
                        m_LifterJointHandle.getVelocity(),
                        0.0,
                        m_CurrentActionGoalPtr->target_position
                    );
                }
            }
            m_PreviousUpdateTime = time;
            m_NewGoalArrived = false;
            m_GoalActive = true;
            prevPosPoint = m_LifterJointHandle.getPosition();
            
            m_ControllerRate->sleep();
            
            return;
        }
        else if(m_GoalActive){

            const auto tc = time - m_PreviousUpdateTime; // tc: Current Time
            
            // If elapsed time is longer than maximum profile time:
            if(tc > m_MaxProfileTime){
                LifterResult res;
                res.target_reached = false;
                res.status_str = ProfileTimeErrorStr;
                m_LifterOperationServer->setAborted(res);
                
            }

            const double tcSec = tc.toSec();
            double posRef = m_GoalCoefficients.a0 + (m_GoalCoefficients.a1 * (tcSec)) + (m_GoalCoefficients.a2 * (tcSec*tcSec)) + (m_GoalCoefficients.a3 * (tcSec*tcSec*tcSec)) + (m_GoalCoefficients.a4 * (tcSec*tcSec*tcSec*tcSec)) + (m_GoalCoefficients.a5 * (tcSec*tcSec*tcSec*tcSec*tcSec));
            double velRef =  m_GoalCoefficients.a1 + (2.0 * m_GoalCoefficients.a2 *(tcSec)) + (3.0 * m_GoalCoefficients.a3 * (tcSec*tcSec)) + (4.0 * m_GoalCoefficients.a4 * (tcSec*tcSec*tcSec)) + (5.0 * m_GoalCoefficients.a5 * (tcSec*tcSec*tcSec*tcSec));
            double accRef = (2.0 * m_GoalCoefficients.a2) + (6.0 * m_GoalCoefficients.a3 * (tcSec)) + (12.0 * m_GoalCoefficients.a4 * (tcSec*tcSec)) + (20.0 * m_GoalCoefficients.a5 * (tcSec*tcSec*tcSec));
            
            double cyclicPosRef = posRef - prevPosPoint;
            prevPosPoint = posRef;

            if(std::abs(cyclicPosRef) > m_Limits.posLimit){
                (cyclicPosRef < 0 ? cyclicPosRef = -1.0 * m_Limits.posLimit : cyclicPosRef = m_Limits.posLimit);
            }
    
            if(std::abs(velRef) > m_Limits.velLimit){
                (velRef < 0 ? velRef = -1.0 * m_Limits.velLimit : velRef = m_Limits.velLimit);
            }

            if(std::abs(accRef) > m_Limits.accelLimit){
                (accRef < 0 ? accRef = -1.0 * m_Limits.accelLimit : accRef = m_Limits.accelLimit );
            }
            
            m_LifterJointHandle.setCommandPosition(cyclicPosRef);
            m_LifterJointHandle.setCommandVelocity(velRef);
            m_LifterJointHandle.setCommandAcceleration(accRef);

        }
        else if(m_GoalDone){ // Clean up.
            cleanup();
        }
        else if(m_GoalError){
            cleanup();
        }
        else{

        }

        m_ControllerRate->sleep();

    }

    void HamalLifterController::stopping(const ros::Time &time)
    {
        m_LifterOperationServer->shutdown();
    }

    void HamalLifterController::lifterActionCallback()
    {
        const auto goal = m_LifterOperationServer->acceptNewGoal();
        m_CurrentActionGoalPtr = new hamal_custom_interfaces::LifterOperationGoal(*goal);
        m_NewGoalArrived = true;
    }

    const HamalLifterController::Coefficients HamalLifterController::calculateControlParams(
        double profile_time,
        const double &initial_position,
        const double &initial_velocity,
        const double &initial_acceleration,
        const double &target_position,
        const double &target_vel,
        const double &target_acc)
    {
        const double distanceToTravel = target_position - initial_position;
        const double tf = profile_time + 0.0; // tf = ti + tp (tp = profile duration) 
        Coefficients coeffs;
        coeffs.a0 = initial_position;
        coeffs.a1 = initial_velocity;
        coeffs.a2 = initial_acceleration / 2.0;
        coeffs.a3 = (10.0 * distanceToTravel) / (tf*tf*tf);
        coeffs.a4 = (-15.0 * distanceToTravel) / (tf*tf*tf*tf);
        coeffs.a5 = (6.0 * distanceToTravel) / (tf*tf*tf*tf*tf);

        m_MaxProfileTime = ros::Duration(profile_time);

        return coeffs;
    }

    const HamalLifterController::Coefficients HamalLifterController::calculateControlParams(
        const double& initial_position,
        const double& initial_velocity,
        const double& initial_acceleration,
        const double& target_position,
        const double& target_vel,
        const double& target_acc
    )
    {
        double duration = fmax((15.0 * (fabs(target_position - initial_position))) / (8.0 * m_Limits.velLimit), sqrt((10.0 * (fabs(target_position - initial_position))) / (m_Limits.accelLimit * sqrt(3.0))));
        return calculateControlParams(duration, initial_position, initial_velocity, initial_acceleration, target_position, target_vel, target_acc);
    }

    void HamalLifterController::cleanup()
    {
        m_CurrentActionGoalPtr = nullptr;
        m_GoalCoefficients.reset();
        m_NewGoalArrived = false;
        m_GoalActive = false;
        m_GoalDone = false;
        m_GoalError = false;
    }

}

PLUGINLIB_EXPORT_CLASS(
    hamal_lifter_controller::HamalLifterController, controller_interface::ControllerBase
);
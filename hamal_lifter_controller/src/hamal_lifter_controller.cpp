#include "hamal_lifter_controller/hamal_lifter_controller.hpp"

namespace hamal_lifter_controller
{

    HamalLifterController::HamalLifterController()
    {

    }

    HamalLifterController::~HamalLifterController()
    {
        m_LifterOperationServer->shutdown();   
    }

    bool HamalLifterController::init(hardware_interface::PosVelAccJointInterface* lifter_control_interface, ros::NodeHandle &base_nh, ros::NodeHandle &controller_nh)
    {
        if(controller_nh.hasParam("lifter_joint_name"))
        {
            controller_nh.getParam("lifter_joint_name", m_LifterJointName);
        }

        if(controller_nh.hasParam("position_tolerance"))
        {
            /* double toleranceTemp; */
            controller_nh.getParam("position_tolerance", m_LifterPositionTolerance);
        }

        m_LifterJointHandle = lifter_control_interface->getHandle(m_LifterJointName);

        //controller_nh.param("controller_frequency", m_ControllerFrequency, 50.0);

        m_LifterOperationServer = std::make_unique<LifterOperationActionServer>(base_nh, "lifter_action_server", false);
        m_LifterOperationServer->registerGoalCallback(
            boost::bind(
                &HamalLifterController::lifterActionCallback, this
            )
        );

        return true;
    }

    void HamalLifterController::starting(const ros::Time &time)
    {

        m_LifterOperationServer->start();
    }

    void HamalLifterController::update(const ros::Time &time, const ros::Duration &period)
    {
        if(m_CommandQueue.empty())
        {
            return;
        }
        
        // This means a new goal has arrived:
        if(!isGoalCurrentlyActive && !m_CommandQueue.empty())
        {   
            auto currentGoalCopy = m_CommandQueue.front();
/*             m_LifterMotionProfileController.setupProfile(currentGoalCopy.x_target, MotionProfileType::Trapezoidal);
 */

            auto times = motion_profile_generators::triangular_profile::calculateTriangularOperationTimes<double, double>(
                currentGoalCopy.x_target,
                m_MotionConstraints
            );
            
            isGoalCurrentlyActive = true;
        }        
        bool goalDoneWithFail = false;
        bool goalDoneWithSucc = false;

        if(isGoalCurrentlyActive)
        {   

            if(inRange(m_LifterJointHandle.getPosition(), m_LifterPositionTolerance, m_CommandQueue.front().x_target))
            {
                goalDoneWithSucc = true;
            }   

            if(goalDoneWithSucc)
            {
                /* auto newRef = m_LifterMotionProfileController.generateMotionProfileReference(
                m_CommandQueue.front().x_target
                ); */
                
                if(newRef)
                {
                    m_LifterJointHandle.setCommandPosition(newRef->position);
                    m_LifterJointHandle.setCommandVelocity(newRef->velocity);
                    m_LifterJointHandle.setCommandAcceleration(newRef->acceleration);
                    /* m_LifterOperationServer->publishFeedback(
                    
                    ) */
                }
                else
                {
                    
                    goalDoneWithFail = true;
                }
            }
            
        }
        
        if(goalDoneWithFail || goalDoneWithSucc)
        {
            if(goalDoneWithFail)
            {
                LifterResult res;
                res.target_reached = false;
                m_LifterOperationServer->setAborted(res);
                m_CommandQueue.pop();
            }
            else if(goalDoneWithSucc)
            {
                LifterResult res;
                res.target_reached = true;
                m_LifterOperationServer->setSucceeded(res);
                m_CommandQueue.pop();
            }

            isGoalCurrentlyActive = false;
        }


    }

    void HamalLifterController::stopping(const ros::Time &time)
    {
        
    }

    void HamalLifterController::lifterActionCallback()
    {
        const auto goal = m_LifterOperationServer->acceptNewGoal();
        hamal_custom_interfaces::MotionProfileCommand cmd;
        cmd.x_target = goal->target_position;
        m_CommandQueue.push(cmd);
    }


}

PLUGINLIB_EXPORT_CLASS(
    hamal_lifter_controller::HamalLifterController, controller_interface::ControllerBase
);
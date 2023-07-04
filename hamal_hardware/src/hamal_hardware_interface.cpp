/**
 * @file hamal_hardware_interface.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hamal_hardware/hamal_hardware_interface.hpp"

namespace hamal
{
    HardwareInterface::HardwareInterface(ros::NodeHandle& nh)
    {
        configure();

        m_ControllerManager = std::make_shared<controller_manager::ControllerManager>(
            this,
            m_NodeHandle
        );

        m_EthercatController = std::make_unique<HamalEthercatController>(
            m_EthercatConfigFilePath,
            true
        );
        
        // Register Joint State and Command Interfaces
        for(auto& [joint_name, wheel_joint] : m_WheelJointsMap)
        {
            hardware_interface::JointStateHandle jointStateHandle(
                wheel_joint.jointName,
                &wheel_joint.position,
                &wheel_joint.velocity,
                &wheel_joint.effort
            );

            m_JointStateInterface.registerHandle(jointStateHandle);

            hardware_interface::JointHandle velCommandHandle(
                jointStateHandle,
                &wheel_joint.targetVelocity
            );

            m_VelJointInterface.registerHandle(velCommandHandle);
        }

        this->registerInterface(&m_JointStateInterface);
        this->registerInterface(&m_VelJointInterface);
    }

    HardwareInterface::~HardwareInterface()
    {

    }

    void HardwareInterface::update()
    {
        ros::Rate rate(m_LoopFrequency);

        ros::Time prevTime = ros::Time::now();

        while(ros::ok() && m_RosLoopFlag)
        {
            // Read

            read();
            // Update CM
            ros::Time current = ros::Time::now();
            ros::Duration period = ros::Duration(current - prevTime);
            m_ControllerManager->update(current, period);
            prevTime = current;

            // Write();

            write();

            rate.sleep();

        }
    }

    void HardwareInterface::configure()
    {

        std::vector<std::string> wheelJointNames;

        if(m_NodeHandle.hasParam("/hamal/hardware_interface/wheel_joints"))
        {
            m_NodeHandle.getParam("/hamal/hardware_interface/wheel_joints", wheelJointNames);
        }
        else
        {
            ROS_ERROR("No joint names find in the parameter server. Shutting down the hardware interface.");
            if(ros::ok())
                ros::shutdown();
        }

        for(const auto& wheel_joint_name : wheelJointNames)
        {
            m_WheelJointsMap[wheel_joint_name] = WheelJointHandle(wheel_joint_name);
        }

        if(m_NodeHandle.hasParam("/hamal/harware_interface/loop_hz"))
        {
            m_NodeHandle.getParam("/hamal/hardware_interface/loop_hz", m_LoopFrequency);
        }
        else
        {
            ROS_INFO("Loop freuency is not specified in the parameter server. Defaulting back to 50 Hz");
        }

        if(m_NodeHandle.hasParam("/hamal/hardware_interface/ethercat_config_path"))
        {
            m_NodeHandle.getParam("/hamal/hardware_interface/ethercat_config_path", m_EthercatConfigFilePath);
        }
    }

    void HardwareInterface::read()
    {
        auto rawLeftPosOpt = m_EthercatController->getData<int32_t>("SOMANET_NODE_0", "current_position");
        auto rawRightPosOpt = m_EthercatController->getData<int32_t>("SOMANET_NODE_1", "current_position");

        // Write to values to joints
        if(rawLeftPosOpt != std::nullopt && rawRightPosOpt != std::nullopt)
        {
            /* if(m_EthercatController->m_SystemEnabled)
            {
                if(m_WheelHomingHelper.initialEcRead)
                {
                    m_WheelHomingHelper.leftPosDiff = amr::utils::motorPositionToWheelPositionRad(rawLeftPosOpt.value(), m_PositionHelper);
                    m_WheelHomingHelper.rightPosDiff = amr::utils::motorPositionToWheelPositionRad(rawRightPosOpt.value(), m_PositionHelper);
                    
                    m_WheelHomingHelper.initialEcRead = false;
                }
            } */
        }

    }

    void HardwareInterface::write()
    {

    }
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "hamal_hw");
    ros::NodeHandle nh;
    hamal::HardwareInterface hw(nh);
    ros::AsyncSpinner asyncSpinner(0);
    
    asyncSpinner.start();

    ros::waitForShutdown();

    return 0;
}
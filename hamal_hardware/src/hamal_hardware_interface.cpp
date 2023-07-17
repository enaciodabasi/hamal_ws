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

        bool ec_ok = m_EthercatController->setup();
        if(!ec_ok)
        {
            ros::shutdown();
        }
        /* if(m_LifterInterfaceType == LifterInterfaceType::Position)
        {
            m_EthercatController->setLifterControlType(ControlType::Position);
        }
        else if(m_LifterInterfaceType == LifterInterfaceType::Velocity)
        {
            m_EthercatController->setLifterControlType(ControlType::Velocity);
        } */
        
        // Register Joint State and Command Interfaces
        for(auto& [joint_name, joint_handle] : m_JointsMap)
        {   

            hardware_interface::JointStateHandle jointStateHandle(
                joint_handle.jointName,
                &joint_handle.position,
                &joint_handle.velocity,
                &joint_handle.effort
            );

            m_JointStateInterface.registerHandle(jointStateHandle);

            /*
            // If control type of the lifter is specified as position
            // Skip registering inside the velocity command interface
            if(joint_name == m_LifterJointName && m_LifterInterfaceType == LifterInterfaceType::Position)
            {   
                m_LifterPositionInterface = hardware_interface::PositionJointInterface();
                
                hardware_interface::JointHandle posCommandHandle(
                    jointStateHandle,
                    &joint_handle.targetPosition
                );
                m_LifterPositionInterface.value().registerHandle(posCommandHandle);
                continue;
            }
            */
            hardware_interface::JointHandle velCommandHandle(
                jointStateHandle,
                &joint_handle.targetVelocity
            );

            m_VelJointInterface.registerHandle(velCommandHandle);
        }  

        /* if(m_LifterInterfaceType == LifterInterfaceType::Velocity)
        {
            m_LifterPositionInterface = std::nullopt;
        }
 */
        this->registerInterface(&m_JointStateInterface);
        this->registerInterface(&m_VelJointInterface);
        /* if(m_LifterPositionInterface != std::nullopt)
        {
            this->registerInterface(&m_LifterPositionInterface.value());
        } */

        m_EthercatController->startTask();

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

            write();

            rate.sleep();

        }

        m_EthercatController->m_EthercatLoopFlag = false;
    }

    void HardwareInterface::configure()
    {

        /* std::vector<std::string> wheelJointNames; */

        /* if(m_NodeHandle.hasParam("/hamal/hardware_interface/wheel_joints"))
        {
            m_NodeHandle.getParam("/hamal/hardware_interface/wheel_joints", wheelJointNames);
            for(const auto& wheel_joint_name : wheelJointNames)
            {
                m_JointsMap[wheel_joint_name] = JointHandle(wheel_joint_name);
            }
        }
        else
        {
            ROS_ERROR("No joint names find in the parameter server. Shutting down the hardware interface.");
            /* if(ros::ok())
                ros::shutdown(); 
        }
        */

        if(m_NodeHandle.hasParam("/hamal/hardware_interface/left_wheel") && m_NodeHandle.hasParam("/hamal/hardware_interface/right_wheel"))
        {
            m_NodeHandle.getParam("/hamal/hardware_interface/left_wheel", m_LeftWheelJointName);
            m_JointsMap[m_LeftWheelJointName] = JointHandle(m_LeftWheelJointName);
            m_NodeHandle.getParam("/hamal/hardware_interface/right_wheel", m_RightWheelJointName);
            m_JointsMap[m_RightWheelJointName] = JointHandle(m_RightWheelJointName);
        }
        else
        {
            ROS_ERROR("No joint name was found in the config file.");
            if(ros::ok())
                ros::shutdown();
        }

        /* if(m_NodeHandle.hasParam("/hamal/hardware_interface/lifter_joint"))
        {
            std::string lifterJointName;
            m_NodeHandle.getParam("/hamal/hardware_interface/lifter_joint", lifterJointName);
            m_JointsMap[lifterJointName] = JointHandle(lifterJointName);
            m_JointsMap.at(lifterJointName).targetVelocity = 0.0;
            m_LifterJointName = lifterJointName;

        } */

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

        /* if(m_NodeHandle.hasParam("/hamal/hardware_interface/lifter_interface_type"))
        {
            std::string lifterInterfaceType;
            m_NodeHandle.getParam("/hamal/hardware_interface/lifter_interface_type", lifterInterfaceType);
        
            if(lifterInterfaceType == "position")
            {
                m_LifterInterfaceType = LifterInterfaceType::Position;
            }
            else if(lifterInterfaceType == "velocity")
            {
                m_LifterInterfaceType = LifterInterfaceType::Velocity;
            }
        } */

        if(m_NodeHandle.hasParam("/hamal/hardware_interface/reduction"))
        {
            m_NodeHandle.getParam("/hamal/hardware_interface/reduction", m_Reduction);
        }
        else
        {
            m_Reduction = 24.985;
        }

        if(m_NodeHandle.hasParam("/hamal/hardware_interface/position_increment"))
        {
            m_NodeHandle.getParam("/hamal/hardware_interface/position_increment", m_Increment);
        }
        else
        {
            m_Reduction = 20480.0;
        }

    }

    void HardwareInterface::read()
    {
        const auto leftWheelPosition = m_EthercatController->getData<int32_t>("somanet_node_0", "actual_position");
        const auto rightWheelPosition = m_EthercatController->getData<int32_t>("somanet_node_0", "actual_position");

        if(leftWheelPosition && rightWheelPosition)
        {
            m_JointsMap.at(m_LeftWheelJointName).position = leftWheelPosition.value();
            m_JointsMap.at(m_RightWheelJointName).position = rightWheelPosition.value();
        }

        const auto leftWheelVelocity = m_EthercatController->getData<int32_t>("somanet_node_0", "actual_velocity");
        const auto rightWheelVelocity = m_EthercatController->getData<int32_t>("somanet_node_0", "actual_velocity");
        
        if(leftWheelVelocity && rightWheelVelocity)
        {
            m_JointsMap.at(m_LeftWheelJointName).velocity = leftWheelVelocity.value();
            m_JointsMap.at(m_RightWheelJointName).velocity = rightWheelVelocity.value();
        }

    }

    void HardwareInterface::write()
    {
        const auto leftWheelTargetVel = m_JointsMap.at(m_LeftWheelJointName).targetVelocity;
        const auto rightWheelTargetVel = m_JointsMap.at(m_RightWheelJointName).targetVelocity;

        /* std::string s = "Target Velocity: " + std::to_string(leftWheelTargetVel) + "[rad/s]\n";
        ROS_INFO(s.c_str()); */

        m_EthercatController->setData<int32_t>("somanet_node_0", "target_velocity", jointVelocityToMotorVelocity(leftWheelTargetVel));
        /* m_EthercatController->setData<int32_t>("somanet_node_0", "target_velocity", jointVelocityToMotorVelocity(rightWheelTargetVel)); */
    }
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "hamal_hw");
    ros::NodeHandle nh;
    hamal::HardwareInterface hw(nh);
    ros::AsyncSpinner asyncSpinner(5);
    
    asyncSpinner.start();

    hw.update();

    ros::waitForShutdown();

    return 0;
}
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
        
        m_LifterHomingHelper = std::make_shared<HomingHelper>();

        m_EthercatController = std::make_unique<HamalEthercatController>(
            m_EthercatConfigFilePath,
            m_LifterHomingHelper,
            true
        );

        bool ec_ok = m_EthercatController->setup();
        if(!ec_ok)
        {
            ros::shutdown();
        }
        if(m_JointsMap.at(m_LifterJointName).controlType == ControlType::Position)
        {
            m_EthercatController->setLifterControlType(ControlType::Position);
        }
        else if(m_JointsMap.at(m_LifterJointName).controlType == ControlType::Velocity)
        {
            m_EthercatController->setLifterControlType(ControlType::Velocity);
        }
        
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

            if(joint_name == m_LifterJointName)
            {
                hardware_interface::PosVelAccJointHandle posVelAccCommandHandle(
                    jointStateHandle,
                    &joint_handle.targetPosition,
                    &joint_handle.targetVelocity,
                    &joint_handle.targetAccel
                );
                
                m_LifterJointInterface.registerHandle(posVelAccCommandHandle);

                continue;
            }

            hardware_interface::JointHandle velCommandHandle(
                jointStateHandle,
                &joint_handle.targetVelocity
            );


            m_VelJointInterface.registerHandle(velCommandHandle);
        }  

        this->registerInterface(&m_JointStateInterface);
        this->registerInterface(&m_VelJointInterface);
        this->registerInterface(&m_LifterJointInterface);

        m_HomingServer = std::make_unique<HomingActionServer>(
            m_NodeHandle,
            "lifter_homing_server",
            false
        );
        m_HomingServer->registerGoalCallback(
            boost::bind(&HardwareInterface::executeHomingCallback, this)
        );

        /* m_LifterOperationServer = std::make_unique<LifterOperationActionServer>(
            m_NodeHandle,
            "lifter_command_server",
            false
        );
        m_LifterOperationServer->registerGoalCallback(
            boost::bind(&HardwareInterface::lifterOperationCallback, this)
        ); */

        m_HardwareInfoPub = m_NodeHandle.advertise<hamal_custom_interfaces::HardwareInfoArray>(
            "hamal/hardware_info",
            10
        );

        m_EthercatController->setLimiterParams(
            m_LimiterParams.max_vel,
            m_LimiterParams.min_vel,
            m_LimiterParams.max_acc,
            m_LimiterParams.min_acc
            
        );
        m_EthercatController->startTask();

        m_HomingServer->start();
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

            if(m_LifterHomingHelper->isHomingActive){
                ROS_INFO("Homing is active");
                if(m_HomingServer->isPreemptRequested()){
                    m_HomingServer->setPreempted();
                }

                const auto homingStatus = m_LifterHomingHelper->getCurrentHomingStatus();
                const auto homingStatusStr = HomingStatusStrings.find(homingStatus);
                switch (homingStatus)
                {
                case HomingStatus::HomingIsPerformed :
                case HomingStatus::HomingIsInterruptedOrNotStarted :
                case HomingStatus::HomingConfirmedTargetNotReached :
                {
                    
                    if(inRange<double>(m_JointsMap.at(m_LifterJointName).position, 0.0, 0.5)){
                        hamal_custom_interfaces::HomingOperationResult homingRes;
                        homingRes.status = homingStatusStr->second;
                        homingRes.homingDone = true;
                        m_HomingServer->setSucceeded(homingRes);
                        m_LifterHomingHelper->reset();
                    }
                    else{
                        hamal_custom_interfaces::HomingOperationFeedback homingFb;
                        homingFb.homingStatus = homingStatusStr->second;

                        m_HomingServer->publishFeedback(homingFb);
                    }
                    break;
                }
                case HomingStatus::HomingCompleted :
                {
                    hamal_custom_interfaces::HomingOperationResult homingRes;
                    homingRes.status = homingStatusStr->second;
                    homingRes.homingDone = true;
                    m_HomingServer->setSucceeded(homingRes);
                    m_LifterHomingHelper->reset();
                    break;
                }
                case HomingStatus::ErrorDetectedMotorStillRunning :
                case HomingStatus::ErrorDuringHomingMotorAtStandstill :
                {
                    hamal_custom_interfaces::HomingOperationResult homingRes;
                    homingRes.status = homingStatusStr->second;
                    homingRes.homingDone = false;
                    m_HomingServer->setAborted(homingRes);
                    m_LifterHomingHelper->reset();
                    break;
                }
                default:
                    break;
                }

            }

            const auto status = m_EthercatController->getSlaveStatus();
            m_JointsMap.at(m_LifterJointName).hardwareInfo.current_state = status.at(0).second;
            m_JointsMap.at(m_LeftWheelJointName).hardwareInfo.current_state = status.at(1).second;
            m_JointsMap.at(m_RightWheelJointName).hardwareInfo.current_state = status.at(2).second;

            std::vector<hamal_custom_interfaces::HardwareInfo> infoArray = {
                m_JointsMap.at(m_LifterJointName).hardwareInfo,
                m_JointsMap.at(m_LeftWheelJointName).hardwareInfo,
                m_JointsMap.at(m_RightWheelJointName).hardwareInfo
            };
            hamal_custom_interfaces::HardwareInfoArray infoArrayMsg;
            infoArrayMsg.hardware_info_array = infoArray;
            m_HardwareInfoPub.publish((
                infoArrayMsg
            ));

            rate.sleep();

        }

        m_EthercatController->m_EthercatLoopFlag = false;
    }

    void HardwareInterface::configure()
    {

        if(
            (m_NodeHandle.hasParam("/hamal/hardware_interface/left_wheel")) && 
            (m_NodeHandle.hasParam("/hamal/hardware_interface/right_wheel")) &&
            (m_NodeHandle.hasParam("/hamal/hardware_interface/lifter_joint"))
        )
        {
            m_NodeHandle.getParam("/hamal/hardware_interface/left_wheel", m_LeftWheelJointName);
            m_JointsMap[m_LeftWheelJointName] = JointHandle(m_LeftWheelJointName);
            m_NodeHandle.getParam("/hamal/hardware_interface/right_wheel", m_RightWheelJointName);
            m_JointsMap[m_RightWheelJointName] = JointHandle(m_RightWheelJointName);    
            m_NodeHandle.getParam("/hamal/hardware_interface/lifter_joint", m_LifterJointName);
            m_JointsMap[m_LifterJointName]  = JointHandle(m_LifterJointName);
            
            
        }
        else
        {
            ROS_ERROR("One or more joint names are missing in the configuration file.");
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

        if(m_NodeHandle.hasParam("/hamal/hardware_interface/lifter_control_type"))
        {
            std::string lifterControlType;
            m_NodeHandle.getParam("/hamal/hardware_interface/lifter_control_type", lifterControlType);
        
            if(lifterControlType == "position")
            {
                m_JointsMap.at(m_LifterJointName).controlType = ControlType::Position;
            }
            else if(lifterControlType == "velocity")
            {
                m_JointsMap.at(m_LifterJointName).controlType = ControlType::Velocity;
            }
        }

        if(m_NodeHandle.hasParam("/hamal/hardware_interface/reduction"))
        {
            m_NodeHandle.getParam("/hamal/hardware_interface/reduction", m_Reduction);
        }
        else
        {
            m_Reduction = 24.985;
        }

        if(m_NodeHandle.hasParam("/hamal/hardware_interface/lifter_reduction"))
        {
            m_NodeHandle.getParam("/hamal/hardware_interface/lifter_reduction", m_LifterMotorReduction);
        }
        else
        {
            m_LifterMotorReduction = 1.0;
        }

        if(m_NodeHandle.hasParam("/hamal/hardware_interface/position_increment"))
        {
            m_NodeHandle.getParam("/hamal/hardware_interface/position_increment", m_Increment);
        }
        else
        {
            m_Reduction = 20480.0;
        }

        if(m_NodeHandle.hasParam("/hamal/hardware_interface/lifter_position_increment"))
        {
            m_NodeHandle.getParam("/hamal/hardware_interface/lifter_position_increment", m_LifterIncrement);
        }
        else
        {
            m_LifterIncrement = 20480.0;
        }

        if(m_NodeHandle.hasParam("/hamal/hardware_interface/max_vel")){
            m_NodeHandle.getParam("/hamal/hardware_interface/max_vel", m_LimiterParams.max_vel);
        }
        if(m_NodeHandle.hasParam("/hamal/hardware_interface/min_vel")){
            m_NodeHandle.getParam("/hamal/hardware_interface/min_vel", m_LimiterParams.min_vel);
        }
        if(m_NodeHandle.hasParam("/hamal/hardware_interface/max_acc")){
            m_NodeHandle.getParam("/hamal/hardware_interface/max_acc", m_LimiterParams.max_acc);
        }
        if(m_NodeHandle.hasParam("/hamal/hardware_interface/min_acc")){
            m_NodeHandle.getParam("/hamal/hardware_interface/min_acc", m_LimiterParams.min_acc);
        }

    }

    void HardwareInterface::read()
    {
        const auto rightWheelPosition = m_EthercatController->getData<int32_t>("somanet_node_1", "actual_position");
        const auto leftWheelPosition = m_EthercatController->getData<int32_t>("somanet_node_2", "actual_position");
        const auto lifterPosition = m_EthercatController->getData<int32_t>("somanet_node_0", "actual_position");
        if(leftWheelPosition && rightWheelPosition && lifterPosition)
        {

            m_JointsMap.at(m_LeftWheelJointName).position = (motorPositionToJointPosition(leftWheelPosition.value())) * -1.0;/* * 0.5 */;
            m_JointsMap.at(m_RightWheelJointName).position = (motorPositionToJointPosition(rightWheelPosition.value())) /* * 0.5 */;
            double lifterPos = static_cast<double>(lifterPosition.value());
            double lifterPosInRads = (lifterPos / m_LifterIncrement) * (2.0 * M_PI);
            m_JointsMap.at(m_LifterJointName).position = lifterPosInRads;
            
            m_JointsMap.at(m_LeftWheelJointName).hardwareInfo.current_pos = motorPositionToJointPosition(leftWheelPosition.value());
            m_JointsMap.at(m_RightWheelJointName).hardwareInfo.current_pos = motorPositionToJointPosition(rightWheelPosition.value());
            m_JointsMap.at(m_LifterJointName).hardwareInfo.current_pos = lifterPosInRads;
/*             ROS_INFO("Position: %f", leftWheelPosition.value());
 */        }

        const auto rightWheelVelocity = m_EthercatController->getData<int32_t>("somanet_node_1", "actual_velocity");
        const auto leftWheelVelocity = m_EthercatController->getData<int32_t>("somanet_node_2", "actual_velocity");
        const auto lifterVelocity = m_EthercatController->getData<int32_t>("somanet_node_0", "actual_velocity");

        if(leftWheelVelocity && rightWheelVelocity)
        {
            m_JointsMap.at(m_LeftWheelJointName).velocity = motorVelocityToJointVelocity(leftWheelVelocity.value());
            m_JointsMap.at(m_RightWheelJointName).velocity = motorVelocityToJointVelocity(rightWheelVelocity.value());
            double lifterVel = static_cast<double>(lifterVelocity.value());
            lifterVel = (lifterVel * 2.0 * M_PI) / 60.0;
            m_JointsMap.at(m_LifterJointName).velocity = lifterVel;


            m_JointsMap.at(m_RightWheelJointName).hardwareInfo.current_vel = rightWheelVelocity.value();
            m_JointsMap.at(m_LeftWheelJointName).hardwareInfo.current_vel = leftWheelVelocity.value();
            m_JointsMap.at(m_LifterJointName).hardwareInfo.current_vel = lifterVel;

        }

        //std::cout << "Lifter: " << m_EthercatController->getSlaveStateString("domain_0", "somanet_node_0").value() << std::endl;
        //std::cout << "Left Motor: "  << m_EthercatController->getSlaveStateString("domain_0", "somanet_node_1").value() << std::endl;
        //std::cout << "Right Motor: " << m_EthercatController->getSlaveStateString("domain_0", "somanet_node_2").value() << std::endl;

    }

    void HardwareInterface::write()
    {

        if(!m_EthercatController->isEthercatOk()){
            return;
        }
        const auto rightWheelTargetVel = m_JointsMap.at(m_RightWheelJointName).targetVelocity;
        const auto leftWheelTargetVel = m_JointsMap.at(m_LeftWheelJointName).targetVelocity;
        
        double lifterTarget = 0.0;
        double tempLifterTarget = 0.0;
        std::string targetsPdoName;
        const ControlType lifterControlType = m_JointsMap.at(m_LifterJointName).controlType;
        if(lifterControlType == ControlType::Position){
            double targetCmd = m_JointsMap.at(m_LifterJointName).targetPosition;
            lifterTarget = targetCmd;
            targetsPdoName = "target_position";
        }
        else if(lifterControlType == ControlType::Velocity){
            double targetCmd = m_JointsMap.at(m_LifterJointName).targetVelocity;
            targetCmd = targetCmd * (60.0 / (2.0 * M_PI));
            lifterTarget = 0.0;
            tempLifterTarget = targetCmd;
            targetsPdoName = "target_velocity";
        }

/*         ROS_INFO("Target Vel: %d", jointVelocityToMotorVelocity(leftWheelTargetVel));
 */     
        
        m_EthercatController->setData<int32_t>("somanet_node_2", "target_velocity", (jointVelocityToMotorVelocity(leftWheelTargetVel)));
        m_EthercatController->setData<int32_t>("somanet_node_1", "target_velocity", (jointVelocityToMotorVelocity(rightWheelTargetVel)));
        m_JointsMap.at(m_RightWheelJointName).hardwareInfo.target_vel = jointVelocityToMotorVelocity(rightWheelTargetVel);
        m_JointsMap.at(m_LeftWheelJointName).hardwareInfo.target_vel = (jointVelocityToMotorVelocity(leftWheelTargetVel));

        if(!targetsPdoName.empty()){
            m_EthercatController->setData<int32_t>("somanet_node_0", "target_velocity", tempLifterTarget);
            m_JointsMap.at(m_LifterJointName).hardwareInfo.target_vel = tempLifterTarget; 
        }
    }

    void HardwareInterface::executeHomingCallback()
    {
        const auto homingGoal = m_HomingServer->acceptNewGoal();
        const auto homingInfo = homingGoal->homingInfo;
        if(homingInfo.switchSearchSpeed != 0){
            m_LifterHomingHelper->switchSearchSpeed = homingInfo.switchSearchSpeed;
        }
        if(homingInfo.zeroSearchSpeed != 0){
            m_LifterHomingHelper->zeroSearchSpeed = homingInfo.zeroSearchSpeed;
        }
        if(homingInfo.homingAccel != 0){
            m_LifterHomingHelper->homingAccel = homingInfo.homingAccel;
        }
        if(homingInfo.homingMethod != 0){
            m_LifterHomingHelper->homingMethod = homingInfo.homingMethod;
        }

        m_LifterHomingHelper->isHomingActive = true;
        
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
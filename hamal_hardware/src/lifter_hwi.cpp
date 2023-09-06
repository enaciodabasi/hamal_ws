/**
 * @file lifter_hwi.cpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-09-06
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#include "hamal_hardware/lifter_hwi.hpp"


CommInterface::CommInterface(const std::string& path_to_config_file)
    : Controller(path_to_config_file)
{

}

void CommInterface::cyclicTask()
{
    clock_gettime(m_DcHelper.clock, &m_DcHelper.wakeupTime);
    while(m_LoopFlag)
    {
        setTaskWakeUpTime();
        sleep_task(m_DcHelper.clock, TIMER_ABSTIME, &m_DcHelper.wakeupTime, NULL);
        m_Master->setMasterTime(timespecToNanoSec(m_DcHelper.wakeupTime));
        
        m_Master->receive("lifter_domain");
        
        bool slavesEnabled = m_Master->enableSlaves();

        m_Master->write<int8_t>("lifter_domain", "somanet_node", "op_mode", 0x09);

        // Read
        if(slavesEnabled){
            auto lifterPos = m_Master->read<int32_t>("lifter_domain", "somanet_node", "actual_position");
            auto lifterVel = m_Master->read<int32_t>("lifter_domain", "somanet_node", "actual_velocity");

            if(lifterPos){

                setData<int32_t>("somanet_node", "actual_position", lifterPos.value());

                if(lifterVel){
                    setData<int32_t>("somanet_node", "actual_velocity", lifterVel.value());
                }
            } 
        }

        if(slavesEnabled){
            auto targetVelOpt = getData<int32_t>("somanet_node", "target_velocity");
            if(targetVelOpt){
                m_Master->write<int32_t>("lifter_domain", "somanet_node", "target_velocity", targetVelOpt.value());
            }
        }


        clock_gettime(m_DcHelper.clock, &m_DcHelper.currentTime);
        m_Master->syncMasterClock(timespecToNanoSec(m_DcHelper.currentTime), m_DcHelper);

        m_Master->send("lifter_domain");

    }
}




LifterHardwareInterface::LifterHardwareInterface(ros::NodeHandle& nh)
    : m_NodeHandle(nh)
{
    configure();
    
    m_HardwareInfoMsg = hamal_custom_interfaces::HardwareInfo();

    m_ControllerManager = std::make_shared<controller_manager::ControllerManager>(
        this,
        m_NodeHandle
    );

    m_CommInterface = std::make_unique<CommInterface>(m_CommConfigPath);

    bool ec_ok = m_CommInterface->setup();
    if(!ec_ok){
        ros::shutdown();
    }

    hardware_interface::JointStateHandle stateHandle(
        m_LifterJoint.m_LifterJointName,
        &m_LifterJoint.currentPos,
        &m_LifterJoint.currentVel,
        &m_LifterJoint.currentAcc
    );

    m_JointStateInterface.registerHandle(stateHandle);

    hardware_interface::PosVelAccJointHandle lifterCommandHandle(
        stateHandle,
        &m_LifterJoint.targetPos,
        &m_LifterJoint.targetVel,
        &m_LifterJoint.targetAcc
    );
    m_LifterJointInterface.registerHandle(lifterCommandHandle);

    registerInterface(&m_JointStateInterface);
    registerInterface(&m_LifterJointInterface);

    m_HardwareInfoPub = m_NodeHandle.advertise<hamal_custom_interfaces::HardwareInfo>("/hardware_info", 10);

    m_CommInterface->startTask();
}   

LifterHardwareInterface::~LifterHardwareInterface()
{

}

void LifterHardwareInterface::update()
{
    ros::Rate rate(m_LoopFreq);
    ros::Time prevTime = ros::Time::now();

    while(ros::ok() && m_RosLoopFlag)
    {
        read();
        ros::Time currTime = ros::Time::now();
        ros::Duration period = ros::Duration(currTime - prevTime);
        m_ControllerManager->update(currTime, period);
        prevTime = currTime;
        write();
        rate.sleep();
    }

    m_CommInterface->stopEcThread();
}

void LifterHardwareInterface::configure()
{

        if(m_NodeHandle.hasParam("/hamal/lifter_hardware_interface/lifter_joint"))
        {    
            m_NodeHandle.getParam("/hamal/lifter_hardware_interface/lifter_joint", m_LifterJoint.m_LifterJointName);
        }    
        else
        {
            ROS_ERROR("No joint name was found in the config file.");
            if(ros::ok())
                ros::shutdown();
        }

        if(m_NodeHandle.hasParam("/hamal/lifter_hardware_interface/loop_hz"))
        {
            m_NodeHandle.getParam("/hamal/lifter_hardware_interface/loop_hz", m_LoopFreq);
        }
        else
        {
            ROS_INFO("Loop freuency is not specified in the parameter server. Defaulting back to 50 Hz");
        }

        if(m_NodeHandle.hasParam("/hamal/lifter_hardware_interface/ethercat_config_path"))
        {
            m_NodeHandle.getParam("/hamal/lifter_hardware_interface/ethercat_config_path", m_CommConfigPath);
        }


        if(m_NodeHandle.hasParam("/hamal/lifter_hardware_interface/reduction"))
        {
            m_NodeHandle.getParam("/hamal/lifter_hardware_interface/reduction", m_Reduction);
        }
        else
        {
            m_Reduction = 24.985;
        }

        if(m_NodeHandle.hasParam("/hamal/lifter_hardware_interface/position_increment"))
        {
            m_NodeHandle.getParam("/hamal/lifter_hardware_interface/position_increment", m_EncoderResolution);
        }
        else
        {
            m_Reduction = 20480.0;
        }
}

void LifterHardwareInterface::read()
{
    const auto lifterPosition = m_CommInterface->getData<int32_t>("somanet_node", "actual_position");
    const auto lifterVel = m_CommInterface->getData<int32_t>("somanet_node", "actual_velocity");

    if(lifterPosition){
        m_LifterJoint.currentPos = motorPositionToJointPosition(lifterPosition.value());
        m_HardwareInfoMsg.current_pos = m_LifterJoint.currentPos;
    }

    if(lifterVel){
        m_LifterJoint.currentVel = motorVelocityToJointVelocity(lifterVel.value());
        m_HardwareInfoMsg.current_vel = m_LifterJoint.currentVel;
    }  

    const auto lifterSlaveStateStrOpt =  m_CommInterface->getSlaveStateString("lifter_domain", "somanet_node");
    if(lifterSlaveStateStrOpt)
        m_HardwareInfoMsg.current_state = lifterSlaveStateStrOpt.value();
        
    m_HardwareInfoMsg.timestamp = ros::Time::now();
    m_HardwareInfoPub.publish(m_HardwareInfoMsg);

}

void LifterHardwareInterface::write()
{
    const auto lifterTargetVel = m_LifterJoint.targetVel;
    int32_t lifterTargetRPM = jointVelocityToMotorVelocity(lifterTargetVel);
    m_CommInterface->setData<int32_t>("somanet_node", "target_velocity", lifterTargetRPM);
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "lifter_hardware_interface");
    ros::NodeHandle nh;
    LifterHardwareInterface hw(nh);
    ros::AsyncSpinner asyncSpinner(5);
    asyncSpinner.start();

    hw.update();

    ros::waitForShutdown();

    return 0;
}



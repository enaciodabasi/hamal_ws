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

const HomingStatus HomingHelper::getCurrentHomingStatus()
{
    if(!homingErrorBit && !homingAttainedBit && !targetReachedBit){
        return HomingStatus::HomingIsPerformed;
    }
    else if(!homingErrorBit && !homingAttainedBit && targetReachedBit){
        return HomingStatus::HomingIsInterruptedOrNotStarted;
    }
    else if(!homingErrorBit && homingAttainedBit && !targetReachedBit){
        return HomingStatus::HomingConfirmedTargetNotReached;
    }
    else if(!homingErrorBit && homingAttainedBit && targetReachedBit){
        return HomingStatus::HomingCompleted;
    }
    else if(homingErrorBit && !homingAttainedBit && !targetReachedBit){
        return HomingStatus::ErrorDetectedMotorStillRunning;
    }
    else if(homingErrorBit && !homingAttainedBit && targetReachedBit){
        return HomingStatus::ErrorDuringHomingMotorAtStandstill;
    }

    return HomingStatus::Unknown;
}

CommInterface::CommInterface(const std::string& path_to_config_file, std::shared_ptr<HomingHelper>& homing_helper_ptr)
    : Controller(path_to_config_file), m_HomingHelper(homing_helper_ptr)
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

        if(!m_HomingHelper->isHomingActive){
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
        }

        if(m_HomingHelper->isHomingActive){
            if(slavesEnabled && !m_HomingHelper->isHomingSetupDone){
                
                m_Master->write<int32_t>("lifter_domain", "somanet_node", "target_velocity", 0);
                m_Master->write<int8_t>("lifter_domain", "somanet_node", "op_mode", 0x06);
                m_Master->write<int8_t>("lifter_domain", "somanet_node", "homing_method", m_HomingHelper->homingMethod);
                m_Master->write<uint32_t>("lifter_domain", "somanet_node", "homing_speed", m_HomingHelper->switchSearchSpeed);
                m_Master->write<uint32_t>("lifter_domain", "somanet_node", "homing_speed2", m_HomingHelper->zeroSearchSpeed);
                m_Master->write<uint32_t>("lifter_domain", "somanet_node", "homing_accel", m_HomingHelper->homingAccel);
                m_HomingHelper->isHomingSetupDone = true;
                m_HomingHelper->isHomingInProgress = true;
            }


            bool opModeSetCorrect = false;
            const auto opModeDisplayOpt = m_Master->read<int8_t>("lifter_domain", "somanet_node", "op_mode_display");
            if(opModeDisplayOpt){
                if(opModeDisplayOpt.value() == 0x06){
                    opModeSetCorrect = true;
                }
            }

            if(!opModeSetCorrect){
                m_Master->write<int8_t>("lifter_domain", "somanet_node", "op_mode", 0x06);
            }

            if(m_HomingHelper->isHomingInProgress && opModeSetCorrect){
                /* uint16_t ctrlWord = [this]() -> uint16_t {
                    const auto ctrlWordOpt = m_Master->getControlWord("lifter_domain", "somanet_node");
                    if(ctrlWordOpt){
                        return ctrlWordOpt.value();
                    }
                    return 0x0;
                }();
                
                uint16_t statusWord = [this]() -> uint16_t {
                    const auto statusWordOpt = this->m_Master->read<uint16_t>("lifter_domain", "somanet_node", "status_word");
                    if(statusWordOpt){
                        return statusWordOpt.value();
                    }
                    return 0x0;
                }(); */

                const auto ctrlWordOpt = m_Master->getControlWord("lifter_domain", "somanet_node");
                const auto statusWordOpt = this->m_Master->read<uint16_t>("lifter_domain", "somanet_node", "status_word");
                if(ctrlWordOpt && statusWordOpt){
                    uint16_t ctrlWord = ctrlWordOpt.value();
                    uint16_t statusWord = statusWordOpt.value();
                    // From the Synapticon instructions:
                    // Set 8th bit to 0.
                    if(ethercat_interface::utilities::isBitSet(ctrlWord, 8))
                    {
                        ethercat_interface::utilities::resetBitAtIndex(ctrlWord, 8);
                    }
                    // Set 4th bit to 1.
                    if(!ethercat_interface::utilities::isBitSet(ctrlWord, 4)){
                        ethercat_interface::utilities::setBitAtIndex(ctrlWord, 4);
                    }

                    if(m_HomingHelper->previousCtrlWord != ctrlWord){
                        m_Master->write<uint16_t>("lifter_domain", "somanet_node", "ctrl_word", ctrlWord);
                    }

                    if(ethercat_interface::utilities::isBitSet(statusWord, 13)){
                        m_HomingHelper->homingErrorBit = true;
                    }
                    else{
                        m_HomingHelper->homingErrorBit = false;
                    }

                    if(ethercat_interface::utilities::isBitSet(statusWord, 12)){
                        m_HomingHelper->homingAttainedBit = true;
                    }
                    else{
                        m_HomingHelper->homingAttainedBit = false;
                    }   

                    if(ethercat_interface::utilities::isBitSet(statusWord, 10)){
                        m_HomingHelper->targetReachedBit = true;
                    }
                    else{
                        m_HomingHelper->targetReachedBit = false;
                    }

                    m_HomingHelper->previousCtrlWord = ctrlWord;
                }

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

    m_HomingHelper = std::make_shared<HomingHelper>();

    m_ControllerManager = std::make_shared<controller_manager::ControllerManager>(
        this,
        m_NodeHandle
    );

    m_CommInterface = std::make_unique<CommInterface>(m_CommConfigPath, m_HomingHelper);

    bool ec_ok = m_CommInterface->setup();
    if(!ec_ok){
        ros::shutdown();
    }

    ROS_INFO("EtherCAT communication is set up successfully.");

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
    
    m_HomingActionServer = std::make_unique<HomingActionServer>(
        m_NodeHandle,
        "lifter_homing_server",
        false
    );
    m_HomingActionServer->registerGoalCallback(boost::bind(&LifterHardwareInterface::execHomingCb, this));

    m_CommInterface->startTask();

    ROS_INFO("EtherCAT task started");

    m_HomingActionServer->start();
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

        if(m_HomingHelper->isHomingActive){
            
            if(m_HomingActionServer->isPreemptRequested()){
                m_HomingActionServer->setPreempted();
            }
            const auto currHomingStatus = m_HomingHelper->getCurrentHomingStatus();
            const auto found = HomingStatusStrings.find(currHomingStatus);
            
            switch (currHomingStatus)
            {
            case HomingStatus::HomingIsPerformed :
            case HomingStatus::HomingIsInterruptedOrNotStarted :
            case HomingStatus::HomingConfirmedTargetNotReached :
            {

                hamal_custom_interfaces::HomingOperationFeedback homingFb;
                homingFb.homingStatus = found->second;

                m_HomingActionServer->publishFeedback(homingFb);

                break;
            }
            case HomingStatus::HomingCompleted :
            {
                hamal_custom_interfaces::HomingOperationResult homingRes;
                homingRes.status = found->second;
                homingRes.homingDone = true;
                m_HomingActionServer->setSucceeded(homingRes);
                m_HomingHelper->isHomingActive = false;
                break;
            }
            case HomingStatus::ErrorDetectedMotorStillRunning :
            case HomingStatus::ErrorDuringHomingMotorAtStandstill :
            {
                hamal_custom_interfaces::HomingOperationResult homingRes;
                homingRes.status = found->second;
                homingRes.homingDone = false;
                m_HomingActionServer->setAborted(homingRes);
                m_HomingHelper->isHomingActive = false;
                break;
            }
            default:
                break;
            }

            
            
        }
        rate.sleep();
    }

    m_CommInterface->m_LoopFlag = false;
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
    if(m_HomingHelper->isHomingActive){

        return;
    }

    const auto lifterTargetVel = m_LifterJoint.targetVel;
    int32_t lifterTargetRPM = jointVelocityToMotorVelocity(lifterTargetVel);
/*     m_CommInterface->setData<int32_t>("somanet_node", "target_velocity", lifterTargetRPM);
 */
    m_CommInterface->setData<int32_t>("somanet_node", "target_velocity", 250);

 }

void LifterHardwareInterface::execHomingCb()
{
    const auto goal = m_HomingActionServer->acceptNewGoal();
    const auto operationInfo = goal->homingInfo;
    m_HomingHelper->switchSearchSpeed = operationInfo.switchSearchSpeed;
    m_HomingHelper->zeroSearchSpeed = operationInfo.zeroSearchSpeed;
    m_HomingHelper->homingAccel = operationInfo.homingAccel;
    m_HomingHelper->homingMethod = operationInfo.homingMethod;
    m_HomingHelper->isHomingActive = true;
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



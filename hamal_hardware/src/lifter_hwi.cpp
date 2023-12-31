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

PositionController::PositionController()
{

}

bool PositionController::calculateControlParams(
    const double& initial_position,
    const double& initial_velocity,
    const double& initial_acceleration,
    const double& target_position,
    const double& target_vel,
    const double& target_acc
)
{
    //m_ControlDataShMutex->lock_shared();
    //std::cout << "Initial Position: " << initial_position << " Initial Velocity: " << initial_velocity << " Initial acceleration" << initial_acceleration;
    //std::cout << '\n' << "Target Position: " << target_position << " Target Vel: " << target_vel << " Target accel: " << target_acc << std::endl; 
    double duration = fmax((15.0 * (fabs(target_position - initial_position))) / 
    (8.0 * m_Limits.velLimit), sqrt((10.0 * (fabs(target_position - initial_position))) / (m_Limits.accelLimit * sqrt(3.0))));
    double durationAlt = fmax((15.0 * fabs(target_position - initial_position)) / (8.0 * m_Limits.velLimit), sqrt((10.0 * fabs(target_position - initial_position)) / (m_Limits.accelLimit * sqrt(3.0))));
/*     duration = 28.0;
 *//*     duration = (2.0 * std::abs(target_position - initial_position) / m_Limits.velLimit);
 */    ROS_INFO("Target profile time: %f", duration);
    ROS_INFO("target profile time2: %f", durationAlt);
    m_MaxProfileTime = ros::Duration(duration);

    const double tf = duration;
    const double ti = 0.0;
    const double posDiff = target_position - initial_position;
    
    positionTracker = initial_position;
    m_Coeffs.a0 = initial_position;
    m_Coeffs.a1 = initial_velocity;
    m_Coeffs.a2 = initial_acceleration / 2.0;
    m_Coeffs.a3 = (10.0 * posDiff) / (tf*tf*tf);
    m_Coeffs.a4 = (-15.0 * posDiff) / (tf*tf*tf*tf);
    m_Coeffs.a5 = (6.0 * posDiff) / (tf*tf*tf*tf*tf);

    std::cout << "Coeffs: " << "a0: " << m_Coeffs.a0 << " a1: " << m_Coeffs.a1 << " a2: " << m_Coeffs.a2 << " a3: " << m_Coeffs.a3 << " a4: " << m_Coeffs.a4 << " a5: " << m_Coeffs.a5 << std::endl; 

    m_HardwareInfo.targetPosition = target_position;
    m_HardwareInfo.targetVelocity = target_vel;
    //m_ControlDataShMutex->unlock_shared();
    m_PositionPID.init(ros::Time::now());

    m_PrevPosTrajCmd = initial_position;

}

std::optional<Commands> PositionController::getCommands(double current_pos, double current_vel)
{
    const auto currTime = ros::Time::now();
/*     ROS_INFO("Current time: %f", currTime.toSec());
 */    
    if((currTime.toSec() - m_PreviousUpdateTime.toSec()) >= m_MaxProfileTime.toSec()){
        ROS_ERROR("Elapsed time is longer than the calculated maximum profile time: %f", (currTime.toSec() - m_PreviousUpdateTime.toSec()));

        std::stringstream ss;
        ss << currTime.sec << "." << currTime.nsec;
        std::cout << ss.str() << std::endl;
        m_PreviousUpdateTime = currTime;
        m_IsActive = false;
        positionTracker = 0.0;
        return std::nullopt;
    }
    /* else if(inRange<double>(current_pos, m_HardwareInfo.targetPosition, 0.5)){
        m_PreviousUpdateTime = currTime;
        return std::nullopt;
    } */
    const auto tc_alt = static_cast<double>(currTime.toSec()); // Current time in nanoseconds
    const auto tc = (currTime - m_PreviousUpdateTime).toSec();
/*     ROS_INFO("Current time: %f", tc);
 */    Commands newCmd;

    double posRef = m_Coeffs.a0 + (m_Coeffs.a1 * (tc)) + (m_Coeffs.a2 * (tc*tc)) + (m_Coeffs.a3 * (tc*tc*tc)) + (m_Coeffs.a4 * (tc*tc*tc*tc)) + (m_Coeffs.a5 * (tc*tc*tc*tc*tc)); 
    double posRefUnlimited = posRef;
    posRefUnlimited = posRefUnlimited - m_PrevPosTrajCmd;
    ROS_INFO("Traj pos cmd: %f", posRefUnlimited);
    double velRef = m_Coeffs.a1 + (2.0 * m_Coeffs.a2 *(tc)) + (3.0 * m_Coeffs.a3 * (tc*tc)) + (4.0 * m_Coeffs.a4 * (tc*tc*tc)) + (5.0 * m_Coeffs.a5 * (tc*tc*tc*tc));
    double accRef = (2.0 * m_Coeffs.a2) + (6.0 * m_Coeffs.a3 * (tc)) + (12.0 * m_Coeffs.a4 * (tc*tc)) + (20.0 * m_Coeffs.a5 * (tc*tc*tc));
    
    ROS_INFO("Traj vel cmd: %f", velRef * (60.0 / (M_PI * 2.0)) * 24.685);  
    newCmd.targetWithoutControl = velRef;
    
    const double pidOutput = m_PositionPID.pid(currTime, current_pos, posRef);
    velRef += pidOutput;
    
    ROS_INFO("PID output: %f", pidOutput);

    if(std::abs(posRefUnlimited) > m_Limits.posLimit){
        
        (posRefUnlimited < 0 ? posRefUnlimited = -1.0 * m_Limits.posLimit : posRefUnlimited = m_Limits.posLimit);
    }
    
    if(std::abs(velRef) > m_Limits.velLimit){
        
        (velRef < 0 ? velRef = -1.0 * m_Limits.velLimit : velRef = m_Limits.velLimit);
    }

    if(std::abs(accRef) > m_Limits.accelLimit){
        
        (accRef < 0 ? accRef = -1.0 * m_Limits.accelLimit : accRef = m_Limits.accelLimit );
    }
    newCmd.pos = posRefUnlimited;
    newCmd.vel = velRef;
    newCmd.acc = accRef;
    
    if(!(positionTracker >= m_HardwareInfo.targetPosition)){
        positionTracker += posRefUnlimited;
    }
    newCmd.setPoint = posRef;
    return newCmd;
}

void PositionController::PID::init(
    ros::Time init_time
)
{
    m_UpdateTime = init_time;
}

/* template<typename T>
T PositionController::PID::pid(const double& dt, T actual, T target)
{
    ROS_INFO("dT: %f", dt);

    const double error = target - actual;
    sumOfErrors += error * dt;
    const double errorRate = (error - prevError) / dt;
    double output = (Kp * error) + (Ki * sumOfErrors) + (Kd * errorRate);
    
    prevError = error;

    return output;
} */

template<typename T>
T PositionController::PID::pid(const ros::Time& current_time, T actual, T target)
{
    const double dt = current_time.toSec() - m_UpdateTime.toSec();
    const auto per = ros::Duration(current_time - m_UpdateTime);
    m_UpdateTime = current_time;
    
    const double error = target - actual;
    sumOfErrors += error * dt;
    const double errorRate = (error - prevError) / dt;
    double output = (Kp * error) + (Ki * sumOfErrors) + (Kd * errorRate);
    
    prevError = error;

    return output;
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
                /* else{
                    setData<int32_t>("somanet_node", "target_velocity", 0);
                } */
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
            ROS_INFO("Homing is active");

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
    
    m_LifterCommandActionServer = std::make_unique<LifterCommandActionServer>(
        m_NodeHandle,
        "lifter_command_server",
        false
    );  
    m_LifterCommandActionServer->registerGoalCallback(boost::bind(&LifterHardwareInterface::lifterCommandCb, this));

    m_HomingActionServer = std::make_unique<HomingActionServer>(
        m_NodeHandle,
        "lifter_homing_server",
        false
    );
    m_HomingActionServer->registerGoalCallback(boost::bind(&LifterHardwareInterface::execHomingCb, this));

    m_CommInterface->startTask();

    ROS_INFO("EtherCAT task started");

    m_ControlDataShMutex = std::make_shared<std::shared_mutex>();
    m_PositionController.m_ControlDataShMutex = m_ControlDataShMutex;

    m_LifterCommandActionServer->start();
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
            m_Reduction = 24.685;
        }

        if(m_NodeHandle.hasParam("/hamal/lifter_hardware_interface/position_increment"))
        {
            m_NodeHandle.getParam("/hamal/lifter_hardware_interface/position_increment", m_EncoderResolution);
        }
        else
        {
            m_Reduction = 20480.0;
        }

        double maxPos, maxVel, maxAcc = 0.0;
        if(m_NodeHandle.hasParam("/hamal/lifter_hardware_interface/max_position"))
        {
            m_NodeHandle.getParam("/hamal/lifter_hardware_interface/max_position", maxPos);
        }
        
        if(m_NodeHandle.hasParam("/hamal/lifter_hardware_interface/max_velocity"))
        {
            m_NodeHandle.getParam("/hamal/lifter_hardware_interface/max_velocity", maxVel);
        }
        
        if(m_NodeHandle.hasParam("/hamal/lifter_hardware_interface/max_accel"))
        {
            m_NodeHandle.getParam("/hamal/lifter_hardware_interface/max_accel", maxAcc);
        }

        double kp, ki, kd = 0.0;
        if(m_NodeHandle.hasParam("/hamal/lifter_hardware_interface/pid/Kp")){
            m_NodeHandle.getParam("/hamal/lifter_hardware_interface/pid/Kp", kp);
        }
        if(m_NodeHandle.hasParam("/hamal/lifter_hardware_interface/pid/Kp")){
            m_NodeHandle.getParam("/hamal/lifter_hardware_interface/pid/Ki", ki);
        }
        if(m_NodeHandle.hasParam("/hamal/lifter_hardware_interface/pid/Kp")){
            m_NodeHandle.getParam("/hamal/lifter_hardware_interface/pid/Kd", kd);
        }

        m_PositionController.setLimits(maxPos, maxVel, maxAcc);
        m_PositionController.m_PositionPID.setParams(kp, ki, kd);


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
        m_HardwareInfoMsg.current_vel = lifterVel.value();
    }  

    const auto lifterSlaveStateStrOpt =  m_CommInterface->getSlaveStateString("lifter_domain", "somanet_node");
    if(lifterSlaveStateStrOpt)
        m_HardwareInfoMsg.current_state = lifterSlaveStateStrOpt.value();

    m_HardwareInfoMsg.timestamp = ros::Time::now();
    

}

void LifterHardwareInterface::write()
{   
    if(m_HomingHelper->isHomingActive){

        return;
    }

    auto lifterTargetVel = m_LifterJoint.targetVel;
/*     ROS_INFO("Target Vel: %f", lifterTargetVel);
 */    
    if(m_PositionController.isActive()){

        /* if(inRange<double>(m_LifterJoint.currentPos, m_PositionController.getCurrentTarget(), 0.5)){
            hamal_custom_interfaces::LifterOperationResult lifterOpRes;
            lifterOpRes.target_reached = true;
            m_LifterCommandActionServer->setSucceeded(lifterOpRes);

        } */

        const auto cmds = m_PositionController.getCommands(m_LifterJoint.currentPos, m_LifterJoint.currentVel);
        if(cmds){
            m_HardwareInfoMsg.target_pos = cmds.value().pos;
            lifterTargetVel = cmds.value().vel;
            m_HardwareInfoMsg.target_without_control = (int32_t)cmds.value().targetWithoutControl * (60.0 / (M_PI * 2.0)) * 24.685;
            m_HardwareInfoMsg.setpoint = cmds.value().setPoint;
            hamal_custom_interfaces::LifterOperationFeedback lifterCmdFeedback;
            lifterCmdFeedback.current_position = m_LifterJoint.currentPos;
            lifterCmdFeedback.target_command = m_PositionController.getCurrentTarget();
            m_LifterCommandActionServer->publishFeedback(lifterCmdFeedback);
            //m_PositionController.m_PositionPID.reset();
        }
        else{
            
            hamal_custom_interfaces::LifterOperationResult lifterOpRes;
            lifterOpRes.target_reached = false;
            m_LifterCommandActionServer->setAborted(lifterOpRes);
            m_PositionController.m_PositionPID.reset();
        }
    }
    else{
    }
    
    int32_t lifterTargetRPM = lifterTargetVel * (60.0 / (M_PI * 2.0)) * 24.685;
    m_CommInterface->setData<int32_t>("somanet_node", "target_velocity", lifterTargetRPM);
    m_HardwareInfoMsg.target_vel = lifterTargetRPM;
    m_HardwareInfoPub.publish(m_HardwareInfoMsg);
    /* m_CommInterface->setData<int32_t>("somanet_node", "target_velocity", 250); */

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

void LifterHardwareInterface::lifterCommandCb()
{
    //m_ControlDataShMutex->lock();
    const auto goal = m_LifterCommandActionServer->acceptNewGoal();
    m_PositionController.calculateControlParams(
        m_LifterJoint.currentPos,
        m_LifterJoint.currentVel,
        m_LifterJoint.currentAcc,
        goal->target_position,
        0.0,
        0.0
    );
    //m_ControlDataShMutex->unlock();
    m_PositionController.setToActiveState();
    m_PositionController.updateUpdateTime();

    
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



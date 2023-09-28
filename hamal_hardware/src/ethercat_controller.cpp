/**
 * @file ethercat_controller.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hamal_hardware/ethercat_controller.hpp"

HamalEthercatController::HamalEthercatController(
    const std::string& config_file_path,
    std::shared_ptr<HomingHelper> homing_helper_ptr,
    bool enable_dc
)
    : 
        ethercat_interface::controller::Controller(config_file_path), 
        m_HomingHelperPtr(homing_helper_ptr), 
        m_EnableDC(enable_dc)
{
        /* m_LeftMotorLimiter = rpm_limiter();
        m_RightMotorLimiter = rpm_limiter(); */
    m_PrevUpdateTimePoint = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now());
}

HamalEthercatController::~HamalEthercatController()
{
    if(m_LeftMotorLimiter){
        delete m_LeftMotorLimiter;
    }
    if(m_RightMotorLimiter){
        delete m_RightMotorLimiter;
    }
    joinThread();
}

void HamalEthercatController::cyclicTask()
{
    if(m_EnableDC)
    {
        clock_gettime(m_DcHelper.clock, &m_DcHelper.wakeupTime);
    }
    while(m_EthercatLoopFlag)
    {

        if(m_EnableDC)
        {
            
            setTaskWakeUpTime();
            sleep_task(m_DcHelper.clock, TIMER_ABSTIME, &m_DcHelper.wakeupTime, NULL);

            m_Master->setMasterTime(timespecToNanoSec(m_DcHelper.wakeupTime));
        
        }

        m_Master->receive("domain_0");
        /* m_Master->updateDomainStates();
        m_Master->updateMasterState();
        m_Master->updateSlaveStates(); */
        
        bool slavesEnabled = m_Master->enableSlaves();

        int8_t lifterOpMode = [
                        homingActive = m_HomingHelperPtr->isHomingActive,
                        lifterControlType = this->m_LifterControlType
                        ]() -> int8_t {
            if(homingActive){
                return 0x06;
            }

            return ((lifterControlType == ControlType::Position) ? 0x08 : 0x09);
        }(); 


        m_Master->write<int8_t>(
            "domain_0",
            "somanet_node_0",
            "op_mode",
            lifterOpMode
        );
        
        if(const auto opModeDisplayOpt = m_Master->read<int8_t>("domain_0", "somanet_node_1", "op_mode_display")){
            if(opModeDisplayOpt.value() != 0x09)
                m_Master->write<int8_t>(
                    "domain_0",
                    "somanet_node_1",
                    "op_mode",
                    0x09
            );
        }
        if(const auto opModeDisplayOpt = m_Master->read<int8_t>("domain_0", "somanet_node_2", "op_mode_display")){
            if(opModeDisplayOpt.value() != 0x09)
                m_Master->write<int8_t>(
                    "domain_0",
                    "somanet_node_2",
                    "op_mode",
                    0x09
            );
        }

        if(m_HomingHelperPtr->isHomingActive && !m_HomingHelperPtr->isHomingSetupDone){
            m_Master->write<int32_t>("domain_0", "somanet_node_0", "target_velocity", 0);
            m_Master->write<int8_t>("domain_0", "somanet_node_0", "homing_method", m_HomingHelperPtr->homingMethod);
            m_Master->write<uint32_t>("domain_0", "somanet_node_0", "homing_speed", m_HomingHelperPtr->switchSearchSpeed);
            m_Master->write<uint32_t>("domain_0", "somanet_node_0", "homing_speed2", m_HomingHelperPtr->zeroSearchSpeed);
            m_Master->write<uint32_t>("domain_0", "somanet_node_0", "homing_accel", m_HomingHelperPtr->homingAccel);
            
            m_HomingHelperPtr->isHomingSetupDone = true;
            m_HomingHelperPtr->isHomingInProgress = true;
        }

        bool opModeSetCorrect = false;
        const auto opModeDisplayOpt = m_Master->read<int8_t>("domain_0", "somanet_node_0", "op_mode_display");
        if(opModeDisplayOpt){
            if(opModeDisplayOpt.value() == 0x06){
                opModeSetCorrect = true;
            }
        }
        // slavesEnabled = m_Master->enableSlaves();

        /* const auto current_time = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now());
        const auto elapsed = (m_PrevUpdateTimePoint - current_time).count() / 1000.0;
        m_PrevUpdateTimePoint = current_time; */

        if(slavesEnabled)
        {

            auto leftMotorPosOpt = m_Master->read<int32_t>("domain_0", "somanet_node_2", "actual_position");
            auto rightMotorPosOpt = m_Master->read<int32_t>("domain_0", "somanet_node_1", "actual_position");
            auto lifterPosOpt = m_Master->read<int32_t>("domain_0", "somanet_node_0", "actual_position");

            if(leftMotorPosOpt){
                setData<int32_t>("somanet_node_2", "actual_position", leftMotorPosOpt.value());
            }
            if(rightMotorPosOpt){
                setData<int32_t>("somanet_node_1", "actual_position", rightMotorPosOpt.value());
            }
            if(lifterPosOpt){
                setData<int32_t>("somanet_node_0", "actual_position", lifterPosOpt.value());
            }

            auto leftMotorVelOpt = m_Master->read<int32_t>("domain_0", "somanet_node_2", "actual_velocity");
            auto rightMotorVelOpt = m_Master->read<int32_t>("domain_0", "somanet_node_1", "actual_velocity");
            auto lifterVelOpt = m_Master->read<int32_t>("domain_0", "somanet_node_0", "actual_velocity");

            if(leftMotorVelOpt){
                setData<int32_t>("somanet_node_2", "actual_velocity", leftMotorVelOpt.value());
            }
            if(rightMotorVelOpt){
                setData<int32_t>("somanet_node_1", "actual_velocity", rightMotorVelOpt.value());
            }
            if(lifterVelOpt){
                setData<int32_t>("somanet_node_0", "actual_velocity", lifterVelOpt.value());
            }
        }
        // slavesEnabled = m_Master->enableSlaves();

        if(slavesEnabled && !m_HomingHelperPtr->isHomingActive)
        {
            auto leftTargetVelOpt = getData<int32_t>("somanet_node_2", "target_velocity");
            auto rightTargetVelOpt = getData<int32_t>("somanet_node_1", "target_velocity");
            auto lifterTargetVelOpt = getData<int32_t>("somanet_node_0", "target_velocity");
            if(leftTargetVelOpt != std::nullopt && rightTargetVelOpt && lifterTargetVelOpt)
            {

                double leftTargetVel = leftTargetVelOpt.value();
                double rightTargetVel = rightTargetVelOpt.value();
                m_LeftMotorLimiter->limit(leftTargetVel, 0.002);
                m_RightMotorLimiter->limit(rightTargetVel, 0.002);
/*                 std::cout << "Left Motor RPM:" << leftTargetVel << "Right Motor RPM" << rightTargetVel << std::endl; 
 */                m_Master->write<int32_t>("domain_0", "somanet_node_1", "target_velocity", rightTargetVel);
                m_Master->write<int32_t>("domain_0", "somanet_node_2", "target_velocity", leftTargetVel * -1);
                m_Master->write<int32_t>("domain_0", "somanet_node_0", "target_velocity", lifterTargetVelOpt.value());
            }
           
        }
        else if(slavesEnabled && m_HomingHelperPtr->isHomingActive && m_HomingHelperPtr->isHomingInProgress && opModeSetCorrect){
            const auto ctrlWordOpt = m_Master->getControlWord("domain_0", "somanet_node_0");
            const auto statusWordOpt = this->m_Master->read<uint16_t>("domain_0", "somanet_node_0", "status_word");
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
                if(m_HomingHelperPtr->previousCtrlWord != ctrlWord){
                    m_Master->write<uint16_t>("domain_0", "somanet_node_0", "ctrl_word", ctrlWord);
                }
                if(ethercat_interface::utilities::isBitSet(statusWord, 13)){
                    m_HomingHelperPtr->homingErrorBit = true;
                }
                else{
                    m_HomingHelperPtr->homingErrorBit = false;
                }
                if(ethercat_interface::utilities::isBitSet(statusWord, 12)){
                    m_HomingHelperPtr->homingAttainedBit = true;
                }
                else{
                    m_HomingHelperPtr->homingAttainedBit = false;
                }   
                if(ethercat_interface::utilities::isBitSet(statusWord, 10)){
                    m_HomingHelperPtr->targetReachedBit = true;
                }
                else{
                    m_HomingHelperPtr->targetReachedBit = false;
                }
                m_HomingHelperPtr->previousCtrlWord = ctrlWord;

                std::cout << "Homing in progress\n";
            }
        }
        else if(!slavesEnabled){
            /* m_Master->enableSlaves(); */
            /* m_Master->write("domain_0", "somanet_node_1", "target_velocity", 0);
            m_Master->write("domain_0", "somanet_node_2", "target_velocity", 0);
            m_Master->write("domain_0", "somanet_node_0", "target_velocity", 0); */
        }

        if(m_EnableDC)
        {
            clock_gettime(m_DcHelper.clock, &m_DcHelper.currentTime);
            m_Master->syncMasterClock(timespecToNanoSec(m_DcHelper.currentTime), m_DcHelper);
        }

        m_Master->send("domain_0");
        m_EthercatOk = slavesEnabled;
    }
}
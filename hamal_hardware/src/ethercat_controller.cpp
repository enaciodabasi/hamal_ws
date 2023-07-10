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
    bool enable_dc
)
    : ethercat_interface::controller::Controller(config_file_path), m_EnableDC(enable_dc), m_LifterControlType(ControlType::Velocity)
{
    
}

HamalEthercatController::~HamalEthercatController()
{
    joinThread();
}

void HamalEthercatController::cyclicTask()
{
    if(m_EnableDC)
    {
        std::cout << "DC enabled\n";
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
        m_Master->updateDomainStates();
        m_Master->updateMasterState();
        m_Master->updateSlaveStates();
        
        bool slavesEnabled = m_Master->enableSlaves();

        int8_t opMode = 0x09; 

        if(m_LifterControlType == ControlType::Velocity)
        {
            opMode = 0x09;
        }
        else if(m_LifterControlType == ControlType::Position)
        {
            opMode = 0x08;
        }

        if(slavesEnabled)
        {
            m_Master->write<int8_t>("domain_0", "somanet_node", "op_mode", opMode);
            auto lifterPosition = m_Master->read<int32_t>("domain_0", "somanet_node", "actual_position");
            if(lifterPosition != std::nullopt)
            {
                setData<int32_t>("somanet_node", "actual_position", lifterPosition.value());
                /* std::cout << "Actual Position: " << lifterPosition.value() << std::endl; */
            }

            auto lifterVelOpt = m_Master->read<int32_t>("domain_0", "somanet_node", "actual_velocity");
            
            if(lifterVelOpt != std::nullopt)
            {
                setData<int32_t>("somanet_node", "actual_velocity", lifterVelOpt.value());
                /* std::cout << "Actual velocity: " << lifterVelOpt.value() << std::endl; */
            } 
        }

        
        if(slavesEnabled)
        {   
            std::optional<int32_t> lifterTargetValue = std::nullopt;

            if(m_LifterControlType == ControlType::Velocity)
                lifterTargetValue = getData<int32_t>("somanet_node", "target_velocity");
            else if(m_LifterControlType == ControlType::Position)
                lifterTargetValue = getData<int32_t>("somanet_node", "target_position");

                        
            if(lifterTargetValue != std::nullopt)
            {
                /* std::cout << "Target Value: " << lifterTargetValue.value(); */
                if(m_LifterControlType == ControlType::Velocity)
                    m_Master->write("domain_0", "somanet_node", "target_velocity", lifterTargetValue.value());
                else if(m_LifterControlType == ControlType::Position)
                    m_Master->write("domain_0", "somanet_node", "target_position", lifterTargetValue.value());
            }
        } 


        /* m_Master->write<int8_t>(
                    "domain_0",
                    "somanet_node_0",
                    "op_mode",
                    0x09
        );
        m_Master->write<int8_t>(
            "domain_0",
            "somanet_node_1",
            "op_mode",
            0x09
        );

        if(slavesEnabled)
        {

            auto leftMotorPosOpt = m_Master->read<int32_t>("domain_0", "somanet_node_0", "actual_position");
            auto rightMotorPosOpt = m_Master->read<int32_t>("domain_0", "somanet_node_1", "actual_position");

            if(leftMotorPosOpt != std::nullopt && rightMotorPosOpt != std::nullopt)
            {
                setData<int32_t>("somanet_node_0", "actual_position", leftMotorPosOpt.value());
                setData<int32_t>("somanet_node_1", "actual_position", rightMotorPosOpt.value());
            }

            auto leftMotorVelOpt = m_Master->read<int32_t>("domain_0", "somanet_node_0", "actual_velocity");
            auto rightMotorVelOpt = m_Master->read<int32_t>("domain_0", "somanet_node_1", "actual_velocity");
            if(leftMotorVelOpt != std::nullopt && rightMotorVelOpt != std::nullopt)
            {
                setData<int32_t>("somanet_node_0", "actual_velocity", leftMotorVelOpt.value());
                setData<int32_t>("somanet_node_1", "actual_velocity", rightMotorVelOpt.value());
            }   
        }

        if(slavesEnabled)
        {
            auto leftTargetVelOpt = getData<int32_t>("somanet_node_0", "target_velocity");
            auto rightTargetVelOpt = getData<int32_t>("EL7221_9015_1", "target_velocity");

            if(leftTargetVelOpt != std::nullopt && rightTargetVelOpt != std::nullopt)
            {
                m_Master->write("domain_0", "somanet_node_0", "target_velocity", leftTargetVelOpt.value());
                m_Master->write("domain_0", "somanet_node_1", "target_velocity", rightTargetVelOpt.value());
            }
        } */

        if(m_EnableDC)
        {
            clock_gettime(m_DcHelper.clock, &m_DcHelper.currentTime);
            m_Master->syncMasterClock(timespecToNanoSec(m_DcHelper.currentTime), m_DcHelper);
        }

        m_Master->send("domain_0");
    }
}
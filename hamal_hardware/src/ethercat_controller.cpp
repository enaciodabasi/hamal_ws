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
    : ethercat_interface::controller::Controller(config_file_path), m_EnableDC(enable_dc)
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


        m_Master->write<int8_t>(
                    "domain_0",
                    "somanet_node",
                    "op_mode",
                    0x09
        );
        /* m_Master->write<int8_t>(
            "domain_0",
            "somanet_node_1",
            "op_mode",
            0x09
        ); */

        if(slavesEnabled)
        {

            auto lifterPositionOpt = m_Master->read<int32_t>("domain_0", "somanet_node", "actual_position");
/*             auto rightMotorPosOpt = m_Master->read<int32_t>("domain_0", "somanet_node_1", "actual_position");
 */
            if(lifterPositionOpt != std::nullopt/*  && rightMotorPosOpt != std::nullopt */)
            {
                /* std::cout << "Motor position: " << leftMotorPosOpt.value() << std::endl; */
                setData<int32_t>("somanet_node", "actual_position", lifterPositionOpt.value());
/*                 setData<int32_t>("somanet_node_1", "actual_position", rightMotorPosOpt.value());
 */            }

            auto lifterVelOpt = m_Master->read<int32_t>("domain_0", "somanet_node", "actual_velocity");
/*             auto rightMotorVelOpt = m_Master->read<int32_t>("domain_0", "somanet_node_1", "actual_velocity");
 */            if(lifterVelOpt != std::nullopt /* && rightMotorVelOpt != std::nullopt */)
            {
                setData<int32_t>("somanet_node", "actual_velocity", lifterVelOpt.value());
                /* std::cout << "Motor velocity: " << leftMotorVelOpt.value() << std::endl; */
                /* setData<int32_t>("somanet_node_1", "actual_velocity", rightMotorVelOpt.value()); */
            }   
        }

        if(slavesEnabled)
        {
            auto lifterTargetVelOpt = getData<int32_t>("somanet_node", "target_velocity");
/*             auto rightTargetVelOpt = getData<int32_t>("somanet_node_1", "target_velocity");
 */
            if(lifterTargetVelOpt != std::nullopt)
            {
                m_Master->write("domain_0", "somanet_node", "target_velocity", lifterTargetVelOpt.value());
                /* m_Master->write("domain_0", "somanet_node_1", "target_velocity", rightTargetVelOpt.value()); */
            }
        }

        if(m_EnableDC)
        {
            clock_gettime(m_DcHelper.clock, &m_DcHelper.currentTime);
            m_Master->syncMasterClock(timespecToNanoSec(m_DcHelper.currentTime), m_DcHelper);
        }

        m_Master->send("domain_0");
    }
}
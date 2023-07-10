/**
 * @file ethercat_controller.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef ETHERCAT_CONTROLLER_HPP_
#define ETHERCAT_CONTROLLER_HPP_

#include "ethercat_interface/controller.hpp"

#include "hamal_hardware/hamal_hardware_defs.hpp"

class HamalEthercatController : public ethercat_interface::controller::Controller
{
    public:

    HamalEthercatController(
        const std::string& config_file_path,
        bool enable_dc = true
    );

    ~HamalEthercatController();

    bool m_EthercatLoopFlag = true;

    inline void setLifterControlType(const ControlType& control_type)
    {
        m_LifterControlType = control_type;
    }

    void stopEcThread()
    {
        m_EthercatLoopFlag = false;
    }

    void startTask() override
    {
        m_CyclicTaskThread = std::thread(
            &HamalEthercatController::cyclicTask,
            this
        );
        this->updateThreadInfo();
    }

    private:

    ControlType m_LifterControlType;

    bool m_EnableDC = false;
    
    

    void cyclicTask() override;

};


#endif
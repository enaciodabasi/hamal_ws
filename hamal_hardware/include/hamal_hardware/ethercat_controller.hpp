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
#include "hamal_hardware/homing_helper.hpp"
/* #include "hamal_hardware/hamal_hardware_defs.hpp"
 */
class HamalEthercatController : public ethercat_interface::controller::Controller
{
    public:

    HamalEthercatController(
        const std::string& config_file_path,
        std::shared_ptr<HomingHelper> homing_helper_ptr,
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
        this->setThreadParams(SCHED_FIFO, 99);
        m_CyclicTaskThread = std::thread(
            &HamalEthercatController::cyclicTask,
            this
        );
        this->updateThreadInfo();
    }

    private:

    ControlType m_LifterControlType;

    std::shared_ptr<HomingHelper> m_HomingHelperPtr;

    bool m_EnableDC = true;

    void cyclicTask() override;

};


#endif
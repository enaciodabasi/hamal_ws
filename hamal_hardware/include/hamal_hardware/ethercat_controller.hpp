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

#include "ethercat_interface/controller.hpp"

#ifndef ETHERCAT_CONTROLLER_HPP_
#define ETHERCAT_CONTROLLER_HPP_


class HamalEthercatController : public ethercat_interface::controller::Controller
{
    public:

    HamalEthercatController(
        const std::string& config_file_path,
        bool enable_dc = true
    );

    ~HamalEthercatController();

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

    bool m_EnableDC = true;
    
    bool m_EthercatLoopFlag = true;

    void cyclicTask() override;

};


#endif
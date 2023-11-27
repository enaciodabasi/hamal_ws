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
#include "hamal_hardware/rpm_limiter.hpp"
#include <string>
#include <vector>
#include <chrono>
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

    const std::vector<std::pair<std::string, std::string>> getSlaveStatus() const
    {
        const std::string lifterStatus = m_Master->getSlaveStateString("domain_0", "somanet_node_0").value();
        const std::string leftWheelStatus = m_Master->getSlaveStateString("domain_0", "somanet_node_1").value();
        const std::string rightWheelStatus = m_Master->getSlaveStateString("domain_0", "somanet_node_2").value();

        
        return {
            {"lifter_joint", lifterStatus},
            {"left_wheel_joint", leftWheelStatus},
            {"right_wheel_joint", rightWheelStatus}
        };

    }

    inline void setLifterControlType(const ControlType& control_type)
    {
        m_LifterControlType = control_type;
    }
    inline const bool isEthercatOk() const
    {
        return m_EthercatOk;
    }
    void stopEcThread()
    {
        m_EthercatLoopFlag = false;
    }

    void setLimiterParams(
        double max_vel,
        double min_vel,
        double max_acc,
        double min_acc
    )
    {
        m_LeftMotorLimiter = new rpm_limiter(max_vel, min_vel, max_acc, min_acc);
        m_RightMotorLimiter = new rpm_limiter(max_vel, min_vel, max_acc, min_acc);
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

    void setQuickStop(){
        m_ActivateQuickStop = true;
    }

    void deactivateQuickStop()
    {
        m_ActivateQuickStop = false;
    }

    const bool isQuickStopActive() const
    {
        return m_ActivateQuickStop;
    }
    

    private:

    ControlType m_LifterControlType;

    std::chrono::time_point<std::chrono::steady_clock, std::chrono::milliseconds> m_PrevUpdateTimePoint;

    rpm_limiter* m_LeftMotorLimiter;

    rpm_limiter* m_RightMotorLimiter;

    std::shared_ptr<HomingHelper> m_HomingHelperPtr;

    bool m_EnableDC = true;

    bool m_EthercatOk = false;

    bool m_ActivateQuickStop = false;

    void cyclicTask() override;

    bool activateQuickStop();

};


#endif
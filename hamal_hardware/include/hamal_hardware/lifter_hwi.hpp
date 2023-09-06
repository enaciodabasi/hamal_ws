/**
 * @file lifter_hwi.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-09-06
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef LIFTER_HWI_
#define LIFTER_HWI_


#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_manager/controller_manager.h>
#include <hamal_custom_interfaces/HardwareInfo.h>

#include <iostream>
#include <memory>
#include <unordered_map>
#include <optional>
#include <thread>
#include <string>

#include "ethercat_interface/controller.hpp"

class CommInterface : public ethercat_interface::controller::Controller
{
    public:

    CommInterface(const std::string& path_to_config_file);

    ~CommInterface()
    {
        joinThread();   
    }

    bool m_LoopFlag = true;

    void stopEcThread()
    {
        m_LoopFlag = false;
    }

    void startTask() override
    {
        setThreadParams(SCHED_FIFO, 99);
        m_CyclicTaskThread = std::thread(
            &CommInterface::cyclicTask,
            this
        );
        this->updateThreadInfo();
    }

    private:

    void cyclicTask() override;
};

class LifterHardwareInterface : public hardware_interface::RobotHW
{
    public:

    LifterHardwareInterface(ros::NodeHandle& nh);

    ~LifterHardwareInterface();

    void update();

    private:

    ros::NodeHandle m_NodeHandle;

    ros::Publisher m_HardwareInfoPub;
    hamal_custom_interfaces::HardwareInfo m_HardwareInfoMsg;

    std::unique_ptr<CommInterface> m_CommInterface;

    std::shared_ptr<controller_manager::ControllerManager> m_ControllerManager;
    
    struct{
        std::string m_LifterJointName;
        double currentPos;
        double currentVel;
        double currentAcc;
        double targetPos;
        double targetVel;
        double targetAcc;
    } m_LifterJoint;

    std::string m_CommConfigPath;    

    hardware_interface::JointStateInterface m_JointStateInterface;

    hardware_interface::PosVelAccJointInterface m_LifterJointInterface;

    double m_LoopFreq = 50.0;

    bool m_RosLoopFlag = true;

    double m_Reduction = 0.0;
    
    double m_EncoderResolution = 0.0;

    void configure();

    void read();

    void write();

    inline const double motorPositionToJointPosition(const int32_t& motor_position)
    {
        return (double)(motor_position / m_EncoderResolution) * (2.0 * M_PI) / m_Reduction;
    }
    
    /**
     * @brief 
     * 
     * @param joint_position 
     * @return const int32_t 
     */
    inline const int32_t jointPositionToMotorPosition(const double& joint_position)
    {
        return (int32_t)((joint_position * m_EncoderResolution * m_Reduction) / (2.0 * M_PI));
    }
    /**
     * @brief 
     * 
     * @param motor_velocity 
     * @return const double 
     */
    inline const double motorVelocityToJointVelocity(const int32_t& motor_velocity)
    {
        const double currentVel = ((double)motor_velocity * 2.0) / (60.0 * M_PI * m_Reduction);
        return currentVel;
    }
    /**
     * @brief 
     * 
     * @param joint_velocity 
     * @return const int32_t 
     */
    inline const int32_t jointVelocityToMotorVelocity(const double& joint_velocity)
    {
        int32_t targetVel = ((60 * joint_velocity) / 2 * M_PI) * m_Reduction; 
        targetVel = joint_velocity *((60.0/M_PI*2.0)) * m_Reduction;
        return targetVel;
    }

};


#endif // LIFTER_HWI_
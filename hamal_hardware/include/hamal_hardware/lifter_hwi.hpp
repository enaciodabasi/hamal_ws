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

#include <boost/bind/bind.hpp>
#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_manager/controller_manager.h>
#include <actionlib/server/simple_action_server.h>
#include <hamal_custom_interfaces/HardwareInfo.h>
#include <hamal_custom_interfaces/HomingInfo.h>
#include <hamal_custom_interfaces/HomingOperationAction.h>
#include <hamal_custom_interfaces/LifterOperationAction.h>

#include <iostream>
#include <memory>
#include <unordered_map>
#include <optional>
#include <thread>
#include <string>

#include "ethercat_interface/controller.hpp"

using LifterCommandActionServer = actionlib::SimpleActionServer<hamal_custom_interfaces::LifterOperationAction>;
using HomingActionServer = actionlib::SimpleActionServer<hamal_custom_interfaces::HomingOperationAction>;

enum class HomingStatus
{
    Unknown,
    HomingIsPerformed,
    HomingIsInterruptedOrNotStarted,
    HomingConfirmedTargetNotReached,
    HomingCompleted,
    ErrorDetectedMotorStillRunning,
    ErrorDuringHomingMotorAtStandstill
};

const std::unordered_map<HomingStatus, std::string> HomingStatusStrings = {
    {HomingStatus::Unknown, "Unknown"},
    {HomingStatus::HomingIsPerformed, "Homing is performed"},
    {HomingStatus::HomingIsInterruptedOrNotStarted, "Homing is interrupted or not yet started"},
    {HomingStatus::HomingConfirmedTargetNotReached, "Homing confirmed but target not yet reached"},
    {HomingStatus::HomingCompleted, "Homing completed"},
    {HomingStatus::ErrorDetectedMotorStillRunning, "Error detected, motor still running"},
    {HomingStatus::ErrorDuringHomingMotorAtStandstill, "Error during homing, motor at standstill"}
};

struct HomingHelper
{
    bool isHomingActive;
    bool isHomingInProgress;
    bool isHomingSetupDone;
    
    int8_t homingMethod;
    int32_t switchSearchSpeed;
    int32_t zeroSearchSpeed;
    int32_t homingAccel;
    
    uint16_t previousCtrlWord;

    bool homingErrorBit = false;
    bool homingAttainedBit = false;
    bool targetReachedBit = false;

    HomingHelper()
    {
        isHomingActive = false;
        isHomingInProgress = false;
        isHomingSetupDone = false;
        switchSearchSpeed = 0;
        homingMethod = 0x11;
        zeroSearchSpeed = 0;
        homingAccel = 0;
        previousCtrlWord = 0x0;
    }

    const HomingStatus getCurrentHomingStatus();

};

struct Commands
{
    double pos;
    
    double vel;
    
    double acc;

    double targetWithoutControl;

    Commands() : pos(0.0), vel(0.0), acc(0.0) {}
    Commands(double pos, double vel, double acc, double targetWithoutControl)
    {
        this->pos = pos;
        this->vel = vel;
        this->acc = acc;
        this->targetWithoutControl = targetWithoutControl;
    }

};


class PositionController
{
    public:
    
    PositionController();

    void setLimits(double pos_limit, double vel_limit, double accel_limit)
    {
        m_Limits.posLimit = pos_limit;
        m_Limits.velLimit = vel_limit;
        m_Limits.accelLimit = accel_limit;
    }

    bool calculateControlParams(
        const double& initial_position,
        const double& initial_velocity,
        const double& initial_acceleration,
        const double& target_position,
        const double& target_vel = 0.0,
        const double& target_acc = 0.0
    );

    std::optional<Commands> getCommands(double current_pos, double current_vel);    

    void updateUpdateTime(){
        m_PreviousUpdateTime = ros::Time::now();
    }

    double getCurrentTarget()
    {
        return m_HardwareInfo.targetPosition;
    }

    void setToActiveState()
    {
        m_IsActive = true;
    }

    bool isActive(){
        return m_IsActive;
    }
    private:

    struct PID
    {

        double Kp;
        double Ki;
        double Kd;

        double prevError;
        double sumOfErrors;

        ros::Time updateTime;

        void setParams(
            const double kp,
            const double ki,
            const double kd
        )
        {
            Kp = kp;
            Ki = ki;
            Kd = kd;
        }

        template<typename T>
        T pid(ros::Time current_time, T actual, T target);

        void init(
            ros::Time init_time
        );

        void reset()
        {
            prevError = 0.0;
            sumOfErrors = 0.0;
            updateTime = ros::Time(0.0);
        }

        PID()
        {
            Kp = 0;
            Ki = 0;
            Kd = 0;
            updateTime = ros::Time(0.0);
        }
    };

    public:
    
    PID m_PositionPID;

    private:

    bool m_IsActive = false;

    struct{
        double actualPosition = 0.0;
        double actualVelocity = 0.0;
        double targetPosition = 0.0; 
        double targetVelocity = 0.0;
    } m_HardwareInfo;

    double positionTracker = 0.0;
    

    struct{ 
        double posLimit = 0.0;
        double velLimit = 0.0;
        double accelLimit = 0.0;
    } m_Limits;

    struct{
        double a0 = 0.0;
        double a1 = 0.0;
        double a2 = 0.0;
        double a3 = 0.0;
        double a4 = 0.0;
        double a5 = 0.0;
    } m_Coeffs;
    
    ros::Time m_PreviousUpdateTime;

    ros::Duration m_MaxProfileTime;
    

};

class CommInterface : public ethercat_interface::controller::Controller
{
    public:

    CommInterface(const std::string& path_to_config_file, std::shared_ptr<HomingHelper>& homing_helper_ptr);

    ~CommInterface()
    {
        joinThread();   
    }

    bool m_LoopFlag = true;

    void stopEcThread()
    {
        m_LoopFlag = false;
    }

    std::shared_ptr<HomingHelper> m_HomingHelper;

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

    std::unique_ptr<LifterCommandActionServer> m_LifterCommandActionServer;

    std::unique_ptr<HomingActionServer> m_HomingActionServer;

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

    std::shared_ptr<HomingHelper> m_HomingHelper;

    PositionController m_PositionController;

    double m_LoopFreq = 50.0;

    bool m_RosLoopFlag = true;

    bool m_IsHomingModeActive = false;

    double m_Reduction = 0.0;
    
    double m_EncoderResolution = 0.0;

    void configure();

    void read();

    void write();

    void execHomingCb();

    void lifterCommandCb();

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
        targetVel = joint_velocity * (((60.0/M_PI*2.0)) * 24.685);
        std::cout << "Target Motor Vel: " << targetVel << std::endl;
        return targetVel;
    }

};


#endif // LIFTER_HWI_
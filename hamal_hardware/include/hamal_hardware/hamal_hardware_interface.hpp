/**
 * @file hamal_hardware_interface.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef HAMAL_HARDWARE_INTERFACE_
#define HAMAL_HARDWARE_INTERFACE_

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <controller_manager/controller_manager.h>

#include <unordered_map>
#include <optional>
#include <memory>
#include <variant>

#include "hamal_hardware/ethercat_controller.hpp"

namespace hamal
{
/*     using LifterInterface = std::variant<hardware_interface::PositionJointInterface, hardware_interface::VelocityJointInterface>;
 */
    enum LifterInterfaceType
    {
        Position,
        Velocity
    };

    struct JointHandle
    {
        JointHandle(){}
        JointHandle(
            const std::string& name
        )
        {
            jointName = name;
            position = 0.0;
            velocity = 0.0;
            effort = 0.0;
            /* targetVelocity = 0.0; */
            targetPosition = 0.0;
        }

        std::string jointName;
        double position;
        double velocity;
        double effort;
        
        double targetVelocity;
        double targetPosition;
        double targetAccel;
    };

    class HardwareInterface : public hardware_interface::RobotHW
    {
        public:

        /**
         * @brief Construct a new Hardware Interface object
         * 
         * @param nh 
         */
        HardwareInterface(ros::NodeHandle& nh);

        /**
         * @brief 
         * 
         */
        ~HardwareInterface();
        
        /**
         * @brief 
         * 
         */
        void update();

        private:

        /**
         * @brief 
         * 
         */
        ros::NodeHandle m_NodeHandle;

        std::shared_ptr<controller_manager::ControllerManager> m_ControllerManager;

        std::unique_ptr<HamalEthercatController> m_EthercatController;
        std::string m_EthercatConfigFilePath = "";
        /**
         * @brief 
         * 
         */
        std::unordered_map<std::string, JointHandle> m_JointsMap;

        std::string m_LeftWheelJointName;
        std::string m_RightWheelJointName;
        std::string m_LifterJointName;

        hardware_interface::JointStateInterface m_JointStateInterface;

        hardware_interface::VelocityJointInterface m_VelJointInterface;

        hardware_interface::PosVelAccJointInterface m_LifterJointInterface;

        /* std::optional<hardware_interface::PositionJointInterface> m_LifterPositionInterface;

        std::string m_LifterJointName = "";

        LifterInterfaceType m_LifterInterfaceType; */
        
        double m_LoopFrequency = 50.0;

        bool m_RosLoopFlag = true;

        double m_Reduction = 0.0;

        double m_Increment = 0.0;

        /**
         * @brief 
         * 
         */
        void configure();

        void read();

        void write();

        /**
         * @brief Turns motor position [increments] coming from EtherCAT to joint position [rad].
         * 
         * @param motor_position Motor position in incrementsç
         * @return const double: Joint position in radians. 
         */
        inline const double motorPositionToJointPosition(const int32_t& motor_position)
        {
            return (double)(motor_position / m_Increment) * (2.0 * M_PI) / m_Reduction;
        }
        
        /**
         * @brief 
         * 
         * @param joint_position 
         * @return const int32_t 
         */
        inline const int32_t jointPositionToMotorPosition(const double& joint_position)
        {
            return (int32_t)((joint_position * m_Increment * m_Reduction) / (2.0 * M_PI));
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
}

#endif

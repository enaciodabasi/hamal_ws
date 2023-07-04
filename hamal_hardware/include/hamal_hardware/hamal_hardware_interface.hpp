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
#include <controller_manager/controller_manager.h>

#include <unordered_map>
#include <optional>
#include <memory>

#include "hamal_hardware/ethercat_controller.hpp"

namespace hamal
{

    struct WheelJointHandle
    {
        WheelJointHandle(){}
        WheelJointHandle(
            const std::string& name
        )
        {
            jointName = name;
            position = 0.0;
            velocity = 0.0;
            effort = 0.0;
            targetVelocity = 0.0;
        }

        std::string jointName;
        double position;
        double velocity;
        double effort;
        
        double targetVelocity;
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
        std::unordered_map<std::string, WheelJointHandle> m_WheelJointsMap;

        hardware_interface::JointStateInterface m_JointStateInterface;

        hardware_interface::VelocityJointInterface m_VelJointInterface;
        
        double m_LoopFrequency = 50.0;

        bool m_RosLoopFlag = true;

        /**
         * @brief 
         * 
         */
        void configure();

        void read();

        void write();

    };
}

#endif

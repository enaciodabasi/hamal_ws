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
#include <unordered_map>

#include "hamal_hardware/ethercat_controller.hpp"

namespace hamal
{

    struct WheelJointHandle
    {
        std::string name;
        double position;
        double velocity;
        double effort;
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

        private:

        /**
         * @brief 
         * 
         */
        ros::NodeHandle m_NodeHandle;

        std::unordered_map<std::string, WheelJointHandle> m_WheelJointsMap;
        
        /**
         * @brief 
         * 
         */
        void configure();

    };
}

#endif

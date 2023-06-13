/**
 * @file hamal_hardware_interface.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hamal_hardware/hamal_hardware_interface.hpp"

namespace hamal
{
    HardwareInterface::HardwareInterface(ros::NodeHandle& nh)
    {
        configure();
    }

    HardwareInterface::~HardwareInterface()
    {

    }

    void HardwareInterface::configure()
    {

    }
}
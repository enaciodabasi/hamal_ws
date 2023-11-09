/**
 * @file emg_stop_client.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef EMG_STOP_CLIENT_
#define EMG_STOP_CLIENT_

#include <ros/ros.h>

#include "hamal_custom_interfaces/EmergencyStop.h"

using EmgClient = hamal_custom_interfaces::EmergencyStop;

/**
 * @brief Base class for interacting with the Emergency Stop server inside the hamal_hardware. 
 * 
 */
class EmgClientBase
{  
    public:

    EmgClientBase(ros::NodeHandle& nh);

    virtual ~EmgClientBase();

    virtual bool sendEmg();

    private:

    ros::ServiceClient m_ServiceClient;

};

#endif // EMG_STOP_CLIENT_
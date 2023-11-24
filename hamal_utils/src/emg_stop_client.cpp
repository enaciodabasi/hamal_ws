/**
 * @file emg_stop_client.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hamal_utils/emg_stop_client.hpp"

EmgClientBase::EmgClientBase(ros::NodeHandle& nh)
{
    m_ServiceClient = nh.serviceClient<EmgClient>("emg_stop_server");
}

EmgClientBase::~EmgClientBase()
{
    
}

bool EmgClientBase::sendEmg()
{   
    if(!m_ServiceClient.exists()){
        return false;
    }

    EmgClient srv;
    srv.request.trigger = std_msgs::Empty();
    if(m_ServiceClient.call(srv)){
        return srv.response.stopped;
    }   

    return true;
    
}
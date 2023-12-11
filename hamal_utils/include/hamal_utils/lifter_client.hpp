/**
 * @file lifter_client.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-12-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef LIFTER_CLIENT_HPP_
#define LIFTER_CLIENT_HPP_


#include <hamal_custom_interfaces/HomingOperationAction.h>
#include <actionlib/server/simple_action_server.h>

class LifterClient
{
    public:

    LifterClient();

    LifterClient(std::string lifter_action_server_name);

    ~LifterClient();

    private:

    

};

#endif // LIFTER_CLIENT_HPP_
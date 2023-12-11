/**
 * @file connection_checker.cpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-12-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hamal_utils/connection_checker.hpp"

ConnectionCheckerServer::ConnectionCheckerServer()
{

}

ConnectionCheckerServer::ConnectionCheckerServer(
    const std::string& self_ip_addres,
    uint16_t port_num
)   : m_PortNumber(port_num), m_SelfIpAddres(m_SelfIpAddres)
{

} 

ConnectionCheckerServer::~ConnectionCheckerServer()
{
    
}

bool ConnectionCheckerServer::init()
{
    sockaddr_in serverConfig;

    bzero((char*)&serverConfig, sizeof(serverConfig));

    int setIpRes = inet_aton(m_SelfIpAddres.c_str(), &serverConfig.sin_addr);
    if(setIpRes == 0){
        return false;
    }
    serverConfig.sin_family = AF_INET;
    serverConfig.sin_port = htons(m_PortNumber);

    m_SocketFileDescriptor = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    if(m_SocketFileDescriptor == -1){
        return false;
    }
     
    auto clientWaitTimerStart = std::chrono::steady_clock::now();
    double elapsedTimeWhileWaitingClient = 0.0;
    bool clientConnected = false;
    
    int bindRes = bind(m_SocketFileDescriptor, (struct sockaddr*)&serverConfig, sizeof(serverConfig));
    if(bindRes < 0){
        return false;
    }

    listen(m_SocketFileDescriptor, 5);
    socklen_t incomingConnectionConfigSize = sizeof(m_ConnectedClientConfig); 
    while(elapsedTimeWhileWaitingClient <= 5.0 && clientConnected)
    {
        int accResult = accept(m_SocketFileDescriptor, (sockaddr*)&m_ConnectedClientConfig, &incomingConnectionConfigSize);
        if(accResult < 0){
            clientConnected = false;
            break;
        }
        auto currTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> diffTime = currTime - clientWaitTimerStart;
        elapsedTimeWhileWaitingClient= diffTime.count();
    };

    if(!clientConnected){
        return false;
    }

    return true;

}

bool ConnectionCheckerServer::queryForConnections()
{
        
}

/**
 * 
 * 
 * 
 * 
 */


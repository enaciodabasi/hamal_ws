/**
 * @file connection_checker.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-12-06
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef CONNECTION_CHECKER_HPP_
#define CONNECTION_CHECKER_HPP_

#include <iostream>
#include <string>
#include <chrono>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <netdb.h>
#include <sys/uio.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <fstream>

class ConnectionCheckerServer
{
    public:

    ConnectionCheckerServer();

    ConnectionCheckerServer(
        const std::string& self_ip_addres, 
        uint16_t port_num
    );
    
    ~ConnectionCheckerServer();

    bool init();

    bool queryForConnections();

    private:

    int m_SocketFileDescriptor;

    uint16_t m_PortNumber;

    std::string m_SelfIpAddres;

    sockaddr_in m_ConnectedClientConfig;

};

#endif // CONNECTION_CHECKER_HPP_
#pragma once

#include <ros/ros.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <autol_msgs/AutolPacket.h>
#include "autol_driver/udp_packet.h"

namespace autol_driver
{
    class UDPSocket
    {
    public:
        UDPSocket();
        ~UDPSocket();

        int CreateSocket();
        int CloseSocket();
        int RecvFrom(char* buffer, int len, int flags = 0); // OOB = 0, Non-Blocking = 2
        int RecvFrom(autol_msgs::AutolPacket* buffer, int len, int flags = 0);
        int Bind(unsigned short port);
        int SetTimeout(timeval tv);
        int SetSocketBuffer(int size);
        int PrintSocketBuffer();
        int udp_socket_;
    };
}

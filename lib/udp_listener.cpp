#include <novatel_receive/udp_listener.h>
#include <stdio.h>
#include <stdint.h>
#include <signal.h>
#include <stdlib.h>


UDP_receiver::UDP_receiver()
{
    memset(&ServerInfo, 0, sizeof(ServerInfo));
    //memset((char *)&FromClient, 0, sizeof(FromClient));

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock < 0)
    {
        perror("socket");
        exit(0);
    }

    ServerInfo.sin_family = AF_INET;
    ServerInfo.sin_port = htons(UDP_PORT);
    ServerInfo.sin_addr.s_addr = htonl(INADDR_ANY);
    // tv.tv_sec = 10;
    // tv.tv_usec = 0;

    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&optVal, optLen);

    if(sock < 0)
    {
        perror("socket init error");
        exit(0);
    }

    if(bind(sock, (struct sockaddr *)&ServerInfo, sizeof(ServerInfo)) < 0)
    {
        perror("Error Bind");
        exit(0);
    }

    FD_ZERO(&readfds);
    FD_SET(sock, &readfds);
}

UDP_receiver::~UDP_receiver()
{

}

void UDP_receiver::end(int sig)
{
    if(sig == 2)
    {
        int test = shutdown(sock, SHUT_RDWR);
        test = close(sock);
        std::cout << "socket end : " << test << std::endl;
    }
}

void UDP_receiver::loop()
{
    int i = 0;
    can_msgs::Frame can_msg;

    signal(SIGINT, end);

    while(ros::ok())
    {
        ros::spinOnce();

        memset(Buffer_recv, 0, PACKET_SIZE);

        Recv_Size = recvfrom(sock, Buffer_recv, PACKET_SIZE, 0, (struct sockaddr *)&FromClient, (socklen_t*) &rx_addr_len);

        if(Recv_Size < 0)
        {
            perror("recvfrom error");
            exit(0);
        }
        else
        {
            ROS_INFO("RECV SiZE: %d", Recv_Size);
        }

        structCPT7_header.MessageID = (uint16_t)Buffer_recv[4] + (((uint16_t)Buffer_recv[5]) << 8);
        ROS_INFO("Recv Message ID : %d", structCPT7_header.MessageID);

        can_msg.header.stamp = ros::Time::now();

    


    }
}
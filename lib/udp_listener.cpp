#include <novatel_receive/udp_listener.h>
#include <stdio.h>
#include <stdint.h>
#include <signal.h>
#include <stdlib.h>


UDP_receiver::UDP_receiver()
{
    memset(&ServerInfo, 0, sizeof(ServerInfo));
    memset(&NMEAServerInfo, 0, sizeof(NMEAServerInfo));

    //memset((char *)&FromClient, 0, sizeof(FromClient));

    memset(&msg_BESTPOS, 0, sizeof(BESTPOS_t));

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    nmea_sock = socket(AF_INET, SOCK_DGRAM, 0);

    if(sock < 0)
    {
        perror("socket");
        exit(0);
    }

    if(nmea_sock < 0)
    {
        perror("nmea socket");
        exit(0);
    }

    ServerInfo.sin_family = AF_INET;
    ServerInfo.sin_port = htons(UDP_PORT);
    ServerInfo.sin_addr.s_addr = htonl(INADDR_ANY);
    NMEAServerInfo.sin_family = AF_INET;
    NMEAServerInfo.sin_port = htons(NMEA_PORT);
    NMEAServerInfo.sin_addr.s_addr = htonl(INADDR_ANY);
    tv.tv_sec = 10;
    tv.tv_usec = 0;

    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&optVal, optLen);

    if(sock < 0)
    {
        perror("socket init error");
        exit(0);
    }
    if(nmea_sock < 0)
    {
        perror("nmea socket init error");
        exit(0);
    }

    if(bind(sock, (struct sockaddr *)&ServerInfo, sizeof(ServerInfo)) < 0)
    {
        perror("Error Bind");
        exit(0);
    }
    if(bind(nmea_sock, (struct sockaddr *)&NMEAServerInfo, sizeof(NMEAServerInfo)) < 0)
    {
        perror("NMEA Error Bind");
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

        int test2 = shutdown(nmea_sock, SHUT_RDWR);
        test2 = close(nmea_sock);
        std::cout << "socket end : " << test2 << std::endl;
    }
}

void UDP_receiver::loop()
{
    signal(SIGINT, end);

    while(ros::ok())
    {
        ros::spinOnce();

        memset(Buffer_recv, 0, PACKET_SIZE);

        FD_ZERO(&readfds);

        FD_SET(sock, &readfds);
        FD_SET(nmea_sock, &readfds);

        int activity = select(FD_SETSIZE, &readfds, NULL, NULL, &tv);

        if((activity < 0) && (errno != EINTR))
        {
            perror("select error");
            exit(0);
        }

        if(FD_ISSET(sock, &readfds))
        {
            Recv_Size = recvfrom(sock, Buffer_recv, PACKET_SIZE, MSG_WAITALL, (struct sockaddr *)&FromClient, (socklen_t*) &rx_addr_len);
            ROS_INFO("PORT 3003 Recv Size: %d", Recv_Size);
            Parser();
        }

        if(FD_ISSET(nmea_sock, &readfds))
        {
            Recv_Size = recvfrom(nmea_sock, Buffer_recv, PACKET_SIZE, MSG_WAITALL, (struct sockaddr *)&FromClient, (socklen_t*) &rx_addr_len);
            ROS_INFO("PORT 3002 Recv Size: %d", Recv_Size);
            Parser();
        }
        

        // if(Recv_Size < 0)
        // {
        //     perror("recvfrom error");
        //     exit(0);
        // }
        // else
        // {
        //     ROS_INFO("RECV SiZE: %d", Recv_Size);

        //     Parser();
        // }

    }
}

void UDP_receiver::NMEA_Parser()
{
    char NMEAbuf[200];
    char *token;

    memcpy(NMEAbuf, Buffer_recv, 200);
    token = strtok(NMEAbuf, ",*"); //$GPRMC
    ROS_INFO("%s", token); 
    
    token = strtok(NULL, ",*"); //utc
    // memcpy(msg_GPRMC.utc, token, sizeof(token));
    // msg_GPRMC.utc[sizeof(token)] = NULL;
    msg_GPRMC.utc = token;
    ROS_INFO("%s", msg_GPRMC.utc);

    token = strtok(NULL, ",*"); //pos_status
    msg_GPRMC.pos_status = token[0];
    ROS_INFO("%c", msg_GPRMC.pos_status);

    token = strtok(NULL, ",*"); //lat
    // memcpy(msg_GPRMC.lat, token, sizeof(token));
    // msg_GPRMC.lat[sizeof(token)] = NULL;
    msg_GPRMC.lat = token;
    ROS_INFO("%s", msg_GPRMC.lat);
    
    token = strtok(NULL, ",*"); //utc
    // memcpy(msg_GPRMC.lat_dir, token, sizeof(token));
    msg_GPRMC.lat_dir = token[0];
    ROS_INFO("%c", msg_GPRMC.lat_dir);

    token = strtok(NULL, ",*"); //lat
    // memcpy(msg_GPRMC.lon, token, sizeof(token));
    // msg_GPRMC.lon[sizeof(token)] = NULL;
    msg_GPRMC.lon = token;
    ROS_INFO("%s", msg_GPRMC.lon);
    
    token = strtok(NULL, ",*"); //utc
    // memcpy(msg_GPRMC.lon_dir, token, sizeof(token));
    msg_GPRMC.lon_dir = token[0];
    ROS_INFO("%c", msg_GPRMC.lon_dir);

    token = strtok(NULL, ",*"); //lat
    // memcpy(msg_GPRMC.speed_kn, token, sizeof(token));
    // msg_GPRMC.speed_kn[sizeof(token)] = NULL;
    msg_GPRMC.speed_kn = token;
    ROS_INFO("%s", msg_GPRMC.speed_kn);

    token = strtok(NULL, ",*"); //lat
    // memcpy(msg_GPRMC.track_true, token, sizeof(token));
    // msg_GPRMC.track_true[sizeof(token)] = NULL;
    msg_GPRMC.track_true = token;
    ROS_INFO("%s", msg_GPRMC.track_true);

    token = strtok(NULL, ",*"); //lat
    // memcpy(msg_GPRMC.date, token, sizeof(token));
    // msg_GPRMC.date[sizeof(token)] = NULL;
    msg_GPRMC.date = token;
    ROS_INFO("%s", msg_GPRMC.date);
    
    token = strtok(NULL, ",*"); //lat
    // memcpy(msg_GPRMC.mag_var, token, sizeof(token));
    // msg_GPRMC.mag_var[sizeof(token)] = NULL;
    msg_GPRMC.mag_var = token;
    ROS_INFO("%s", msg_GPRMC.mag_var);

    token = strtok(NULL, ",*"); //lat
    msg_GPRMC.var_dir = token[0];
    ROS_INFO("%c", msg_GPRMC.var_dir);

    token = strtok(NULL, ",*"); //lat
    msg_GPRMC.mode_ind = token[0];
    ROS_INFO("%c", msg_GPRMC.mode_ind);

    msg_GPRMC.d_utc = atof(msg_GPRMC.utc);
    msg_GPRMC.d_lat = atof(msg_GPRMC.lat);
    msg_GPRMC.d_lon = atof(msg_GPRMC.lon);
    msg_GPRMC.d_speed_kn = atof(msg_GPRMC.speed_kn);
    msg_GPRMC.d_track_true = atof(msg_GPRMC.track_true);

    ROS_INFO("%lf", msg_GPRMC.d_utc);
    ROS_INFO("%lf", msg_GPRMC.d_lat);
    ROS_INFO("%lf", msg_GPRMC.d_lon);
    ROS_INFO("%lf", msg_GPRMC.d_speed_kn);
    ROS_INFO("%lf", msg_GPRMC.d_track_true);
}

void UDP_receiver::Parser()
{
    uint16_t temp_id;
    
    ROS_INFO("%x", Buffer_recv[0]);
    if(Buffer_recv[0] == 0x24) //for NMEA
    {
        NMEA_Parser();
    }
    else
    {
        temp_id = (uint16_t)Buffer_recv[4];
        temp_id |= ((((uint16_t)Buffer_recv[5]) << 8) & 0xFF00);
        // ROS_INFO("header %02x %02x %02x", Buffer_recv[0], Buffer_recv[1], Buffer_recv[2]);
        ROS_INFO("ID %02x %02x, %x dec: %d", Buffer_recv[4], Buffer_recv[5], temp_id, temp_id);
        
        switch(temp_id)
        {
            case BESTPOS:
            {
                // void *p;
                int i = 0;

                // p = malloc(sizeof(BESTPOS_t));
                // memcpy(p, Buffer_recv, sizeof(BESTPOS_t));
                memcpy(&msg_BESTPOS, (BESTPOS_t *)Buffer_recv, sizeof(BESTPOS_t));
                // ROS_INFO("%2x %2x", Buffer_recv[36], Buffer_recv[37]);

                can_msg.header.seq += 1;
                can_msg.header.stamp = ros::Time::now();
                can_msg.header.frame_id = "BESTPOS_LAT";

                can_msg.id = 0x100;
                can_msg.dlc = 8;
                for(i=0;i<8;i++)
                {
                    can_msg.data[i] = msg_BESTPOS.lat.bytes[i];
                }
                
                pub.publish(can_msg);

                can_msg.header.seq += 1;
                can_msg.header.stamp = ros::Time::now();
                can_msg.header.frame_id = "BESTPOS_LON";

                can_msg.id = 0x101;
                can_msg.dlc = 8;
                for(i=0;i<8;i++)
                {
                    can_msg.data[i] = msg_BESTPOS.lon.bytes[i];
                }
                
                pub.publish(can_msg);
                // free(p);
                
                
                break;
            }
            case CORRIMUS:
            {
                int i = 0;
                
                memcpy(&msg_CORRIMUS, (CORRIMUS_t *)Buffer_recv, sizeof(CORRIMUS_t));

                can_msg.header.seq += 1;
                can_msg.header.stamp = ros::Time::now();
                can_msg.header.frame_id = "INSPVAS_YAWRATE";

                can_msg.id = 0x106;
                can_msg.dlc = 8;
                for(i=0;i<4;i++)
                {
                    can_msg.data[i] = msg_CORRIMUS.YawRate.bytes[i];
                }
                
                pub.publish(can_msg);
                break;
            }
            case INSPVAS:
            {
                int i = 0;
                
                memcpy(&msg_INSPVAS, (INSPVAS_t *)Buffer_recv, sizeof(INSPVAS_t));

                can_msg.header.seq += 1;
                can_msg.header.stamp = ros::Time::now();
                can_msg.header.frame_id = "INSPVAS_AZIMUTH";

                can_msg.id = 0x105;
                can_msg.dlc = 8;
                for(i=0;i<4;i++)
                {
                    can_msg.data[i] = msg_INSPVAS.Azimuth.bytes[i];
                }
                ROS_INFO("Azimuth: %.2lf", msg_INSPVAS.Azimuth.doubleValue);
                pub.publish(can_msg);
                break;
            }
            case HEADING2:
            {
                int i = 0;
                
                memcpy(&msg_HEADING2, (HEADING2_t *)Buffer_recv, sizeof(HEADING2_t));

                can_msg.header.seq += 1;
                can_msg.header.stamp = ros::Time::now();
                can_msg.header.frame_id = "HEADING2_HEADING";

                can_msg.id = 0x104;
                can_msg.dlc = 4;
                for(i=0;i<4;i++)
                {
                    can_msg.data[i] = msg_HEADING2.heading.bytes[i];
                }
                
                pub.publish(can_msg);

                break;
            }
            case BESTUTM:
            {
                int i = 0;

                memcpy(&msg_BESTUTM, (BESTUTM_t *)Buffer_recv, sizeof(BESTUTM_t));

                can_msg.header.seq += 1;
                can_msg.header.stamp = ros::Time::now();
                can_msg.header.frame_id = "BESTUTM_NORTH";

                can_msg.id = 0x102;
                can_msg.dlc = 8;
                for(i=0;i<8;i++)
                {
                    can_msg.data[i] = msg_BESTUTM.northing.bytes[i];
                }
                
                pub.publish(can_msg);

                can_msg.header.seq += 1;
                can_msg.header.stamp = ros::Time::now();
                can_msg.header.frame_id = "BESTUTM_EAST";

                can_msg.id = 0x103;
                can_msg.dlc = 8;
                for(i=0;i<8;i++)
                {
                    can_msg.data[i] = msg_BESTUTM.easting.bytes[i];
                }
                
                pub.publish(can_msg);

                ROS_INFO("%lf %lf", msg_BESTUTM.northing.doubleValue, msg_BESTUTM.easting.doubleValue);
                break;
            }
            default:
                break;
                
        }
    }
}
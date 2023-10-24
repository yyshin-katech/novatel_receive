#ifndef _UDP_LISTENER_H_
#define _UDP_LISTENER_H_

#include <fstream>
#include <iostream>
#include <vector>
#include <cstdio>
#include <iostream>
#include <stdio.h>
#include <cerrno>
#include <cstring>


#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <ros/ros.h>
#include <can_msgs/Frame.h>

#define UDP_PORT 3003
#define UDP_IP
#define PACKET_SIZE 300

typedef enum {
    UNKNOWN = 20,
    APPROXIMATE = 60,
    COARSEADJUSTING = 80,
    COARSE = 100,
    COARSESTEERING = 120,
    FREEWHEELING = 130,
    FINEADJUSTING = 140,
    FINE = 160,
    FINEBACKUPSTEERING = 170,
    FINESTEERING = 180,
    SATTIME = 200
} GPS_Reference_Time_Status_t;

typedef enum {
    SOL_COMPUTED = 0,
    INSUFFICIENT_OBS = 1,
    NO_CONVERGENCE = 2,
    SINGULARITY = 3,
    COV_TRACE = 4,
    TEST_DIST = 5,
    COLD_START = 6,
    V_H_LIMIT = 7,
    VARIANCE = 8,
    Solution_Status_RESIDUALS = 9,
    Solution_Status_RESERVED1 = 10,
    Solution_Status_RESERVED2 = 11,
    Solution_Status_RESERVED3 = 12,
    INTEGRITY_WARNING = 13,
    Solution_Status_RESERVED4 = 14,
    Solution_Status_RESERVED5 = 15,
    Solution_Status_RESERVED6 = 16,
    Solution_Status_RESERVED7 = 17,
    PENDING = 18,
    INVALID_FIX = 19,
    UNAUTHORIZED = 20,
    Solution_Status_RESERVED8 = 21,
    INVALID_RATE = 22
} Solution_Status_t;

typedef enum {
    NONE = 0,
    FIXEDPOS = 1,
    FIXEDHEIGHT = 2,
    Position_Velocity_Type_RESERVED1 = 3,
    Position_Velocity_Type_RESERVED2 = 4,
    Position_Velocity_Type_RESERVED3 = 5,
    Position_Velocity_Type_RESERVED4 = 6,
    Position_Velocity_Type_RESERVED5 = 7,
    DOPPLER_VELOCITY = 8,
    SINGLE = 16,
    PSRDIFF = 17,
    WAAS = 18,
    PROPAGATED = 19,
    L1_FLOAT = 32,
    NARROW_FLOAT = 34,
    L1_INT = 48,
    WIDE_INT = 49,
    NARROW_INT = 50,
    RTK_DIRECT_INS = 51,
    INS_SBAS = 52,
    INS_PSRSP = 53,
    INS_PSRDIFF = 54,
    INS_RTKFLOAT = 55,
    INS_RTKFIXED = 56,
    PPP_CONVERGING = 68,
    PPP = 69,
    OPERATIONAL = 70,
    WARNING = 71,
    OUT_OF_BOUNDS = 72,
    INS_PPP_CONVERGING = 73,
    INS_PPP = 74,
    PPP_BASIC_CONVERGING = 77,
    PPP_BASIC = 78,
    INS_PPP_BASIC_CONVERGING = 79,
    INS_PPP_BASIC = 80
} Position_Velocity_Type_t;

typedef enum {
    WGS84 = 61,
    USER = 63
} DATUM_ID_t;

typedef struct CPT7_header {
    uint8_t     Sync1;
    uint8_t     Sync2;
    uint8_t     Sync3;
    uint8_t     Header_Length;
    uint16_t    MessageID;
    uint8_t     Message_Type;
    uint8_t     Port_Address;
    uint16_t    Message_Length;
    uint16_t    Sequence;
    uint8_t     Idle_Time;
    GPS_Reference_Time_Status_t     Time_Status;
    uint16_t    Week;
    uint32_t    ms;
    uint32_t    Receiver_Status;
    uint16_t    Reserved;
    uint16_t    sw_version;
} CPT7_header_t;

typedef struct BESTPOS {
    Solution_Status_t sol_stat;
    Position_Velocity_Type_t pos_type;
    double      lat;
    double      lon;
    double      hgt;
    float       undulation;
    DATUM_ID_t  datum_id;
    float       lat_sigma;
    float       lon_sigma;
    float       hgt_sigma;
    uint8_t     stn_id[4];
    float       diff_age;
    float       sol_age;
    uint8_t     SVs;
    uint8_t     solnSVs;
    uint8_t     solnL1SVs;
    uint8_t     solnMultiSVs;
    uint8_t     Reserved;
    uint8_t     ext_sol_stat;
    uint8_t     Galileo_and_BeiDou_sig_mask;
    uint8_t     GPS_and_GLONASS_sig_mask;
    uint8_t     CRC[4];
} BESTPOS_t;

static int sock;

class UDP_receiver{

    public:

        UDP_receiver();
        ~UDP_receiver();
        
        ros::Publisher pub;

        CPT7_header_t structCPT7_header;
        BESTPOS_t structBESTPOS;

        struct sockaddr_in  ServerInfo;
        struct sockaddr_in  FromClient;
        
        socklen_t   rx_addr_len;

        fd_set sfd, readfds;
        struct timeval tv;

        int n;

        int FromeClient_Size;
        int Recv_Size;

        char bufData[PACKET_SIZE] = {0, };
        char Buffer[PACKET_SIZE] = {0, };
        char Buffer_recv[PACKET_SIZE] = {0, };

        short ServerPort = UDP_PORT;

        int optVal = 10000;
        int optLen = sizeof(optVal);

        void loop();

        static void end(int sig);

        std::tuple<CPT7_header_t, BESTPOS_t> CPT7_BESTPOS;
};

#endif 
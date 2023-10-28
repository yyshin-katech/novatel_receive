// 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

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
#include <sstream>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <ros/ros.h>
#include <can_msgs/Frame.h>

#include <novatel_receive/novatel_msgid.h>

#define UDP_PORT 3003
#define NMEA_PORT 3002
// #define UDP_IP "192.168.10.110"
#define UDP_IP "192.168.1.195"
#define PACKET_SIZE 1000

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

union DoubleToBytes {
    double doubleValue;
    uint8_t bytes[8];
};
union FloatToBytes {
    float floatValue;
    uint8_t bytes[4];
};

typedef struct sNMEA_GPRMC {
    // uint8_t     utc[10];
    char     *utc;
    uint8_t     pos_status; //A : valid, V : invalid
    // uint8_t     lat[14];
    char     *lat;
    uint8_t     lat_dir;
    // uint8_t     lon[15];
    char     *lon;
    uint8_t     lon_dir;
    // uint8_t     speed_kn[5];
    char     *speed_kn;
    // uint8_t     track_true[5];
    char     *track_true;
    // uint8_t     date[6];
    char     *date;
    // uint8_t     mag_var[5];
    char     *mag_var;
    uint8_t     var_dir;
    uint8_t     mode_ind;

    double      d_utc;
    double      d_lat;
    double      d_lon;
    double      d_speed_kn;
    double      d_track_true;
} NMEA_GPRMC_t;

typedef struct sCPT7_header {
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
    uint8_t     Time_Status;
    uint16_t    Week;
    uint32_t    ms;
    uint32_t    Receiver_Status;
    uint16_t    Reserved;
    uint16_t    sw_version;
} __attribute__((packed)) CPT7_header_t;

typedef struct sCPT7_shortheader {
    uint8_t     Sync1;
    uint8_t     Sync2;
    uint8_t     Sync3;
    uint8_t     Message_Length;
    uint16_t    MessageID;
    uint16_t    Week_Number;
    uint32_t    Miliseconds;
} __attribute__((packed)) CPT7_shortheader_t;

typedef struct sBESTPOS {
    CPT7_header_t   header;
    uint8_t sol_stat[4];
    uint8_t pos_type[4];
    // double      lat;
    // double      lon;
    // double      hgt;
    DoubleToBytes lat;
    DoubleToBytes lon;
    DoubleToBytes hgt;
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
} __attribute__((packed)) BESTPOS_t;

typedef struct sINSPVAS {
    CPT7_shortheader_t   header;
    uint32_t        Week;
    double          Seconds;
    double          Latitude;
    double          Longitude;
    double          Height;
    double          North_Velocity;
    double          East_Velocity;
    double          Up_Velocity;
    double          Roll;
    double          Pitch;
    DoubleToBytes   Azimuth;
    uint32_t        Status;
} __attribute__((packed)) INSPVAS_t;

typedef struct sCORRIMUS {
    CPT7_shortheader_t   header;
    uint32_t        IMUDataCount;
    double          PitchRate;
    double          RollRate;
    DoubleToBytes   YawRate;
    double          LateralAcc;
    double          LongitudinalAcc;
    double          VerticalAcc;
    float           Reserved0;
    uint32_t        Reserved1;
    uint8_t         CRC[4];
} __attribute__((packed)) CORRIMUS_t;

typedef struct sHEADING2 {
    CPT7_header_t   header;
    uint32_t        sol_stat;
    uint32_t        pos_type;
    float           length;
    FloatToBytes    heading;
    float           pitch;
    float           Reserved0;
    float           hdg_std_dev;
    float           ptch_std_dev;
    uint8_t         rover_stn_ID[4];
    uint8_t         Master_stn_ID[4];
    uint8_t         SVs;
    uint8_t         solnSVs;
    uint8_t         obs;
    uint8_t         multi;
    uint8_t         sol_source;
    uint8_t         ext_sol_stat;
    uint8_t         Galileo_and_BeiDou_sig_mask;
    uint8_t         GPS_and_GLONASS_sig_mask;
    uint8_t         CRC[4];
} __attribute__((packed)) HEADING2_t;

typedef struct sBESTUTM {
    CPT7_header_t   header;
    Solution_Status_t sol_stat;
    Position_Velocity_Type_t pos_type;
    uint32_t            zone_number;
    uint32_t            zone;
    DoubleToBytes       northing;
    DoubleToBytes       easting;
    double              hgt;
    float               undulation;
    DATUM_ID_t          datum_id;
    float               N_sigma;
    float               E_sigma;
    float               ght_sigma;
    uint8_t             std_id[4];
    float               diff_age;
    float               sol_age;
    uint8_t             SVs;
    uint8_t             solnSVs;
    uint8_t             ggL1;
    uint8_t             solnMultiSV;
    uint8_t             Reserved;
    uint8_t             ext_sol_stat;
    uint8_t             Galileo_and_BeiDou_sig_mask;
    uint8_t             GPS_and_GLONASS_sig_mask;
    uint8_t             CRC[4];
} __attribute__((packed)) BESTUTM_t;

static int sock;
static int nmea_sock;

class UDP_receiver{

    public:

        UDP_receiver();
        ~UDP_receiver();

        ros::Publisher pub;

        //CPT7_header_t structCPT7_header;
        BESTPOS_t msg_BESTPOS;
        INSPVAS_t msg_INSPVAS;
        CORRIMUS_t  msg_CORRIMUS;
        BESTUTM_t   msg_BESTUTM;
        HEADING2_t  msg_HEADING2;
        NMEA_GPRMC_t msg_GPRMC;

        can_msgs::Frame can_msg;

        struct sockaddr_in  ServerInfo;
        struct sockaddr_in  NMEAServerInfo;
        struct sockaddr_in  FromClient;
        
        socklen_t   rx_addr_len;

        fd_set sfd, readfds;
        struct timeval tv;

        int n;

        int FromeClient_Size;
        int Recv_Size;

        uint8_t Buffer_recv[PACKET_SIZE] = {0, };

        short ServerPort = UDP_PORT;

        int optVal = 10000;
        int optLen = sizeof(optVal);

        void loop();
        
        void Parser();
        void NMEA_Parser();
        
        static void end(int sig);

        //std::tuple<CPT7_header_t, BESTPOS_t> CPT7_BESTPOS;
};

#endif 
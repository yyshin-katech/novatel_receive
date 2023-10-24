#include <ros/ros.h>
#include <novatel_receive/udp_listener.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "novatel_listener");
    ros::NodeHandle node("~");

    UDP_receiver udp_receiver;

    udp_receiver.pub = node.advertise<can_msgs::Frame>("/can_tx", 1);

    udp_receiver.loop();

}
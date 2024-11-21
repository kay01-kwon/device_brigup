#include "interface/mavlink_messages.h"
#include "interface/serial_port.h"

#include "ros_mavlink/ros_wrapper_mavlink.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_mavlink_node");

    ros::NodeHandle nh;

    string port_name;
    int baud_rate;

    nh.getParam("port_name", port_name);
    nh.getParam("baud_rate", baud_rate);


    SerialPort port(port_name, baud_rate);

    RosWrapperMavlink wrapper(nh, &port);

    wrapper.ros_run();

    return 0;
}
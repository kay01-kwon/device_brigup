#include "interface/mavlink_messages.h"
#include "interface/serial_port.h"

#include "ros_mavlink/ros_wrapper_mavlink.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_mavlink_node");

    ros::NodeHandle nh;

    string port_name;

    nh.getParam("port_name", port_name);

    SerialPort port(port_name, 921600);

    RosWrapperMavlink wrapper(nh, &port);

    wrapper.ros_run();

    return 0;
}
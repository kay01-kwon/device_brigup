#ifndef ROS_WRAPPER_MAVLINK_H
#define ROS_WRAPPER_MAVLINK_H

#include <time.h>
#include <sys/time.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include "interface/serial_port.h"
#include "interface/mavlink_messages.h"

#include <boost/thread/thread.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


#include <tf/transform_broadcaster.h>

using sensor_msgs::Imu;
using sensor_msgs::MagneticField;

using Eigen::Vector3d;
using Eigen::Vector4d;


class RosWrapperMavlink
{
    public:

    RosWrapperMavlink() = delete;

    RosWrapperMavlink(const RosWrapperMavlink &other) = delete;

    RosWrapperMavlink &operator=(const RosWrapperMavlink &other) = delete;


    // Constructor for setting up the ROS node handle, 
    // serial port, and default ROS rate of 100 Hz
    RosWrapperMavlink(const ros::NodeHandle &nh,SerialPort *port);

    ~RosWrapperMavlink();

    private:

    MavlinkMessages current_messages_;

    SerialPort *port_;

    ros::NodeHandle nh_;

    ros::Publisher imu_pub_;
    ros::Publisher mag_pub_;

    bool is_highres_imu_received_{false};
    bool is_attitude_quaternion_received_{false};

    
    Imu imu_msg_;
    MagneticField mag_msg_;

    void ros_run_mavlink();

    void publisher_subscriber_setup();

    void message_interveral_setup();

    void read_heart_beat(mavlink_message_t *message);

    void set_message_interval(const int sysid, const int compid,
    const int message_id, const float interval);


    tf::Transform transform_;
    
};


#endif // ROS_WRAPPER_MAVLINK_H
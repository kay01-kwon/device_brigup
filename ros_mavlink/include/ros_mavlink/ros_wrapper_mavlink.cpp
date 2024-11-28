#include "ros_wrapper_mavlink.hpp"

RosWrapperMavlink::RosWrapperMavlink(const ros::NodeHandle &nh,
SerialPort *port)
: nh_(nh), port_(port)
{
    port_->start();

    usleep(1000);

    message_interveral_setup();

    publisher_subscriber_setup();

    transform_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));

    ros_run_mavlink();

}

RosWrapperMavlink::~RosWrapperMavlink()
{
}


void RosWrapperMavlink::ros_run_mavlink()
{
    mavlink_message_t message;

    ROS_INFO("Reading IMU message");

    while(ros::ok())
    {
        // int result = port_->read_message(&message);
        while(port_->read_message(&message))
        {
            current_messages_.sysid = message.sysid;
            current_messages_.compid = message.compid;

            switch(message.msgid)
            {
                case MAVLINK_MSG_ID_HIGHRES_IMU:
                {
                    mavlink_msg_highres_imu_decode(&message, 
                    &current_messages_.highres_imu);

                    imu_msg_.header.stamp = ros::Time::now();
                    imu_msg_.header.frame_id = "base_link";
                    
                    imu_msg_.linear_acceleration.x = 
                    current_messages_.highres_imu.xacc;

                    imu_msg_.linear_acceleration.y =
                    -(current_messages_.highres_imu.yacc);

                    imu_msg_.linear_acceleration.z =
                    -(current_messages_.highres_imu.zacc);

                    imu_msg_.angular_velocity.x =
                    current_messages_.highres_imu.xgyro;

                    imu_msg_.angular_velocity.y =
                    -(current_messages_.highres_imu.ygyro);

                    imu_msg_.angular_velocity.z =
                    -(current_messages_.highres_imu.zgyro);


                    mag_msg_.header.stamp = ros::Time::now();
                    mag_msg_.header.frame_id = "base_link";

                    mag_msg_.magnetic_field.x =
                    current_messages_.highres_imu.xmag;

                    mag_msg_.magnetic_field.y =
                    -(current_messages_.highres_imu.ymag);

                    mag_msg_.magnetic_field.z =
                    -(current_messages_.highres_imu.zmag);

                    imu_pub_.publish(imu_msg_);
                    mag_pub_.publish(mag_msg_);

                    break;
                }

                case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
                {
                    mavlink_msg_attitude_quaternion_decode(&message, 
                    &current_messages_.attitude_quaternion);

                    imu_msg_.orientation.w =
                    current_messages_.attitude_quaternion.q1;

                    imu_msg_.orientation.x =
                    current_messages_.attitude_quaternion.q2;

                    imu_msg_.orientation.y =
                    -(current_messages_.attitude_quaternion.q3);

                    imu_msg_.orientation.z =
                    -(current_messages_.attitude_quaternion.q4);

                    static tf::TransformBroadcaster br;

                    tf::Quaternion q_tf(imu_msg_.orientation.x, 
                    imu_msg_.orientation.y,
                    imu_msg_.orientation.z, 
                    imu_msg_.orientation.w);

                    transform_.setRotation(q_tf);

                    br.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), 
                    "world", "base_link"));

                    break;
                }
                default:
                {
                    break;
                }

            }   // End of switch
        }   // End of if(result)

    }   // End of while(ros::ok())
}

void RosWrapperMavlink::publisher_subscriber_setup()
{
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data",1);
    mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("/mag/data",1);
}

void RosWrapperMavlink::message_interveral_setup()
{
    // Message interval in micro seconds
    float interval;
    nh_.getParam("interval", interval);

    // Read the heartbeat message to get system and component id, resepctively
    mavlink_message_t message;
    read_heart_beat(&message);

    current_messages_.sysid = message.sysid;
    current_messages_.compid = message.compid;

    printf("System id: %d\n", current_messages_.sysid);
    printf("Component id: %d\n", current_messages_.compid);

    // After getting the system and component id, set the message interval

    // Set the message interval for the following messages
    // 1. Attitude quaternion
    set_message_interval(message.sysid, message.compid,
    MAVLINK_MSG_ID_ATTITUDE_QUATERNION, interval);

    // 2. Highres IMU
    set_message_interval(message.sysid, message.compid,
    MAVLINK_MSG_ID_HIGHRES_IMU, interval);

}

void RosWrapperMavlink::read_heart_beat(mavlink_message_t *message)
{
    message->msgid = 100;
    port_->read_message(message);

    while(message->msgid != MAVLINK_MSG_ID_HEARTBEAT)
    {
        port_->read_message(message);
    }
}

void RosWrapperMavlink::set_message_interval(const int sysid, const int compid,
    const int message_id, const float interval)
{
    mavlink_command_long_t com = {0};
    mavlink_message_t message_to_write;

    com.target_system = sysid;
    com.target_component = compid;
    com.confirmation = 0;
    
    com.command = MAV_CMD_SET_MESSAGE_INTERVAL;
    com.param1 = message_id;
    com.param2 = interval;

    mavlink_msg_command_long_encode(sysid, compid,
    &message_to_write, &com);
    port_->write_message(&message_to_write);
}
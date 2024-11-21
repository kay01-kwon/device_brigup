#include "ros_wrapper_mavlink.hpp"

RosWrapperMavlink::RosWrapperMavlink(const ros::NodeHandle &nh,
                                     SerialPort *port)
: nh_(nh), port_(port)
{
    current_messages_.sysid = sysid_;
    current_messages_.compid = compid_;

    publisher_setup();

    port_->start();

    usleep(1000);

    mavlink_message_t message;

    read_heart_beat(&message);

    current_messages_.sysid = message.sysid;
    current_messages_.compid = message.compid;

    printf("System id: %d\n", current_messages_.sysid);
    printf("Component id: %d\n", current_messages_.compid);

    set_message_interval(message.sysid, message.compid,
    MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 10000.0);

    set_message_interval(message.sysid, message.compid,
    MAVLINK_MSG_ID_HIGHRES_IMU, 10000.0);


}

RosWrapperMavlink::RosWrapperMavlink(const ros::NodeHandle &nh, SerialPort *port, int ros_rate)
: nh_(nh), port_(port), reading_status_(false)
{
    current_messages_.sysid = sysid_;
    current_messages_.compid = compid_;

    loop_rate_ = ros::Rate(ros_rate);

    arm_disarm(true);

    publisher_setup();

}

void RosWrapperMavlink::ros_run()
{
    mavlink_message_t message;
    float qx, qy, qz, qw;

    std::cout << "ROS Wrapper Mavlink running..." << std::endl;

    while(ros::ok())
    {
        // boost::lock_guard<boost::mutex> lock(mtx_);
        int result = port_->read_message(&message);
        if(result)
        {
            current_messages_.sysid = message.sysid;
            current_messages_.compid = message.compid;

            switch(message.msgid)
            {
                case MAVLINK_MSG_ID_HIGHRES_IMU:
                {
                    mavlink_msg_highres_imu_decode(&message, 
                    &current_messages_.highres_imu);
                    // printf("Highres IMU received\n");

                    // Write IMU message data
                    imu_msg_.header.frame_id = "imu_link";
                    
                    imu_msg_.linear_acceleration.x = 
                    current_messages_.highres_imu.xacc;

                    imu_msg_.linear_acceleration.y = 
                    current_messages_.highres_imu.yacc;

                    imu_msg_.linear_acceleration.z = 
                    current_messages_.highres_imu.zacc;

                    // Write Magnetic Field message data
                    mag_msg_.header.stamp = ros::Time::now();
                    mag_msg_.header.frame_id = "imu_link";

                    mag_msg_.magnetic_field.x =
                    current_messages_.highres_imu.xmag;

                    mag_msg_.magnetic_field.y =
                    current_messages_.highres_imu.ymag;

                    mag_msg_.magnetic_field.z =
                    current_messages_.highres_imu.zmag;

                    highres_imu_received_ = true;

                    mag_pub_.publish(mag_msg_);

                    break;
                }

                case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
                {
                    mavlink_msg_attitude_quaternion_decode(&message, 
                    &current_messages_.attitude_quaternion);

                    // printf("Attitude quaternion received\n");

                    qw = current_messages_.attitude_quaternion.q1;
                    qx = current_messages_.attitude_quaternion.q2;
                    qy = current_messages_.attitude_quaternion.q3;
                    qz = current_messages_.attitude_quaternion.q4;

                    imu_msg_.orientation.w = qw;

                    imu_msg_.orientation.x = qx;

                    imu_msg_.orientation.y =-qy;

                    imu_msg_.orientation.z =-qz;

                    imu_msg_.angular_velocity.x = current_messages_.attitude_quaternion.rollspeed;

                    imu_msg_.angular_velocity.y = -(current_messages_.attitude_quaternion.pitchspeed);

                    imu_msg_.angular_velocity.z = -(current_messages_.attitude_quaternion.yawspeed);

                    attitude_quaternion_received_ = true;

                    break;
                }
                default:
                    break;
            }   // End of switch
        }   // End of if

        if(highres_imu_received_ && attitude_quaternion_received_)
        {
            imu_msg_.header.stamp = ros::Time::now();
            highres_imu_received_ = false;
            attitude_quaternion_received_ = false;
            imu_pub_.publish(imu_msg_);
        }
    }   // End of while

}

RosWrapperMavlink::~RosWrapperMavlink()
{
    arm_disarm(false);
}

void RosWrapperMavlink::publisher_setup()
{
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data",1);
    mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("/mag/data",1);
}

int RosWrapperMavlink::arm_disarm(bool flag)
{
    mavlink_command_long_t com = {0};

    com.target_system = sysid_;
    com.target_component = compid_;
    com.command = MAV_CMD_COMPONENT_ARM_DISARM;
    com.confirmation = 0;
    com.param1 = flag ? 1.0 : 0.1;
    com.param2 = 0.0;
    com.param3 = 0.0;
    com.param4 = 0.0;
    com.param5 = 0.0;
    com.param6 = 0.0;
    com.param7 = 0.0;

    mavlink_message_t message;
    mavlink_msg_command_long_encode(sysid_, compid_, &message, &com);

    int len = port_->write_message(&message);

    return len;
}

int RosWrapperMavlink::toggle_offboard_control(bool flag)
{
    mavlink_command_long_t com = {0};
    com.target_system = sysid_;
    com.target_component = compid_;
    com.command = MAV_CMD_NAV_GUIDED_ENABLE;
    com.confirmation = true;
    com.param1 = flag ? 1.0 : 0.1;

    mavlink_message_t message;
    mavlink_msg_command_long_encode(sysid_, compid_, &message, &com);

    int len = port_->write_message(&message);

    return len;
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
    const int message_id, const float dt)
{
    mavlink_command_long_t com = {0};
    mavlink_message_t message_to_write;

    com.target_system = sysid;
    com.target_component = compid;
    com.confirmation = 0;
    
    com.command = MAV_CMD_SET_MESSAGE_INTERVAL;
    com.param1 = message_id;
    com.param2 = dt;

    mavlink_msg_command_long_encode(sysid_, compid_,
    &message_to_write, &com);
    port_->write_message(&message_to_write);
}

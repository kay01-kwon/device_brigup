#include "ros_wrapper_mavlink.hpp"

RosWrapperMavlink::RosWrapperMavlink(const ros::NodeHandle &nh,
SerialPort *port)
: nh_(nh), port_(port)
{
    port_->start();

    usleep(1000);

    message_interveral_setup();

    publisher_subscriber_setup();

    t_curr_ = ros::Time::now().toSec();
    t_prev_ = t_curr_;
    t_cam_prev_ = t_curr_;

    transform_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));

    ros_thread_ = boost::thread(&RosWrapperMavlink::ros_run_thread, this);
    mavlink_read_thread_ = boost::thread(&RosWrapperMavlink::read_IMU_message_thread, this);

}

RosWrapperMavlink::~RosWrapperMavlink()
{
    ros_thread_.join();
    mavlink_read_thread_.join();
}

void RosWrapperMavlink::ros_run_thread()
{

    while(ros::ok())
    {
        t_curr_ = ros::Time::now().toSec();

        if(t_curr_ - t_prev_ >= dt_cam_/4.0)
        {
            publish_message();
            // ROS_INFO("dt_cam: %f", dt_cam_);
            t_prev_ = t_curr_;
            imu_data_num_++;
        }

        if(imu_data_num_ >= 10)
        {
            imu_data_num_ = 0;
        }

        ros::spinOnce();
        boost::this_thread::sleep_for(
            boost::chrono::milliseconds(1)
        );
        // ROS_INFO("dt: %d", time_sleep);
    }
}

void RosWrapperMavlink::read_IMU_message_thread()
{
    mavlink_message_t message;

    double t_now;

    ROS_INFO("Reading IMU message thread");

    while(ros::ok())
    {
        // boost::lock_guard<boost::mutex> lock(mtx_);

        mtx_.lock();
        
        int result = port_->read_message(&message);
        if(result)
        {
            current_messages_.sysid = message.sysid;
            current_messages_.compid = message.compid;

            switch(message.msgid)
            {
                case MAVLINK_MSG_ID_HIGHRES_IMU:
                {
                    is_highres_imu_received_ = true;
                    mavlink_msg_highres_imu_decode(&message, 
                    &current_messages_.highres_imu);

                    // Only keep the last two messages
                    if(t_highres_imu_queue_.size() >= 2)
                    {
                        // ROS_INFO("Pop");
                        t_highres_imu_queue_.pop();
                        acc_queue_.pop();
                        mag_queue_.pop();
                    }

                    t_now = ros::Time::now().toSec();

                    t_highres_imu_queue_.push(t_now);

                    acc_queue_.push(Eigen::Vector3d(
                        current_messages_.highres_imu.xacc,
                        -(current_messages_.highres_imu.yacc),
                        -(current_messages_.highres_imu.zacc)
                    ));

                    mag_queue_.push(Eigen::Vector3d(
                        current_messages_.highres_imu.xmag,
                        current_messages_.highres_imu.ymag,
                        current_messages_.highres_imu.zmag
                    ));

                    break;
                }

                case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
                {
                    is_attitude_quaternion_received_ = true;
                    mavlink_msg_attitude_quaternion_decode(&message, &current_messages_.attitude_quaternion);

                    // Only keep the last two messages
                    if(t_attitude_quaternion_queue_.size() >= 2)
                    {
                        t_attitude_quaternion_queue_.pop();
                        gyro_queue_.pop();
                        quat_queue_.pop();
                    }

                    t_now = ros::Time::now().toSec();

                    t_attitude_quaternion_queue_.push(t_now);
                    
                    gyro_queue_.push(Eigen::Vector3d(
                        current_messages_.attitude_quaternion.rollspeed,
                        -(current_messages_.attitude_quaternion.pitchspeed),
                        -(current_messages_.attitude_quaternion.yawspeed)
                    ));

                    quat_queue_.push(Eigen::Vector4d(
                        current_messages_.attitude_quaternion.q1,
                        current_messages_.attitude_quaternion.q2,
                        -(current_messages_.attitude_quaternion.q3),
                        -(current_messages_.attitude_quaternion.q4)
                    ));

                    break;
                }
                default:
                {
                    break;
                }

            }   // End of switch
        }   // End of if
        mtx_.unlock();
        if(is_highres_imu_received_ && is_attitude_quaternion_received_)
        {
            boost::this_thread::sleep_for(boost::chrono::milliseconds(3));
            is_highres_imu_received_ = false;
            is_attitude_quaternion_received_ = false;
        }
    }   // End of while(ros::ok())
}

void RosWrapperMavlink::publish_message()
{
    mtx_.lock();

    convert_ros_message();

    static tf::TransformBroadcaster br;

    tf::Quaternion q_tf(imu_msg_.orientation.x, imu_msg_.orientation.y,
    imu_msg_.orientation.z, imu_msg_.orientation.w);

    transform_.setRotation(q_tf);

    br.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), 
    "world", "imu_link"));

    mtx_.unlock();
}

void RosWrapperMavlink::convert_ros_message()
{

    if(t_highres_imu_queue_.size() >= 2 && t_attitude_quaternion_queue_.size() >= 2)
    {
        assert(t_highres_imu_queue_.size() == 2);
        assert(t_attitude_quaternion_queue_.size() == 2);

        double t_now = ros::Time::now().toSec();

        Vector3d acc_interpolated;
        Vector3d gyro_interpolated;
        Vector3d mag_interpolated;
        Vector4d quat_interpolated;

        linear_interpolation(t_highres_imu_queue_, acc_queue_, t_now, acc_interpolated);
        linear_interpolation(t_highres_imu_queue_, mag_queue_, t_now, mag_interpolated);
        linear_interpolation(t_attitude_quaternion_queue_, gyro_queue_, t_now, gyro_interpolated);
        slerp_interpolation(t_attitude_quaternion_queue_, quat_queue_, t_now, quat_interpolated);

        imu_msg_.header.stamp = ros::Time::now();
        imu_msg_.header.frame_id = "imu_link";

        imu_msg_.linear_acceleration.x = acc_interpolated(0);
        imu_msg_.linear_acceleration.y = acc_interpolated(1);
        imu_msg_.linear_acceleration.z = acc_interpolated(2);

        imu_msg_.angular_velocity.x = gyro_interpolated(0);
        imu_msg_.angular_velocity.y = gyro_interpolated(1);
        imu_msg_.angular_velocity.z = gyro_interpolated(2);

        imu_msg_.orientation.w = quat_interpolated(0);
        imu_msg_.orientation.x = quat_interpolated(1);
        imu_msg_.orientation.y = quat_interpolated(2);
        imu_msg_.orientation.z = quat_interpolated(3);

        imu_pub_.publish(imu_msg_);

        mag_msg_.header.stamp = ros::Time::now();
        mag_msg_.header.frame_id = "mag_link";

        mag_msg_.magnetic_field.x = mag_interpolated(0);
        mag_msg_.magnetic_field.y = mag_interpolated(1);
        mag_msg_.magnetic_field.z = mag_interpolated(2);

        mag_pub_.publish(mag_msg_);
    }
}

void RosWrapperMavlink::publisher_subscriber_setup()
{
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data",1);
    mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("/mag/data",1);
    camera_info_sub_ = nh_.subscribe("/camera/infra1/camera_info", 1, 
    &RosWrapperMavlink::camera_info_callback, this);
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

void RosWrapperMavlink::camera_info_callback(const CameraInfo::ConstPtr &msg)
{

    t_curr_ = ros::Time::now().toSec();

    if(imu_data_num_ != 5)
        publish_message();

    imu_data_num_ = 1;

    dt_cam_ = t_curr_ - t_cam_prev_;
    t_cam_prev_ = t_curr_;
    t_prev_ = t_curr_;
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

void RosWrapperMavlink::linear_interpolation(const queue<double> &t_queue, 
const queue<Vector3d> &vec3_queue, 
const double t_now, Vector3d &vec3_interpolated)
{

    double t1 = t_queue.front();
    double t2 = t_queue.back();

    Vector3d vec3_1 = vec3_queue.front();
    Vector3d vec3_2 = vec3_queue.back();

    double alpha = (t_now - t1) / (t2 - t1);

    vec3_interpolated = (1 - alpha) * vec3_1 + alpha * vec3_2;
}

void RosWrapperMavlink::slerp_interpolation(const queue<double> &t_queue, 
const queue<Vector4d> &q_queue, 
const double t_now, Vector4d &q_interpolated)
{
    double t1 = t_queue.front();
    double t2 = t_queue.back();

    Vector4d q1 = q_queue.front();
    Vector4d q2 = q_queue.back();

    double alpha = (t_now - t1) / (t2 - t1);

    Eigen::Quaterniond quat1(q1(0), q1(1), q1(2), q1(3));
    Eigen::Quaterniond quat2(q2(0), q2(1), q2(2), q2(3));

    Eigen::Quaterniond quat_interpolated = quat1.slerp(alpha, quat2);

    q_interpolated << quat_interpolated.w(), quat_interpolated.x(),
    quat_interpolated.y(), quat_interpolated.z();
}

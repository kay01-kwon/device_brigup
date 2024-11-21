#ifndef ROS_WRAPPER_MAVLINK_H
#define ROS_WRAPPER_MAVLINK_H

#include <time.h>
#include <sys/time.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/CameraInfo.h>

#include "interface/serial_port.h"
#include "interface/mavlink_messages.h"

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <queue>

#include <tf/transform_broadcaster.h>

using sensor_msgs::Imu;
using sensor_msgs::MagneticField;
using sensor_msgs::CameraInfo;

using Eigen::Vector3d;
using Eigen::Vector4d;

using std::queue;

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

    bool is_highres_imu_received_{false};
    bool is_attitude_quaternion_received_{false};

    SerialPort *port_;

    ros::NodeHandle nh_;

    boost::thread ros_thread_;
    boost::thread mavlink_read_thread_;

    boost::mutex mtx_;
    boost::condition_variable cv_;

    ros::Publisher imu_pub_;
    ros::Publisher mag_pub_;
    ros::Subscriber camera_info_sub_;

    double t_curr_, t_prev_;
    
    double t_cam_prev_{0.0};
    double dt_cam_{1/30.0};

    int imu_data_num_{0};
    
    Imu imu_msg_;
    MagneticField mag_msg_;

    void ros_run_thread();

    void read_IMU_message_thread();

    void publish_message();

    void convert_ros_message();

    void publisher_subscriber_setup();

    void message_interveral_setup();

    void camera_info_callback(const CameraInfo::ConstPtr &msg);

    void read_heart_beat(mavlink_message_t *message);

    void set_message_interval(const int sysid, const int compid,
    const int message_id, const float interval);

    void linear_interpolation(const queue<double> &t_queue,
    const queue<Vector3d> &vec3_queue, 
    const double t_now, Vector3d &vec3_interpolated);

    void slerp_interpolation(const queue<double> &t_queue,
    const queue<Vector4d> &q_queue,
    const double t_now, Vector4d &q_interpolated);

    queue<double> t_acc_queue_;
    queue<double> t_ori_gyro_queue_;
    queue<double> t_mag_queue_;
    
    queue<Vector3d> acc_queue_;
    queue<Vector3d> gyro_queue_;
    queue<Vector3d> mag_queue_;
    queue<Vector4d> quat_queue_;

    tf::Transform transform_;
    
};


#endif // ROS_WRAPPER_MAVLINK_H
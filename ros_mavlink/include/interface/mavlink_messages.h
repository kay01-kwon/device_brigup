#ifndef MAVLINK_MESSAGES_H
#define MAVLINK_MESSAGES_H

#include "c_library_v2/common/mavlink.h"

struct TimeStamps{
    TimeStamps()
    : hearbeat(0),
      sys_status(0),
      highres_imu(0),
      attitude_quaternion(0)
    {

    }

    uint64_t hearbeat;
    uint64_t sys_status;
    uint64_t highres_imu;
    uint64_t attitude_quaternion;
};


struct MavlinkMessages{
    int sysid;
    int compid;

    mavlink_heartbeat_t heartbeat;
    mavlink_sys_status_t sys_status;
    mavlink_highres_imu_t highres_imu;
    mavlink_attitude_quaternion_t attitude_quaternion;

    TimeStamps time_stamps;
};

#endif  // MAVLINK_MESSAGES_H
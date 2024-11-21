# mavros for use

This package modifed the launch, yaml files of MAVROS

to reduce the computational load.

Loads of pluginlists has been blacklisted.

## How to launch the node

```
roslaunch ros_mavlink px4.launch
```

## How to increase the rate of streaming

Command for MAV_CMD_SET_MESSAGE_INTERVAL : 511

Message id for HIGHRES_IMU : 105

Message id for ATTITUDE_QUATERNION : 31


```
rosservice call /mavros/cmd/command "{broadcast: false, command: 511, confirmation: 0, param1: 105.0, param2: 2500.0, param3: 0.0,
  param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"
```

```
rosservice call /mavros/cmd/command "{broadcast: false, command: 511, confirmation: 0, param1: 31.0, param2: 2500.0, param3: 0.0,
  param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"
```
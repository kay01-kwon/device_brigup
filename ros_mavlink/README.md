# mavros for use

This package modifed the launch, yaml files of MAVROS to reduce the computational load.

In other words, several plugin lists has been blacklisted to reduce the computational load.

## How to launch the node

```
roslaunch ros_mavlink px4.launch
```

## How to set the rate of streaming as 200 Hz (5000 us = 5 ms)

Command for MAV_CMD_SET_MESSAGE_INTERVAL : 511

|Mavlink name|Mavlink message ID|
|:---:|:---:|
|HIGHRES_IMU|105|
|ATTITUDE_QUATERNION|31|
|MAV_CMD_SET_MESSAGE_INTERVAL|511|

MAV_CMD_SET_MESSASGE_INTERVAL
|Param|Description|Values|Units|
|:---:|:---:|:---:|:---:|
|1(Message ID)|The Mavlink message ID|min:0 max:16777215 inc:1|
|2(Interval)|The interval between two messages.|min:-1 inc:1|us|
|3(Req)|Assumed not used(0)|NA|NA|
|4(Req)|Assumed not used(0)|NA|NA|
|5(Req)|Assumed not used(0)|NA|NA|
|6(Req)|Assumed not used(0)|NA|NA|
|7(Response target)|1:address of requestor, 2:broadcast|min:0 max:2 inc:1|

```
rosservice call /mavros/cmd/command "{broadcast: false, command: 511, confirmation: 0, param1: 105.0, param2: 5000.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"
```

```
rosservice call /mavros/cmd/command "{broadcast: false, command: 511, confirmation: 0, param1: 31.0, param2: 5000.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"
```

Check if the rate of /mavros/imu/data_raw is around 200 Hz or not.

```
rostopic hz /mavros/imu/data_raw
```

## To record imu raw data for three hours (Allan variance ros)

```
rosbag record --duration=3h -O imu_static.bag /mavros/imu/data_raw
```

## Allan variance ros

```
rosrun allan_variance_ros cookbag.py --input imu_static.bag --output imu_static_cooked.bag
```

Make a folder for imu_static_cooked.bag and then move it to the folder.

Make a yaml file like the following.

px6_mini.yaml

```
imu_topic: "/mavros/imu/data_raw"
imu_rate: 200
measure_rate: 200
sequence_time: 10799
```

```
rosrun allan_variance_ros allan_variance ~/imu_static/imu_bag ~/imu_static/config/px6.mini.yaml
```

Finally, execute analysis node to generate imu.yaml file which includes the white and random walk noise information.

```
rosrun allan_variance_ros analysis.py --data ~/imu_static/imu_bag/allan_variance.csv
```

Result of imu.yaml file.

```
#Original

#Accelerometer
accelerometer_noise_density: 0.0009995551402804774 
accelerometer_random_walk: 2.6108694369610206e-05 

#Gyroscope
gyroscope_noise_density: 6.11871181672524e-05 
gyroscope_random_walk: 9.059705258699343e-07 

# white times five
# walks times ten

#Accelerometer
accelerometer_noise_density: 0.004997776 
accelerometer_random_walk: 2.6108694369610206e-04 

#Gyroscope
gyroscope_noise_density: 0.00030593559 
gyroscope_random_walk: 9.059705258699343e-06 


rostopic: '/mavros/imu/data_raw' #Make sure this is correct
update_rate: 200.0 #Make sure this is correct
```

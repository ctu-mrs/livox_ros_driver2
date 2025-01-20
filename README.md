# Livox ROS Driver 2

Livox ROS Driver 2 is the 2nd-generation driver package used to connect LiDAR products produced by Livox, applicable for ROS (Noetic recommended) and ROS2 (Foxy or Humble recommended).

**Note:**
This is a fork of the [original repository](https://github.com/Livox-SDK/livox_ros_driver2) with changes made for compatibility with the [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system).

**Disclaimer:**

As a debugging tool, Livox ROS Driver is not recommended for mass production but limited to test scenarios. You should optimize the code based on the original source to meet your various needs.

## Prerequisities

  * Ubuntu 18.04 for ROS Melodic (tested)
  * Ubuntu 20.04 for ROS Noetic and ROS2 Foxy (untested)
  * Ubuntu 22.04 for ROS2 Humble (untested)

## Install

Replace `workspace` and `ROS_VERSION` with the path to your ROS workspace and your ROS version:
```bash
WORKSPACE=$HOME/workspace
ROS_VERSION=ROS1 # (ROS1, ROS2)

cd ~/git && git clone git@github.com:ctu-mrs/livox_ros_driver2.git
cd $WORKSPACE/src && ln -sf ~/git/livox_ros_driver2 .
cd $WORKSPACE/src/livox_ros_driver2/installation && ./install.sh $ROS_VERSION 
```

## Set up your Livox sensor
Update the `livox_ros_driver2/config/MID360_config.json` with the IP address of your Livox sensor.
The IP is `192.168.1.1XX` where `XX` are the last two digits of the sensor's SN.

## Build
```bash
cd $WORKSPACE
catkin build livox_ros_driver2
```

**Note:** do not forget to source your `WORKSPACE` in your `.*rc` file.

## Launching

```bash
roslaunch livox_ros_driver2 mid360.launch xfer_format:=X
```
where `X` is `1` for `Point-LIO` compatibility and `0` for RViz visualization.

## Parametrizations

All internal parameters of livox_ros_driver2 are in the launch files.
Below are detailed descriptions of the three commonly used parameters:

| Parameter    | Detailed description                                                                                                                                                                                                                         | Default |
| ------------ | ------------------------------------------------------------                                                                                                                                                                                 | ------- |
| publish_freq | Set the frequency of the point cloud publishing.<br>Floating-point data type, recommended values: 5.0, 10.0, 20.0, 50.0, .... (100.0 Hz is the maximum).                                                                                     | 10.0    |
| multi_topic  | If the LiDAR device has an independent topic to publish pointcloud data<br>0 -- All LiDAR devices use the same topic to publish pointcloud data<br>1 -- Each LiDAR device has its own topic to publish point cloud data                                                             | 0       |
| xfer_format  | Set pointcloud format<br>0 -- sensor_msgs::PointCloud2 (Livox custom PointXYZRTLT format)<br>1 -- Livox custom format (Point-LIO compatiblerr) <br>2 -- sensor_msgs::PointCloud2 (pcl::PointXYZI) | 0       |

**Note :**  Other parameters not mentioned in this table are not suggested to be changed unless fully understood.

### Point cloud types:

[0] Livox custom PointXYZRTLT format

```c
float32 x               # X axis (m)
float32 y               # Y axis (m)
float32 z               # Z axis (m)
float32 intensity       # the value is reflectivity, 0.0~255.0
uint8   tag             # Livox tag
uint8   line            # laser number in lidar
float64 timestamp       # timestamp of point
```

**Note :** The number of points in the frame may be different but each point provides a timestamp.

[1] Livox custom format (Point-LIO compatible)

```c
std_msgs/Header header     # ROS standard message header
uint64          timebase   # The time of first point
uint32          point_num  # Total number of pointclouds
uint8           lidar_id   # Lidar device id number
uint8[3]        rsvd       # Reserved use
CustomPoint[]   points     # Pointcloud data
```

with each point being specified as

```c
uint32  offset_time     # offset time relative to the base time
float32 x               # X axis (m)
float32 y               # Y axis (m)
float32 z               # Z axis (m)
uint8   reflectivity    # reflectivity, 0~255
uint8   tag             # Livox tag
uint8   line            # laser number in lidar
```

[2] sensor_msgs::PointCloud2 ([pcl::PointXYZI](http://pointclouds.org/documentation/structpcl_1_1_point_x_y_z_i.html))

### Sensor configurations
For `.json` configurations and mulit-sensor connections, please refer to the [original Livox-SDK repository](https://github.com/Livox-SDK/livox_ros_driver2).

## TODO
- [ ] Nodeletize the I/O
- [ ] Improve param loading through `mrs_lib::ParamLoader`
- [ ] Make the Point-LIO specific format visualizable in Rviz
- [ ] `pub_handler.cpp`: profile if in-for-loop mutexing in `ProcessSphericalPoint()` is slow and potentially replace
- [ ] invalid points: make it work for other point types (custom, XYZI)
- [X] invalid points: invalidity check works only in spherical coordinate reading -> add check/warning for this to warn the user

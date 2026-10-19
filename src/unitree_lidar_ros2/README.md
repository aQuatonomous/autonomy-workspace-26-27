# Unitree Lidar ROS2

## Introduction
This package is a ROS2 package, which is specially used for running Unitree LiDAR products.


The functions that this package can provide includes:
- Parse the raw data transmitted from the lidar hardware, and convert it into pointcloud and IMU data
- Publish the pointcloud data to a ROS2 topic
- Publish the IMU data to a ROS2 topic
- In addition, you can configure the relevant parameters of the lidar in the `launch.py`.

## Dependency
The dependencies include `PCL` and `ROS2`.

We have verified that this package can successfully run under this environment:
- `Ubuntu 20.04` 
- `ROS2 foxy`
- `PCL-1.10`
- `unitree_lidar_sdk`

You are suggested to configure an environment like this to run this package.

## Configuration

Connect your lidar to your computer with a USB cable, then confirm your serial port name for lidar:
```
$ ls /dev/ttyUSB*
/dev/ttyUSB0
```

The default serial port name is `/dev/ttyUSB0`.
If it is not the default one, you need to modify the configuration  in `unitree_lidar_ros2/launch/launch.py`
and change the `port` parameter to yours. For example
```
{'port': '/dev/ttyUSB0'},
```

You can leave other parameters in the configuration file with their default value.

If you have special needs such as changing the cloud topic name or IMU topic name, you are allowed to configure them in the `launch.py` file as well. 

The defalut cloud topic and its frame name is:
- topic: "unilidar/cloud"
- frame: "unilidar_lidar"

The defalut IMU topic and its frame name is:
- topic: "unilidar/imu"
- frame: "unilidar_imu"

## Build (from unified root workspace)

This package now lives in the **root** ROS 2 workspace under `src/unitree_lidar_ros2`, and the C++ SDK is vendored at `src/unitree_lidar_sdk`:

```bash
cd ~/Repos/School/autonomy-ws-25-26
source /opt/ros/jazzy/setup.bash
./build.sh          # builds all packages, including unitree_lidar_ros2
source install/setup.bash
```

## Run

Then you need to source this ROS packege environment and then directly run the launch.py file:
```
source install/setup.bash

ros2 launch unitree_lidar_ros2 launch.py
```

To visualize the cloud in RViz2 (from mapping workspace after sourcing install):
```
rviz2 -d install/unitree_lidar_ros2/share/unitree_lidar_ros2/view.rviz
```

In the Rviz window, you will see our lidar pointcloud like this:

![img](./docs/cloud.png)

You can change the `Fixed Frame` to the imu frame `unilidar_imu`, so that you can view the IMU quaternion vector.
#!/bin/bash
# Terminal 2: Gazebo -> ROS 2 bridges, using the real sensors' topic names and frames.
#   RoboSense Airy 96:  /rslidar_points (frame rslidar), /rslidar_imu_data
#   3 cameras:          /camera0/image_raw (right), /camera1/image_raw (centre), /camera2/image_raw (left)
#   ground truth:       /sim/ground_truth/odom (debugging only; the stack must not use it)
. "$(dirname "${BASH_SOURCE[0]}")/env.sh"
sim_load_course || true
sim_wait_for_gazebo
exec ros2 run ros_gz_bridge parameter_bridge \
  '/lidar_topic/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked' \
  '/rslidar_imu@sensor_msgs/msg/Imu[gz.msgs.IMU' \
  '/camera1_topic@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/camera2_topic@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/camera3_topic@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/model/ourboat/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  --ros-args \
  -r /lidar_topic/points:=/rslidar_points \
  -r /rslidar_imu:=/rslidar_imu_data \
  -r /camera1_topic:=/camera0/image_raw \
  -r /camera2_topic:=/camera1/image_raw \
  -r /camera3_topic:=/camera2/image_raw \
  -r /model/ourboat/odometry:=/sim/ground_truth/odom

cmake_minimum_required(VERSION 3.5)
project(pointcloud_filters)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate service and message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetCloudWindow.srv"
  "msg/BuoyDetection.msg"
  "msg/BuoyDetectionArray.msg"
  "msg/TrackedBuoy.msg"
  "msg/TrackedBuoyArray.msg"
  "msg/FusedBuoy.msg"
  "msg/FusedBuoyArray.msg"
  DEPENDENCIES sensor_msgs std_msgs builtin_interfaces
)

# Install Python scripts
install(PROGRAMS
  pointcloud_filters/lidar_range_filter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lidar_range_filter
)

# Install buoy_detector separately (can't have multiple files with RENAME)
install(PROGRAMS
  pointcloud_filters/buoy_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME buoy_detector
)

# Install buoy_visualizer separately
install(PROGRAMS
  pointcloud_filters/buoy_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME buoy_visualizer
)

# Install buoy_tracker separately
install(PROGRAMS
  pointcloud_filters/buoy_tracker.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME buoy_tracker
)

# Install tracked_buoy_visualizer separately
install(PROGRAMS
  pointcloud_filters/tracked_buoy_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tracked_buoy_visualizer
)

# Install Python module (without ament_python_install_package to avoid duplication)
install(DIRECTORY pointcloud_filters/
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install RViz config for buoy_pipeline.launch.py
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()

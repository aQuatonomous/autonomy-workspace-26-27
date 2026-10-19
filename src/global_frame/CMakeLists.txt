cmake_minimum_required(VERSION 3.5)
project(global_frame)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoatPose.msg"
  "msg/GlobalDetection.msg"
  "msg/GlobalDetectionArray.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

install(PROGRAMS
  global_frame/boat_state_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME boat_state_node
)

install(PROGRAMS
  global_frame/detection_to_global_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME detection_to_global_node
)

install(DIRECTORY global_frame/
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

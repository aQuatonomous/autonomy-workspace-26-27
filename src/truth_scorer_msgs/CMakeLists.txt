cmake_minimum_required(VERSION 3.5)
project(truth_scorer_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ScoredObject.msg"
  "msg/ScoredObjectArray.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_package()

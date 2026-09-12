cmake_minimum_required(VERSION 3.8)
project(br_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Block.msg"
  "msg/BlockArray.msg"
  "msg/RobotStatus.msg"
  "msg/GripperCmd.msg"
  "msg/BuildAction.msg"
  "msg/Violation.msg"
  "msg/RobotPose.msg"
  "msg/MustikaPose.msg"
  "msg/TowerState.msg"
  "msg/TowerArray.msg"
  "msg/DetectedBlock.msg"
  "msg/DetectedBlockArray.msg"
  DEPENDENCIES geometry_msgs
)

ament_package()

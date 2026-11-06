cmake_minimum_required(VERSION 3.16)
project(sparcbot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelState.msg"
  "msg/WheelStates.msg"
  "msg/WheelCommand.msg"
  "msg/WheelVelocities.msg"
  "msg/WheelControllerOutput.msg"
  "msg/WheelControllerDebug.msg"
  "srv/SetPose.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

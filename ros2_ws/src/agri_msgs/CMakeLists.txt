cmake_minimum_required(VERSION 3.8)
project(agri_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DroneBeacon.msg"
  "msg/PanelEvent.msg"
  "msg/PanelState.msg"
  "msg/Heartbeat.msg"
)
ament_package()

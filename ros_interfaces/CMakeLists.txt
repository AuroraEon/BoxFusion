cmake_minimum_required(VERSION 3.8)
project(ros_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ExplainConnection.srv"
  "srv/GetPublicationDiagnostics.srv"
  "srv/GetTopology.srv"
  "srv/GetRoomPublicationState.srv"
  "srv/GetWorldSnapshot.srv"
  "srv/ResolveObjectRoom.srv"
  "srv/RouteToObject.srv"
  "srv/RouteToRoom.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

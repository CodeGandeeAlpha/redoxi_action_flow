cmake_minimum_required(VERSION 3.8)
project(psg_private_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(redoxi_public_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Person.msg"
  "msg/Persons.msg"
  "msg/PersonTrajectories.msg"
  "msg/PersonTrajectory.msg"
  "msg/TrajectoryEvent.msg"
  "msg/TrajectoryEvents.msg"
  "msg/PsgDocument.msg"
  "action/ProcessPsgDocument.action"
  "action/ProcessTrackTargets.action"

  DEPENDENCIES geometry_msgs redoxi_public_msgs std_msgs unique_identifier_msgs
)

ament_package()

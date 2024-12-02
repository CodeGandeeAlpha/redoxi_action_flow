cmake_minimum_required(VERSION 3.8)
project(redoxi_public_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
# find_package(rclpy REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/AnyData.msg"
  "msg/Box2.msg"
  "msg/CacheData.msg"
  "msg/Control.msg"
  "msg/Detection.msg"
  "msg/FeedbackResponse.msg"
  "msg/Frame.msg"
  "msg/FrameMetadata.msg"
  "msg/Keypoints.msg"
  "msg/ReturnResponse.msg"
  "msg/SharedMemoryToken.msg"
  "msg/TrackTarget.msg"
  "msg/TaskMetadata.msg"
  "action/ProcessBodyPoses.action"
  "action/ProcessDetections.action"
  "action/ProcessDetectionsByFrame.action"
  "action/ProcessFrame.action"
  "action/ProcessKeypointsByDets.action"
  DEPENDENCIES
  geometry_msgs
  std_msgs
  unique_identifier_msgs
  sensor_msgs)

ament_package()

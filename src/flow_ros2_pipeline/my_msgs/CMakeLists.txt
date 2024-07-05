cmake_minimum_required(VERSION 3.8)
project(my_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(pg_for_ros2 REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Image1080p.msg"
  "msg/Detection.msg"
  "msg/Detections.msg"
  "msg/Person.msg"
  "msg/Persons.msg"
  "msg/PersonTrajectory.msg"
  "msg/PersonTrajectories.msg"
  "msg/Pose.msg"
  "msg/Poses.msg"
  "msg/TrajectoryEvent.msg"
  "msg/TrajectoryEvents.msg"

  DEPENDENCIES std_msgs
  # ADD_LINTER_TESTS
)

ament_python_install_package(${PROJECT_NAME})

# Declare a C++ library
add_library(converter SHARED
  src/converter.cpp
)
# target_include_directories(converter PUBLIC include)
target_include_directories(converter
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(converter PUBLIC
  rclcpp
  pg_for_ros2
  rosidl_default_runtime  # Add this line to ensure rosidl generated files are found
)

# only required when you want to use interfaces
# in the same package as the one in which they are defined
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

message("----------------- ${cpp_typesupport_target}")

target_link_libraries(converter PUBLIC "${cpp_typesupport_target}")


install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS converter
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_export_include_directories(include)
# ament_export_libraries(converter)
# ament_export_dependencies(rclcpp rosidl_default_generators)
ament_package()
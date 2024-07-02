cmake_minimum_required(VERSION 3.8)
project(my_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(passengerflow REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Image1080p.msg"
  "msg/Detection.msg"
  "msg/Detections.msg"
  "msg/Person.msg"
  "msg/Persons.msg"
  "msg/PersonTrajectory.msg"
  "msg/PersonTrajectories.msg"
  "msg/Pose.msg"
  "msg/TrajectoryEvent.msg"
  "msg/TrajectoryEvents.msg"

  # DEPENDENCIES builtin_interfaces
  # ADD_LINTER_TESTS
)

include_directories(include)


## Declare a C++ library
add_library(converter SHARED
  src/converter.cpp
)
ament_target_dependencies(converter rclcpp passengerflow)

install(
  TARGETS converter
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
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

ament_export_include_directories(include)
ament_export_libraries(converter)
ament_export_dependencies(rclcpp passengerflow rosidl_default_generators)
ament_package()
cmake_minimum_required(VERSION 3.8)
project(example)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(psg_actions REQUIRED)
find_package(psg_services REQUIRED)
find_package(psg_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories(include)

install(DIRECTORY include/
  DESTINATION include)

add_executable(test_client
  src/test_client.cpp
)
add_executable(test_server
  src/test_server.cpp
)


ament_target_dependencies(test_client
  rclcpp
  rclcpp_action
  rclpy
  std_msgs
  psg_actions
  psg_services
  psg_common
  sensor_msgs
)

ament_target_dependencies(test_server
  rclcpp
  rclcpp_action
  rclpy
  std_msgs
  psg_actions
  psg_services
  psg_common
  sensor_msgs
)

ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/test_py_client.py
  ${PROJECT_NAME}/test_py_server.py
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS test_client test_server
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()

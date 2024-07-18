cmake_minimum_required(VERSION 3.8)
project(master_node)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(psg_services REQUIRED)
find_package(psg_actions REQUIRED)
find_package(psg_common REQUIRED)
find_package(psg_private_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)


find_package(vineyard REQUIRED)

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


include_directories(include ${VINEYARD_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}_node src/main.cpp src/master_node.cpp src/memory_registry.cpp src/master_node_types.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclpy
  std_msgs
  psg_services
  psg_actions
  psg_common
  rclcpp_action
  psg_private_msgs
)

target_link_libraries(${PROJECT_NAME}_node ${VINEYARD_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})
# install(DIRECTORY include/
#   DESTINATION include)

# ament_export_include_directories(include)
# ament_export_dependencies(std_msgs)
ament_package()
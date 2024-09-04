cmake_minimum_required(VERSION 3.8)
project(psg_count)

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

find_package(Boost REQUIRED)
find_package(vineyard REQUIRED)
find_package(RedoxiTrack REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PassengerFlow REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${VINEYARD_INCLUDE_DIRS})

install(DIRECTORY include/
  DESTINATION include)

add_executable(psg_count_node
  src/psg_count_node.cpp
  src/psg_count.cpp
  src/psg_count_types.cpp
)

ament_target_dependencies(psg_count_node PUBLIC
  rclcpp
  rclcpp_action
  rclpy
  std_msgs
  psg_actions
  psg_services
  psg_common
  sensor_msgs
)

target_link_libraries(psg_count_node PUBLIC ${Boost_LIBRARIES} ${VINEYARD_LIBRARIES} ${OpenCV_LIBS} RedoxiTrack::RedoxiTrack PassengerFlow::PassengerFlow)

install(TARGETS psg_count_node
DESTINATION lib/${PROJECT_NAME})

# ament_python_install_package(${PROJECT_NAME})

# # Install Python executables
# install(PROGRAMS
#   ${PROJECT_NAME}/rtm_pose_detector_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# # Environment hook to set PYTHONPATH
# ament_environment_hooks(
#   "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in"
# )

# # Install environment hook
# install(FILES env-hooks/${PROJECT_NAME}.dsv
#   DESTINATION share/${PROJECT_NAME}/env-hooks)

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

cmake_minimum_required(VERSION 3.8)
project(redoxi_ros_logging)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)

add_library(redoxi_ros_logging src/redoxi_ros_logging.cpp)
add_library(redoxi_ros_logging::redoxi_ros_logging ALIAS redoxi_ros_logging)
target_compile_features(redoxi_ros_logging PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(redoxi_ros_logging PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  redoxi_ros_logging
  PUBLIC
  rclcpp
  rcpputils
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(redoxi_ros_logging PRIVATE "REDOXI_ROS_LOGGING_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS redoxi_ros_logging
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

ament_export_dependencies(
  rclcpp
  rcpputils
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  redoxi_ros_logging
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(psg_tracker_sink)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(psg_document_sink REQUIRED)
find_package(psg_common REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# target_compile_features(psg_tracker_sink PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# target_include_directories(psg_tracker_sink PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# target_compile_definitions(psg_tracker_sink PRIVATE "PSG_TRACKER_SINK_BUILDING_LIBRARY")

# ament_target_dependencies(
#   psg_tracker_sink  PUBLIC
#   psg_document_sink
#   psg_common
#   rclcpp
#   rclpy
# )


ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/psg_tracker_sink_node.py
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

# ament_export_targets(
#   export_${PROJECT_NAME}
# )

ament_export_dependencies(
  rclcpp
  rclpy
  psg_document_sink
  psg_common
  RedoxiTrack
)

ament_package()

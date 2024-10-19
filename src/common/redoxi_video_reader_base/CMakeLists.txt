cmake_minimum_required(VERSION 3.8)
project(redoxi_video_reader_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(redoxi_common_cpp REQUIRED)
find_package(redoxi_public_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(fmt REQUIRED)
find_package(TBB REQUIRED)

# Find Boost (header-only)
find_package(Boost REQUIRED)


set(source_files
  # src/redoxi_video_reader_base.cpp
  src/redoxi_video_reader_types.cpp
)

add_library(redoxi_video_reader_base ${source_files})
add_library(redoxi_video_reader_base::redoxi_video_reader_base ALIAS redoxi_video_reader_base)
target_compile_features(redoxi_video_reader_base PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(redoxi_video_reader_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(redoxi_video_reader_base PUBLIC TBB::tbb Boost::boost fmt::fmt)  # Link TBB to your target

ament_target_dependencies(
  redoxi_video_reader_base PUBLIC
  "rclcpp"
  "rclcpp_action"
  "redoxi_common_cpp"
  "redoxi_public_msgs"
  "sensor_msgs"
  "rcutils"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(redoxi_video_reader_base PRIVATE "REDOXI_VIDEO_READER_BASE_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS redoxi_video_reader_base
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

ament_export_include_directories(
  include
)
ament_export_libraries(
  redoxi_video_reader_base
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()

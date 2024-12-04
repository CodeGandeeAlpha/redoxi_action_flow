cmake_minimum_required(VERSION 3.8)
project(video_reader_from_url)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(redoxi_video_reader REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(video_reader_from_url src/video_reader_from_url.cpp)
add_library(video_reader_from_url::video_reader_from_url ALIAS video_reader_from_url)
target_compile_features(video_reader_from_url PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(video_reader_from_url PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(video_reader_from_url PRIVATE "VIDEO_READER_FROM_URL_BUILDING_LIBRARY")

ament_target_dependencies(
  video_reader_from_url PUBLIC
  redoxi_video_reader
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS video_reader_from_url
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
  video_reader_from_url
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  redoxi_video_reader
)

ament_package()

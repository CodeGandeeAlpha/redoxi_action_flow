cmake_minimum_required(VERSION 3.8)
project(psg_frame_det_source_sink)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(redoxi_video_reader REQUIRED)
find_package(redoxi_samples_lib REQUIRED)
find_package(redoxi_common_nodes REQUIRED)
find_package(psg_detector REQUIRED)
find_package(psg_document_sink REQUIRED)
find_package(psg_private_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(psg_frame_det_source_sink src/PSGFrameDetSourceSink.cpp src/PSGFrameDetSourceSinkTypes.cpp)
add_library(psg_frame_det_source_sink::psg_frame_det_source_sink ALIAS psg_frame_det_source_sink)
target_compile_features(psg_frame_det_source_sink PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(psg_frame_det_source_sink PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(psg_frame_det_source_sink PRIVATE "PSG_FRAME_DET_SOURCE_SINK_BUILDING_LIBRARY")

ament_target_dependencies(psg_frame_det_source_sink PUBLIC
  redoxi_samples_lib
  redoxi_video_reader
  redoxi_common_nodes
  psg_detector
  psg_document_sink
  psg_private_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS psg_frame_det_source_sink
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
  psg_frame_det_source_sink
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(
  redoxi_samples_lib
  redoxi_video_reader
  redoxi_common_nodes
  psg_detector
  psg_document_sink
  psg_private_msgs
)

ament_package()

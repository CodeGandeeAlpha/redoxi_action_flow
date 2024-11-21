cmake_minimum_required(VERSION 3.8)
project(redoxi_samples_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(redoxi_samples_lib REQUIRED)
find_package(redoxi_video_reader REQUIRED)
find_package(redoxi_common_nodes REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(cv_bridge REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(srcfiles 
  src/redoxi_samples_nodes.cpp
  src/generators/RandomFrameVideoGenerator.cpp
  src/generators/SimpleActionGenerator.cpp
  src/sinks/FrameRelayPublisher.cpp
  src/sinks/FrameRelayNode.cpp
  src/sinks/DetectionRelayNode.cpp
)

add_library(redoxi_samples_nodes ${srcfiles})
add_library(redoxi_samples_nodes::redoxi_samples_nodes ALIAS redoxi_samples_nodes)
target_compile_features(redoxi_samples_nodes PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(redoxi_samples_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(redoxi_samples_nodes PRIVATE "REDOXI_SAMPLES_NODES_BUILDING_LIBRARY")
target_link_libraries(redoxi_samples_nodes PRIVATE Boost::boost Boost::timer)

ament_target_dependencies(redoxi_samples_nodes PUBLIC 
  redoxi_samples_lib
  redoxi_video_reader
  redoxi_common_nodes
  cv_bridge
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS redoxi_samples_nodes
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
  redoxi_samples_lib
  redoxi_video_reader
  redoxi_common_nodes
  cv_bridge
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  redoxi_samples_nodes
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()

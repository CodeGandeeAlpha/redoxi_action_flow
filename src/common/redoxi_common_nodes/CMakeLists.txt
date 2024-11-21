cmake_minimum_required(VERSION 3.8)
project(redoxi_common_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(redoxi_extern_cpp REQUIRED)
find_package(redoxi_common_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(json_struct REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(cpp_required_to_find Boost)
foreach(pkg ${cpp_required_to_find})
  find_package(${pkg} REQUIRED)
endforeach()

set(cpp_required_to_link Boost::boost json_struct::json_struct)

set(srcfiles
  src/redoxi_common_nodes.cpp
  src/AsyncActionOutputPort.cpp
  src/AsyncActionInputPort.cpp
  src/base_nodes/BaseRosNode.cpp
  src/base_nodes/StartStopNode.cpp
)

add_library(redoxi_common_nodes ${srcfiles})
add_library(redoxi_common_nodes::redoxi_common_nodes ALIAS redoxi_common_nodes)
target_compile_features(redoxi_common_nodes PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(redoxi_common_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(redoxi_common_nodes PUBLIC ${cpp_required_to_link})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(redoxi_common_nodes PRIVATE "REDOXI_COMMON_NODES_BUILDING_LIBRARY")

ament_target_dependencies(redoxi_common_nodes PUBLIC
rclcpp rclcpp_action redoxi_extern_cpp redoxi_common_cpp cv_bridge)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS redoxi_common_nodes
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
  rclcpp_action
  redoxi_extern_cpp
  redoxi_common_cpp
  cv_bridge
  json_struct
  ${cpp_required_to_find}
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  redoxi_common_nodes
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()

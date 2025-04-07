cmake_minimum_required(VERSION 3.8)
project(redoxi_common_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(redoxi_common_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(json_struct REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(redoxi_cmake_utils REQUIRED)
# find_package(nav2_util REQUIRED)
# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)

set(cpp_required_to_find Boost nlohmann_json)
foreach(pkg ${cpp_required_to_find})
  find_package(${pkg} REQUIRED)
endforeach()

set(cpp_required_to_link 
    Boost::boost json_struct::json_struct 
    nlohmann_json::nlohmann_json)

set(srcfiles
    src/redoxi_common_nodes.cpp
    src/AsyncActionOutputPort.cpp
    src/AsyncActionInputPort.cpp
    src/base_nodes/BaseRosNode.cpp
    src/base_nodes/StartStopNode.cpp
    src/base_nodes/OpenCloseNode.cpp
    src/driver_nodes/DetectionDriver.cpp)

add_library(redoxi_common_nodes ${srcfiles})
add_library(redoxi_common_nodes::redoxi_common_nodes ALIAS redoxi_common_nodes)
target_compile_features(redoxi_common_nodes PUBLIC c_std_99 cxx_std_17
)# Require C99 and C++17
target_include_directories(
  redoxi_common_nodes
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(redoxi_common_nodes PUBLIC ${cpp_required_to_link})
target_precompile_headers(redoxi_common_nodes PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/_pch.hpp)

# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions(redoxi_common_nodes
                           PRIVATE "REDOXI_COMMON_NODES_BUILDING_LIBRARY")


# add_executable(redoxi_common_nodes_executable src/redoxi_common_nodes_components.cpp)
# ament_target_dependencies(
#   redoxi_common_nodes_executable
#   PUBLIC
#   rclcpp
#   rclcpp_action
#   redoxi_common_cpp
#   cv_bridge
#   rosidl_runtime_cpp
#   rclcpp_lifecycle
#   lifecycle_msgs
#   rclcpp_components)
# rclcpp_components_register_node(redoxi_common_nodes 
#   PLUGIN "redoxi_works::common_nodes::drivers::DetectionDriver"
#   EXECUTABLE redoxi_common_nodes_executable)

# add_library(redoxi_common_nodes_components src/redoxi_common_nodes_components.cpp)
# target_link_libraries(redoxi_common_nodes_components PUBLIC redoxi_common_nodes)
# rclcpp_components_register_node(redoxi_common_nodes_components 
#   PLUGIN "redoxi_works::common_nodes::drivers::DetectionDriver"
#   EXECUTABLE redoxi_common_nodes_executable)
# install(TARGETS redoxi_common_nodes_components
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)

create_composable_components(${PROJECT_NAME}
  SOURCE_FILE src/redoxi_common_nodes_components.cpp
  PLUGINS "redoxi_works::node_pack::detection::DetectionDriver"
  EXECUTABLES "node_pack_detection_DetectionDriver")

ament_target_dependencies(
  redoxi_common_nodes
  PUBLIC
  rclcpp
  rclcpp_action
  redoxi_common_cpp
  cv_bridge
  rosidl_runtime_cpp
  rclcpp_lifecycle
  lifecycle_msgs
  rclcpp_components)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS redoxi_common_nodes
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  rclcpp
  rclcpp_action
  redoxi_common_cpp
  cv_bridge
  json_struct
  rosidl_runtime_cpp
  rclcpp_lifecycle
  lifecycle_msgs
  rclcpp_components
  ${cpp_required_to_find})

ament_export_include_directories(include)
ament_export_libraries(redoxi_common_nodes)
ament_export_targets(export_${PROJECT_NAME})

ament_package()

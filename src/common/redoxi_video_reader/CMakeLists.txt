cmake_minimum_required(VERSION 3.8)
project(redoxi_video_reader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(redoxi_common_cpp REQUIRED)
find_package(redoxi_public_msgs REQUIRED)
find_package(redoxi_common_nodes REQUIRED)
find_package(redoxi_shared_memory REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(fmt REQUIRED)
find_package(TBB REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(redoxi_cmake_utils REQUIRED)

# Find Boost (header-only)
find_package(Boost REQUIRED)


set(source_files
  src/redoxi_video_reader.cpp
  src/base/v1/VideoReaderBase.cpp
  src/base/v1/VideoReaderBaseTypes.cpp
  src/v1/VideoSourceFromUrl.cpp
  src/v2/VideoSourceFromUrl.cpp
  src/base/v2/VideoReaderBase.cpp
  src/base/v2/VideoReaderBaseTypes.cpp
)

add_library(redoxi_video_reader ${source_files})
add_library(redoxi_video_reader::redoxi_video_reader ALIAS redoxi_video_reader)
target_compile_features(redoxi_video_reader PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(redoxi_video_reader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(redoxi_video_reader PUBLIC 
  TBB::tbb 
  Boost::boost 
  fmt::fmt
  nlohmann_json::nlohmann_json
)  # Link TBB to your target

ament_target_dependencies(
  redoxi_video_reader PUBLIC
  "rclcpp"
  "rclcpp_action"
  "redoxi_common_cpp"
  "redoxi_public_msgs"
  "sensor_msgs"
  "rcutils"
  "cv_bridge"
  "redoxi_common_nodes"
  "redoxi_shared_memory"
  "rclcpp_components"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(redoxi_video_reader PRIVATE "REDOXI_VIDEO_READER_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS redoxi_video_reader
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# create composable components
# macro(create_composable_components ProjectName)
#   cmake_parse_arguments(ARG "" "SOURCE_FILE" "PLUGINS;EXECUTABLES" ${ARGN})
  
#   # Set default source file if not provided
#   if(NOT ARG_SOURCE_FILE)
#     set(ARG_SOURCE_FILE "src/${ProjectName}_components.cpp")
#   endif()

#   # Create components library
#   add_library(${ProjectName}_components ${ARG_SOURCE_FILE})
#   target_link_libraries(${ProjectName}_components PUBLIC ${ProjectName})

#   # Register each plugin
#   list(LENGTH ARG_PLUGINS num_plugins)
#   math(EXPR last_idx "${num_plugins} - 1")
  
#   foreach(idx RANGE ${last_idx})
#     list(GET ARG_PLUGINS ${idx} plugin)
    
#     # Use provided executable name if available, otherwise generate one
#     if(DEFINED ARG_EXECUTABLES)
#       list(GET ARG_EXECUTABLES ${idx} exec_name)
#     else()
#       string(REPLACE "::" "_" exec_name "${plugin}")
#       set(exec_name "node_executable_${exec_name}")
#     endif()
    
#     rclcpp_components_register_node(${ProjectName}_components
#       PLUGIN "${plugin}"
#       EXECUTABLE ${exec_name}
#     )
#   endforeach()

#   # Install targets
#   install(TARGETS ${ProjectName}_components
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION bin)
# endmacro()

# Call the function with project-specific parameters
create_composable_components(redoxi_video_reader
  SOURCE_FILE "src/redoxi_video_reader_components.cpp"
  PLUGINS
    "redoxi_works::node_pack::video_readers::VideoSourceFromUrl"
  EXECUTABLES
    "node_pack_video_readers_VideoSourceFromUrl"
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
  redoxi_video_reader
)
ament_export_targets(
  export_${PROJECT_NAME}
)

# use these so that downstream packages can find the dependencies without explicitly listing them
ament_export_dependencies(
  rclcpp
  rclcpp_action
  redoxi_common_cpp
  redoxi_public_msgs
  sensor_msgs
  rcutils
  cv_bridge
  nlohmann_json
  Boost
  redoxi_common_nodes
  redoxi_shared_memory
  rclcpp_components
)


ament_package()

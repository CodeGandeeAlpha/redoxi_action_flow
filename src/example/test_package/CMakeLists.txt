# Minimum required version of CMake
cmake_minimum_required(VERSION 3.8)

# Project name
project(test_package)

# Add compiler options if using GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(spdlog REQUIRED)
find_package(TBB REQUIRED)

# Set dependencies
set(DEPENDENCIES
    psg_private_msgs
    redoxi_video_reader
    redoxi_samples_lib
    redoxi_samples_nodes)

set(additional_dependencies spdlog::spdlog TBB::tbb)

# Find each dependency
foreach(dependency ${DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

set(ALL_TEST_TARGETS "")

# Define options to control building of each target
option(BUILD_TEST_NODE "Build test_node target" OFF)
option(BUILD_TEST_PUBLIC_MSGS "Build test_public_msgs target" OFF)
option(BUILD_TEST_PRIVATE_MSGS "Build test_private_msgs target" OFF)
option(BUILD_TEST_V6D_STANDALONE "Build test_v6d_standalone target" OFF)
option(BUILD_TEST_TBB "Build test_tbb target" OFF)
option(BUILD_TEST_ASYNC_SINGLE_OVERWRITE
       "Build test_async_single_overwrite target" OFF)
option(BUILD_TEST_ASYNC_SINGLE_BUFFER "Build test_async_single_buffer target"
       OFF)
option(BUILD_TEST_TBB_TASK_MANAGEMENT "Build test_tbb_task_management target"
       OFF)
option(BUILD_TEST_VIDEO_READER_RANDOM "Build test_video_reader_random target"
       OFF)
option(BUILD_TEST_VIDEO_SINK "Build test_video_sink target" ON)
option(BUILD_TEST_SIMPLE_ACTION_GENERATOR "Build test_simple_action_generator target" OFF)
option(BUILD_TEST_SIMPLE_ACTION_GENERATOR_V2 "Build test_simple_action_generator_v2 target" ON)
option(BUILD_TEST_UUID_CONVERSION "Build test_uuid_conversion target" OFF)
option(BUILD_TEST_V2_JSON_EXAMPLES "Build test_v2_json_examples target" OFF)
option(BUILD_TEST_V2_VIDEO_SINK "Build test_v2_video_sink target" ON)
option(BUILD_TEST_V2_RANDOM_VIDEO_SOURCE "Build v2_random_video_source target" ON)

# Function to configure a target
function(configure_target target_name)
  add_executable(${target_name} src/${target_name}.cpp)
  target_compile_features(
    ${target_name} PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
  target_link_libraries(${target_name} PUBLIC ${additional_dependencies})
  ament_target_dependencies(${target_name} PUBLIC ${DEPENDENCIES})
  target_include_directories(
    ${target_name} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:include>)
  set(ALL_TEST_TARGETS
      ${ALL_TEST_TARGETS} ${target_name}
      PARENT_SCOPE)
endfunction()

if(BUILD_TEST_V2_RANDOM_VIDEO_SOURCE)
  configure_target(v2_random_video_source)
endif()

if(BUILD_TEST_V2_VIDEO_SINK)
  configure_target(v2_video_sink)
endif()

if(BUILD_TEST_UUID_CONVERSION)
  configure_target(test_uuid_conversion)
endif()

if(BUILD_TEST_V2_JSON_EXAMPLES)
  configure_target(v2_json_examples)
endif()

if(BUILD_TEST_SIMPLE_ACTION_GENERATOR_V2)
  configure_target(v2_simple_action)
endif()

if(BUILD_TEST_SIMPLE_ACTION_GENERATOR)
  configure_target(test_simple_action_generator)
endif()

if(BUILD_TEST_VIDEO_SINK)
  configure_target(test_video_sink)
endif()

if(BUILD_TEST_VIDEO_READER_RANDOM)
  configure_target(test_video_reader_random)
endif()

# Configure targets based on options
if(BUILD_TEST_NODE)
  configure_target(test_node)
endif()

if(BUILD_TEST_PUBLIC_MSGS)
  configure_target(test_public_msgs)
endif()

if(BUILD_TEST_PRIVATE_MSGS)
  configure_target(test_private_msgs)
endif()

if(BUILD_TEST_V6D_STANDALONE)
  configure_target(test_v6d_standalone)
endif()

if(BUILD_TEST_TBB)
  configure_target(test_tbb)
endif()

if(BUILD_TEST_ASYNC_SINGLE_OVERWRITE)
  configure_target(test_async_single_overwrite)
endif()

if(BUILD_TEST_ASYNC_SINGLE_BUFFER)
  configure_target(test_async_single_buffer)
endif()

if(BUILD_TEST_TBB_TASK_MANAGEMENT)
  configure_target(test_tbb_task_management)
endif()

# Install targets
install(TARGETS ${ALL_TEST_TARGETS} DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/)

# Finalize ament package
ament_package()

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
  rclcpp
  std_msgs
  geometry_msgs
  redoxi_public_msgs
  psg_private_msgs
  unique_identifier_msgs
  redoxi_common_cpp
)

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
option(BUILD_TEST_TBB "Build test_tbb target" ON)

# Function to configure a target
function(configure_target target_name)
  add_executable(${target_name} src/${target_name}.cpp)
  target_compile_features(${target_name} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  target_link_libraries(${target_name} PUBLIC ${additional_dependencies})
  ament_target_dependencies(${target_name} PUBLIC ${DEPENDENCIES})
  target_include_directories(${target_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  set(ALL_TEST_TARGETS ${ALL_TEST_TARGETS} ${target_name} PARENT_SCOPE)
endfunction()

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

# Install targets
install(TARGETS ${ALL_TEST_TARGETS}
  DESTINATION lib/${PROJECT_NAME})

# Finalize ament package
ament_package()

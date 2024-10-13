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

set(additional_dependencies spdlog::spdlog)

# Find each dependency
foreach(dependency ${DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

# Add executable for test_node
add_executable(test_node src/test_node.cpp)
target_include_directories(test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(test_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(test_node PUBLIC ${additional_dependencies})
ament_target_dependencies(test_node PUBLIC ${DEPENDENCIES})

# Add executable for test_public_msgs
add_executable(test_public_msgs src/test_public_msgs.cpp)
target_compile_features(test_public_msgs PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(test_public_msgs PUBLIC ${DEPENDENCIES})
target_link_libraries(test_public_msgs PUBLIC ${additional_dependencies})

# Add executable for test_private_msgs
add_executable(test_private_msgs src/test_private_msgs.cpp)
target_compile_features(test_private_msgs PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(test_private_msgs PUBLIC ${additional_dependencies})
ament_target_dependencies(test_private_msgs PUBLIC ${DEPENDENCIES})

# Add executable for test_v6d
add_executable(test_v6d_standalone src/test_v6d_standalone.cpp)
target_compile_features(test_v6d_standalone PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(test_v6d_standalone PUBLIC ${additional_dependencies})
ament_target_dependencies(test_v6d_standalone PUBLIC ${DEPENDENCIES})

# Update the install targets to include test_v6d
install(TARGETS test_v6d_standalone
  DESTINATION lib/${PROJECT_NAME})

# Install targets
install(TARGETS test_public_msgs test_node test_private_msgs test_v6d_standalone
  DESTINATION lib/${PROJECT_NAME})

# Finalize ament package
ament_package()

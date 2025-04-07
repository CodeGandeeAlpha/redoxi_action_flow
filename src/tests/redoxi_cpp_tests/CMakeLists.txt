cmake_minimum_required(VERSION 3.8)
project(redoxi_cpp_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(TBB REQUIRED)
find_package(xtensor REQUIRED)

# dependencies for ros packages
set(ROS_DEPENDENCIES rclcpp yolo8_series)

set(ADDITIONAL_DEPENDENCIES spdlog::spdlog TBB::tbb xtensor)

option(BUILD_ONNX_YOLO8DET "Build onnx yolo8det example" ON)

# Find each dependency
foreach(dependency ${ROS_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

set(ALL_EXAMPLE_TARGETS "")

# Function to configure a target
function(configure_target target_name)
  # add executable, the target name is the same as the source file name
  add_executable(${target_name} src/${target_name}.cpp)
  target_compile_features(
    ${target_name} PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17

  # link to all ros dependencies
  ament_target_dependencies(${target_name} PUBLIC ${ROS_DEPENDENCIES})

  # link to all additional dependencies
  target_link_libraries(${target_name} PUBLIC ${ADDITIONAL_DEPENDENCIES})

  # include directories
  target_include_directories(
    ${target_name} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:include>)

  # precompile headers
  target_precompile_headers(
    ${target_name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/_pch.hpp)

  # global directories defined using macro
  target_compile_definitions(
    ${target_name}
    PUBLIC
      -DTEST_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../tmp/models"
      -DTEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../data"
      -DTEST_OUTPUT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../tmp/output")

  # get DISPLAY env variable from os, and then set it to the target Get DISPLAY
  # environment variable and set as compile definition if(DEFINED ENV{DISPLAY})
  # target_compile_definitions(${target_name} PUBLIC
  # -DDISPLAY_ENV_VAR="$ENV{DISPLAY}") endif()

  set(ALL_EXAMPLE_TARGETS
      ${ALL_EXAMPLE_TARGETS} ${target_name}
      PARENT_SCOPE)
endfunction()

if(BUILD_ONNX_YOLO8DET)
  configure_target(onnx_yolo8det)
endif()

# install targets
install(TARGETS ${ALL_EXAMPLE_TARGETS} DESTINATION lib/${PROJECT_NAME})

# install launch files
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})

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

ament_package()

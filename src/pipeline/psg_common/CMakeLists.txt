cmake_minimum_required(VERSION 3.8)
project(psg_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(psg_private_msgs REQUIRED)
find_package(redoxi_public_msgs REQUIRED)
find_package(PassengerFlow REQUIRED)
find_package(vineyard REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(srcfiles
  src/psg_common.cpp
  src/msg_converter.cpp
)

add_library(psg_common ${srcfiles})
add_library(psg_common::psg_common ALIAS psg_common)
target_compile_features(psg_common PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(psg_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(psg_common PRIVATE "PSG_COMMON_BUILDING_LIBRARY")

# 配置所有依赖
ament_target_dependencies(
  psg_common PUBLIC
  rclcpp
  rclpy
  psg_private_msgs
  redoxi_public_msgs
)

target_include_directories(psg_common PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${VINEYARD_INCLUDE_DIRS}
)

target_link_libraries(psg_common PUBLIC
  ${OpenCV_LIBS}
  Eigen3::Eigen
  PassengerFlow::PassengerFlow
  ${VINEYARD_LIBRARIES}
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS psg_common
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_python_install_package(${PROJECT_NAME})
# # Install Python executables
# install(PROGRAMS
#   ${PROJECT_NAME}/ddq_detector_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 导出所有依赖
ament_export_include_directories(
  include
)

ament_export_libraries(
  psg_common
)

ament_export_dependencies(
  rclcpp
  rclpy
  psg_private_msgs
  redoxi_public_msgs
  PassengerFlow
  vineyard
  OpenCV
  Eigen3
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()

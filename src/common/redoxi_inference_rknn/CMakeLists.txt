cmake_minimum_required(VERSION 3.8)
project(redoxi_inference_rknn)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)

add_library(redoxi_inference_rknn src/redoxi_inference_rknn.cpp)
add_library(redoxi_inference_rknn::redoxi_inference_rknn ALIAS redoxi_inference_rknn)
target_compile_features(redoxi_inference_rknn PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(redoxi_inference_rknn PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(
  redoxi_inference_rknn
  PUBLIC
  "rclcpp"
)

if(NOT DEFINED RKNPU_ROOT)
  message(FATAL_ERROR "RKNPU_ROOT environment variable is not set. Please set RKNPU_ROOT to point to your RKNN NPU SDK installation directory.")
endif()

set(RKNN_INCLUDE_DIRS ${RKNPU_ROOT}/runtime/Linux/librknn_api/include)
set(RKNN_LIBRARIES ${RKNPU_ROOT}/runtime/Linux/librknn_api/aarch64/librknnrt.so)

target_include_directories(redoxi_inference_rknn PUBLIC ${RKNN_INCLUDE_DIRS})
target_link_libraries(redoxi_inference_rknn PUBLIC ${RKNN_LIBRARIES})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(redoxi_inference_rknn PRIVATE "REDOXI_INFERENCE_RKNN_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS redoxi_inference_rknn
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
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  redoxi_inference_rknn
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()

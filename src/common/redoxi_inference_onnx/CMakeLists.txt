cmake_minimum_required(VERSION 3.8)
project(redoxi_inference_onnx)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(redoxi_inference REQUIRED)
find_package(redoxi_common_cpp REQUIRED)

find_package(Boost REQUIRED)

add_library(redoxi_inference_onnx 
  src/redoxi_inference_onnx.cpp
  src/OnnxModelInference.cpp
  src/OnnxPortData.cpp
  )
add_library(redoxi_inference_onnx::redoxi_inference_onnx ALIAS redoxi_inference_onnx)
target_compile_features(redoxi_inference_onnx PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(redoxi_inference_onnx PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  redoxi_inference_onnx PUBLIC
  "rclcpp"
  "redoxi_inference"
  "redoxi_common_cpp"
)

target_link_libraries(redoxi_inference_onnx PUBLIC Boost::boost)

# configure for onnx runtime
set(onnxruntime_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../tmp/onnxruntime/onnxruntime-linux-x64-gpu-1.20.0/lib64/cmake/onnxruntime)
find_package(onnxruntime REQUIRED)
target_link_libraries(redoxi_inference_onnx PUBLIC onnxruntime::onnxruntime)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(redoxi_inference_onnx PRIVATE "REDOXI_INFERENCE_ONNX_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS redoxi_inference_onnx
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
  redoxi_inference
  redoxi_common_cpp
  Boost
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  redoxi_inference_onnx
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()

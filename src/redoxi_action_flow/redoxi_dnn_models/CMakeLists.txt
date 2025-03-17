cmake_minimum_required(VERSION 3.8)
project(redoxi_dnn_models)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(redoxi_common_cpp REQUIRED)
find_package(redoxi_inference REQUIRED)
find_package(sensor_msgs REQUIRED)

# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)

add_library(redoxi_dnn_models
  src/redoxi_dnn_models.cpp
  src/yolo8/Yolo8PoseModel.cpp
  src/yolo8/Yolo8Preprocessor.cpp
  src/yolo8/Yolo8Postprocessor.cpp
  src/yolo8/Yolo8DetectionModel.cpp
  src/yolo8/Yolo8ModelBase.cpp
  src/message_conversion.cpp
  src/visualizations.cpp)
  
add_library(redoxi_dnn_models::redoxi_dnn_models ALIAS redoxi_dnn_models)
target_compile_features(
  redoxi_dnn_models PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
target_include_directories(
  redoxi_dnn_models
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions(redoxi_dnn_models
                           PRIVATE "REDOXI_DNN_MODELS_BUILDING_LIBRARY")

target_precompile_headers(redoxi_dnn_models PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/_pch.hpp)

ament_target_dependencies(redoxi_dnn_models PUBLIC redoxi_common_cpp
                          redoxi_inference sensor_msgs)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS redoxi_dnn_models
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

ament_export_dependencies(redoxi_common_cpp redoxi_inference sensor_msgs)
ament_export_include_directories(include)
ament_export_libraries(redoxi_dnn_models)
ament_export_targets(export_${PROJECT_NAME})

ament_package()

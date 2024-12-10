cmake_minimum_required(VERSION 3.8)
project(yolo8_series)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(redoxi_inference REQUIRED)
find_package(redoxi_dnn_models REQUIRED)
find_package(redoxi_common_nodes REQUIRED)
find_package(json_struct REQUIRED)
find_package(ament_cmake_python REQUIRED)

# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)

add_library(yolo8_series src/yolo8_series.cpp
                         src/bodypose/Yolo8BodyPoseNode.cpp)
add_library(yolo8_series::yolo8_series ALIAS yolo8_series)
target_compile_features(yolo8_series PUBLIC c_std_99 cxx_std_17) # Require C99
                                                                 # and C++17
target_include_directories(
  yolo8_series PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                      $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions(yolo8_series PRIVATE "YOLO8_SERIES_BUILDING_LIBRARY")

ament_target_dependencies(yolo8_series PUBLIC redoxi_inference
                          redoxi_dnn_models redoxi_common_nodes json_struct)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS yolo8_series
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# install python modules
ament_python_install_package(${PROJECT_NAME})

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

ament_export_dependencies(redoxi_inference redoxi_dnn_models
                          redoxi_common_nodes json_struct)

ament_export_include_directories(include)
ament_export_libraries(yolo8_series)
ament_export_targets(export_${PROJECT_NAME})

ament_package()

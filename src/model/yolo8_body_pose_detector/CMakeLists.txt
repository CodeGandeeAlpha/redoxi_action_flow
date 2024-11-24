cmake_minimum_required(VERSION 3.8)
project(yolo8_body_pose_detector)

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

# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)

add_library(
  yolo8_body_pose_detector
  src/yolo8_body_pose_detector.cpp src/Yolo8BodyPoseDetectorNode.cpp
  src/Yolo8BodyPoseDetectorTypes.cpp)

add_library(yolo8_body_pose_detector::yolo8_body_pose_detector ALIAS
            yolo8_body_pose_detector)
target_compile_features(yolo8_body_pose_detector PUBLIC c_std_99 cxx_std_17
)# Require C99 and C++17
target_include_directories(
  yolo8_body_pose_detector
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions(yolo8_body_pose_detector
                           PRIVATE "YOLO8_BODY_POSE_DETECTOR_BUILDING_LIBRARY")

ament_target_dependencies(yolo8_body_pose_detector PUBLIC redoxi_inference
                          redoxi_dnn_models redoxi_common_nodes json_struct)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS yolo8_body_pose_detector
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_executable(yolo8_body_pose_detector_node
               src/yolo8_body_pose_detector_node.cpp)
target_include_directories(
  yolo8_body_pose_detector_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
target_link_libraries(yolo8_body_pose_detector_node yolo8_body_pose_detector)

install(TARGETS yolo8_body_pose_detector_node DESTINATION lib/${PROJECT_NAME})

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
ament_export_libraries(yolo8_body_pose_detector)
ament_export_targets(export_${PROJECT_NAME})

ament_package()

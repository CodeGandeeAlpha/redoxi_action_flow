cmake_minimum_required(VERSION 3.8)
project(psg_all_detector_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(psg_document_sink REQUIRED)
find_package(psg_common REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(pipeline_srcfiles
  src/Pipeline.cpp
  src/Driver.cpp
)

add_library(psg_all_detector_cpp ${pipeline_srcfiles})
add_library(psg_all_detector_cpp::psg_all_detector_cpp ALIAS psg_all_detector_cpp)
target_compile_features(psg_all_detector_cpp PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(psg_all_detector_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(psg_all_detector_cpp PRIVATE "PSG_ALL_DETECTOR_CPP_BUILDING_LIBRARY")

ament_target_dependencies(
  psg_all_detector_cpp  PUBLIC
  psg_document_sink
  psg_common
  rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS psg_all_detector_cpp
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
  include
)
ament_export_libraries(
  psg_all_detector_cpp
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  psg_document_sink
  psg_common
)

ament_package()

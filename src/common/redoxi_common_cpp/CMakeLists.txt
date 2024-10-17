cmake_minimum_required(VERSION 3.8)
project(redoxi_common_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(vineyard REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(fmt REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(source_files src/redoxi_common_cpp.cpp src/redoxi_v6d.cpp src/redoxi_ros_util.cpp)
set(required_dependencies ${VINEYARD_LIBRARIES} ${OpenCV_LIBS} fmt::fmt)
set(required_includes ${OpenCV_INCLUDE_DIRS} ${VINEYARD_INCLUDE_DIRS})

add_library(redoxi_common_cpp ${source_files})
add_library(redoxi_common_cpp::redoxi_common_cpp ALIAS redoxi_common_cpp)
target_compile_features(redoxi_common_cpp PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(redoxi_common_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${required_includes})
target_link_libraries(redoxi_common_cpp PUBLIC ${required_dependencies})

ament_target_dependencies(redoxi_common_cpp PUBLIC rclcpp rcpputils)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(redoxi_common_cpp PRIVATE "REDOXI_COMMON_CPP_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS redoxi_common_cpp
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
  redoxi_common_cpp
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()

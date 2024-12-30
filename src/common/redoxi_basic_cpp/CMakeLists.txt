cmake_minimum_required(VERSION 3.8)
project(redoxi_basic_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rcutils REQUIRED)
find_package(rcpputils REQUIRED)
find_package(json_struct REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Boost REQUIRED)
find_package(fmt REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(redoxi_basic_cpp src/redoxi_basic_cpp.cpp src/logging/utils.cpp)
add_library(redoxi_basic_cpp::redoxi_basic_cpp ALIAS redoxi_basic_cpp)
target_compile_features(redoxi_basic_cpp PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(redoxi_basic_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(redoxi_basic_cpp PUBLIC
  Boost::boost
  json_struct::json_struct
  fmt::fmt
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(redoxi_basic_cpp PRIVATE "REDOXI_BASIC_CPP_BUILDING_LIBRARY")

ament_target_dependencies(redoxi_basic_cpp
  PUBLIC
  rclcpp
  rcpputils
  rcutils
  rclcpp_action
  rclcpp_lifecycle
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS redoxi_basic_cpp
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
  rclcpp
  rcpputils
  json_struct
  rcutils
  Boost
  rclcpp_action
  rclcpp_lifecycle
  fmt
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  redoxi_basic_cpp
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()

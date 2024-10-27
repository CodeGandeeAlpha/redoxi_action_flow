cmake_minimum_required(VERSION 3.8)
project(redoxi_extern_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(json_struct REQUIRED)
find_package(cv_bridge REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(redoxi_extern_cpp src/redoxi_extern_cpp.cpp)
add_library(redoxi_extern_cpp::redoxi_extern_cpp ALIAS redoxi_extern_cpp)
target_compile_features(redoxi_extern_cpp PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(redoxi_extern_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(redoxi_extern_cpp PUBLIC json_struct cv_bridge)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(redoxi_extern_cpp PRIVATE "REDOXI_EXTERN_CPP_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS redoxi_extern_cpp
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
  json_struct
  cv_bridge
)


ament_export_include_directories(
  include
)
ament_export_libraries(
  redoxi_extern_cpp
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()

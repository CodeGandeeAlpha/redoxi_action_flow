cmake_minimum_required(VERSION 3.8)
project(psg_person_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(psg_document_sink REQUIRED)
find_package(redoxi_common_nodes REQUIRED)
find_package(psg_common REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(srcfiles
  src/PSGPersonGenerator.cpp
  src/PSGPersonGeneratorTypes.cpp
)

add_library(psg_person_generator ${srcfiles})
add_library(psg_person_generator::psg_person_generator ALIAS psg_person_generator)
target_compile_features(psg_person_generator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(psg_person_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(psg_person_generator PRIVATE "PSG_PERSON_GENERATOR_BUILDING_LIBRARY")

ament_target_dependencies(
  psg_person_generator  PUBLIC
  psg_document_sink
  psg_common
  redoxi_common_nodes
  "rclcpp"
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS psg_person_generator
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
  psg_person_generator
)
ament_export_targets(
  export_${PROJECT_NAME}
)

# use these so that downstream packages can find the dependencies without explicitly listing them
ament_export_dependencies(
  rclcpp
  psg_document_sink
  psg_common
  redoxi_common_nodes
)

ament_package()

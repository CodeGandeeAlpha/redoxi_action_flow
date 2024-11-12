cmake_minimum_required(VERSION 3.8)
project(psg_master_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(redoxi_common_nodes REQUIRED)
find_package(psg_private_msgs REQUIRED)
find_package(redoxi_samples_lib REQUIRED)

set(srcfiles
  src/StampedDocumentPub.cpp
  src/MasterNodeTypes.cpp
  src/MasterNode.cpp
)

add_library(psg_master_node ${srcfiles})
add_library(psg_master_node::psg_master_node ALIAS psg_master_node)

target_include_directories(psg_master_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(psg_master_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  psg_master_node  PUBLIC
  redoxi_common_nodes
  redoxi_samples_lib
  psg_private_msgs
  "rclcpp"
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS psg_master_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(TARGETS psg_master_node
#   DESTINATION lib/${PROJECT_NAME})

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
  psg_master_node
)
ament_export_targets(
  export_${PROJECT_NAME}
)

# use these so that downstream packages can find the dependencies without explicitly listing them
ament_export_dependencies(
  rclcpp
  redoxi_common_nodes
  psg_private_msgs
  redoxi_samples_lib
  )

ament_package()

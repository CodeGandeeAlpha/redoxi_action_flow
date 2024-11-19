cmake_minimum_required(VERSION 3.8)
project(zz_psg_common)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(psg_private_msgs REQUIRED)
find_package(psg_public_msgs REQUIRED)

find_package(PassengerFlow REQUIRED)
find_package(vineyard REQUIRED)
find_package(OpenCV REQUIRED)

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

add_library(${PROJECT_NAME}
  src/psg_common.cpp
  src/msg_converter.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${VINEYARD_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  rclpy
  psg_private_msgs
  psg_public_msgs
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${VINEYARD_LIBRARIES} ${OpenCV_LIBS} PassengerFlow::PassengerFlow)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp vineyard)

ament_package()

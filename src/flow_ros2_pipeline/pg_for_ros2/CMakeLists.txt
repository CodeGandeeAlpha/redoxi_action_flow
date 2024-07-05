cmake_minimum_required(VERSION 3.8)
project(pg_for_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

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

# import if libraries
# include("/mnt/chengxiao/code/psf_ros2_ws/src/cmake/psgflow.cmake")

# import tracking library
set(PSGFLOW_ROOT_DIR /mnt/chengxiao/code/psf_ros2_ws/src/PassengerFlow)
set(PSGFLOW_BUILD_DIR /mnt/chengxiao/code/psf_ros2_ws/build-3rd)
set(REDOXI_TRACK_ROOT_DIR ${PSGFLOW_ROOT_DIR}/extern/RedoxiTracking)
set(REDOXI_TRACK_LIB ${PSGFLOW_BUILD_DIR}/extern/RedoxiTracking/src/libredoxi_track.so)
set(PSGFLOW_LIB ${PSGFLOW_BUILD_DIR}/src/libpassengerflow.so)
set(PSGFLOW_PIPELINE_UTILS_LIB ${PSGFLOW_BUILD_DIR}/apps/pipeline/libpsgflow_pipeline_utils.so)

# create tracking lib
add_library(redoxi_track SHARED IMPORTED)
set(REDOXI_TRACK_INCLUDE_DIRS
    ${REDOXI_TRACK_ROOT_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})
set_target_properties(redoxi_track PROPERTIES
    IMPORTED_LOCATION ${REDOXI_TRACK_LIB}
    INTERFACE_INCLUDE_DIRECTORIES "${REDOXI_TRACK_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES Eigen3::Eigen ${OpenCV_LIBS}
)

# create psgflow lib
add_library(psgflow SHARED IMPORTED)
set(PSGFLOW_INCLUDE_DIRS
    ${PSGFLOW_ROOT_DIR}/include
    ${PSGFLOW_ROOT_DIR}/extern/RedoxiTracking/include
    ${PSGFLOW_ROOT_DIR}/extern/cpplibs
    ${PSGFLOW_ROOT_DIR}/extern/
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})
set_target_properties(psgflow PROPERTIES
    IMPORTED_LOCATION ${PSGFLOW_LIB}
    INTERFACE_INCLUDE_DIRECTORIES "${PSGFLOW_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES Eigen3::Eigen ${OpenCV_LIBS}
)

# create psgflow pipeline utils lib
add_library(psgflow_pipeline_utils SHARED IMPORTED)
set(PSGFLOW_PIPELINE_UTILS_INCLUDE_DIRS
    ${PSGFLOW_ROOT_DIR}/include
    ${PSGFLOW_ROOT_DIR}/extern/RedoxiTracking/include
    ${PSGFLOW_ROOT_DIR}/extern/cpplibs
    ${PSGFLOW_ROOT_DIR}/extern/
    ${PSGFLOW_ROOT_DIR}/extern/dspatch/include
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})
set_target_properties(psgflow_pipeline_utils PROPERTIES
    IMPORTED_LOCATION ${PSGFLOW_PIPELINE_UTILS_LIB}
    INTERFACE_INCLUDE_DIRECTORIES "${PSGFLOW_PIPELINE_UTILS_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES Eigen3::Eigen ${OpenCV_LIBS}
)


# install(DIRECTORY include/
#   DESTINATION include)

# add_executable(${PROJECT_NAME}_node
#   src/main.cpp src/simple_lib.cpp
# )

# target_include_directories(${PROJECT_NAME}_node PUBLIC include ${OpenCV_INCLUDE_DIRS})
# # target_include_directories(${PROJECT_NAME}_node PUBLIC include)

# ament_target_dependencies(${PROJECT_NAME}_node PUBLIC
#   rclcpp
#   Eigen3
#   OpenCV
# )

# target_link_libraries(${PROJECT_NAME}_node PUBLIC redoxi_track)

# install(TARGETS ${PROJECT_NAME}_node
#   DESTINATION lib/${PROJECT_NAME})

# install(TARGETS redoxi_track
#   EXPORT redoxi_track_export
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

install(FILES ${REDOXI_TRACK_LIB}
  DESTINATION lib)
install(DIRECTORY ${REDOXI_TRACK_INCLUDE_DIRS}
  DESTINATION include)

install(FILES ${PSGFLOW_LIB}
  DESTINATION lib)
install(DIRECTORY ${PSGFLOW_INCLUDE_DIRS}
  DESTINATION include)

install(FILES ${PSGFLOW_PIPELINE_UTILS_LIB}
  DESTINATION lib)
install(DIRECTORY ${PSGFLOW_PIPELINE_UTILS_INCLUDE_DIRS}
  DESTINATION include)

# ament_export_targets(export_redoxi_track HAS_LIBRARY_TARGET)
ament_export_include_directories(${REDOXI_TRACK_INCLUDE_DIRS} ${PSGFLOW_INCLUDE_DIRS} ${PSGFLOW_PIPELINE_UTILS_INCLUDE_DIRS})
ament_export_libraries(redoxi_track psgflow psgflow_pipeline_utils)
ament_export_dependencies(Eigen3 OpenCV)

ament_package()


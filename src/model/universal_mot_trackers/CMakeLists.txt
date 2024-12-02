cmake_minimum_required(VERSION 3.8)
project(universal_mot_trackers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(RedoxiTrack REQUIRED)
find_package(redoxi_common_nodes REQUIRED)

# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)

add_library(universal_mot_trackers src/universal_mot_trackers.cpp)
add_library(universal_mot_trackers::universal_mot_trackers ALIAS
            universal_mot_trackers)
target_compile_features(universal_mot_trackers PUBLIC c_std_99 cxx_std_17
)# Require C99 and C++17
target_include_directories(
  universal_mot_trackers
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions(universal_mot_trackers
                           PRIVATE "UNIVERSAL_MOT_TRACKERS_BUILDING_LIBRARY")

target_link_libraries(universal_mot_trackers PUBLIC RedoxiTrack::RedoxiTrack)
ament_target_dependencies(universal_mot_trackers PUBLIC redoxi_common_nodes)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS universal_mot_trackers
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

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

ament_target_dependencies(universal_mot_trackers PUBLIC redoxi_common_nodes
                          RedoxiTrack)
ament_export_include_directories(include)
ament_export_libraries(universal_mot_trackers)
ament_export_targets(export_${PROJECT_NAME})

ament_package()

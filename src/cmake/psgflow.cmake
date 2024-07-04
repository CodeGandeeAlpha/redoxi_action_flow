find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# import tracking library
set(PSGFLOW_ROOT_DIR /mnt/chengxiao/code/psf_ros2_ws/src/PassengerFlow)
set(PSGFLOW_BUILD_DIR /mnt/chengxiao/code/psf_ros2_ws/build-3rd)
set(REDOXI_TRACK_ROOT_DIR ${PSGFLOW_ROOT_DIR}/extern/RedoxiTracking)
set(REDOXI_TRACK_LIB ${PSGFLOW_BUILD_DIR}/extern/RedoxiTracking/src/libredoxi_track.so)
set(PSGFLOW_LIB ${PSGFLOW_BUILD_DIR}/src/libpassengerflow.so)
set(PSGFLOW_PIPELINE_UTILS_LIB ${PSGFLOW_BUILD_DIR}/apps/pipeline/libpsgflow_pipeline_utils.so)

# create tracking lib
add_library(redoxi_track SHARED IMPORTED)
set(REDOXI_TRACK_INCLUDE_DIRS ${REDOXI_TRACK_ROOT_DIR}/include)
set_target_properties(redoxi_track PROPERTIES
    IMPORTED_LOCATION ${REDOXI_TRACK_LIB}
    INTERFACE_INCLUDE_DIRECTORIES "${REDOXI_TRACK_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES Eigen3::Eigen
)
# create psgflow lib
add_library(psgflow SHARED IMPORTED)
set(PSGFLOW_INCLUDE_DIRS
    ${PSGFLOW_ROOT_DIR}/include
    ${PSGFLOW_ROOT_DIR}/extern/RedoxiTracking/include
    ${PSGFLOW_ROOT_DIR}/extern/cpplibs
    ${PSGFLOW_ROOT_DIR}/extern/)
set_target_properties(psgflow PROPERTIES
    IMPORTED_LOCATION ${PSGFLOW_LIB}
    INTERFACE_INCLUDE_DIRECTORIES "${PSGFLOW_INCLUDE_DIRS}"
)

# create psgflow pipeline utils lib
add_library(psgflow_pipeline_utils SHARED IMPORTED)
set(PSGFLOW_PIPELINE_UTILS_INCLUDE_DIRS
    ${PSGFLOW_ROOT_DIR}/include
    ${PSGFLOW_ROOT_DIR}/extern/RedoxiTracking/include
    ${PSGFLOW_ROOT_DIR}/extern/cpplibs
    ${PSGFLOW_ROOT_DIR}/extern/
    ${PSGFLOW_ROOT_DIR}/extern/dspatch/include)
set_target_properties(psgflow_pipeline_utils PROPERTIES
    IMPORTED_LOCATION ${PSGFLOW_PIPELINE_UTILS_LIB}
    INTERFACE_INCLUDE_DIRECTORIES "${PSGFLOW_PIPELINE_UTILS_INCLUDE_DIRS}"
)
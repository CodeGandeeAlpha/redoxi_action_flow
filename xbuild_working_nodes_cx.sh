#!/bin/bash

# Function to display help message
display_help() {
    echo "Usage: $0 [OPTIONS]"
    echo "Build ROS2 packages with specified options."
    echo
    echo "Options:"
    echo "  --help     Display this help message and exit"
    echo "  --verbose  Enable verbose output"
    echo "  --debug    Build in Debug mode"
    echo "  --release  Build in Release mode (default)"
    echo
    echo "Example:"
    echo "  $0 --verbose --debug"
}

PackagesToBuild="redoxi_common_nodes \
                 redoxi_shared_memory \
                 redoxi_shm_v6d \
                 redoxi_inference \
                 test_cx \
                 rosboard \
                 psg_common \
                 psg_master_node \
                 psg_document_sink \
                 psg_detector \
                 psg_frame_det_source_sink \
                 psg_person_generator \
                 psg_pose_detector \
                 video_reader_orbbec"


# PackagesToBuild="
#                  redoxi_public_msgs \
#                  psg_private_msgs \
#                  "
# PackagesToBuild="redoxi_common_nodes \
#                  redoxi_shared_memory \
#                  redoxi_shm_v6d \
#                  test_package \
#                  rosboard \
#                  psg_detector
#                  "

# Check if --help flag is provided
if [[ "$*" == *"--help"* ]]; then
    display_help
    return
fi

# Check if --verbose flag is provided
VERBOSE=""
if [[ "$*" == *"--verbose"* ]]; then
    VERBOSE="--event-handlers console_direct+"
fi

# number of parallel jobs, equal to the number of cores-1
NUM_JOBS=$(($(nproc) - 1))

# Check if --debug or --release flag is provided, default to release
BUILD_TYPE="RelWithDebInfo"
if [[ "$*" == *"--debug"* ]]; then
    BUILD_TYPE="Debug"
elif [[ "$*" == *"--release"* ]]; then
    BUILD_TYPE="RelWithDebInfo"
fi

# PackagesToBuild="stream_worker psg_common psg_private_msgs psg_public_msgs video_reader"

echo "Building with BUILD_TYPE=$BUILD_TYPE"

# colcon build --packages-select $PackagesToBuild \
#     $VERBOSE \
#     --parallel-workers $NUM_JOBS \
#     --symlink-install \
#     --cmake-args \
#     -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
#     -DCMAKE_CXX_STANDARD=17 \
#     -DCMAKE_CXX_STANDARD_REQUIRED=ON \
#     -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

colcon build --packages-up-to $PackagesToBuild \
    $VERBOSE \
    --parallel-workers $NUM_JOBS \
    --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DCMAKE_CXX_STANDARD=20 \
    -DCMAKE_CXX_STANDARD_REQUIRED=ON \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DJSON_STRUCT_OPT_INSTALL=ON
source install/setup.bash

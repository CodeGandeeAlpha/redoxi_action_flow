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

# Get the absolute path of the directory containing this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# PackagesToBuild="redoxi_common_nodes \
#                  redoxi_inference \
#                  redoxi_inference_onnx \
#                  redoxi_dnn_models \
#                  redoxi_shm_v6d \
#                  yolo8_series \
#                  test_package \
#                  universal_mot_trackers \
#                  rosboard"

PackagesToBuild="redoxi_example_cpp rosboard"

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
NUM_JOBS=$(($(nproc) - 2))

# Add memory check before building
FREE_MEM_GB=$(free -g | awk '/^Mem:/{print $7}')
if [ $FREE_MEM_GB -lt 4 ]; then
    echo "Warning: Low memory available ($FREE_MEM_GB GB). Build might fail."
    NUM_JOBS=1
fi


# Check if --debug or --release flag is provided, default to release
BUILD_TYPE="RelWithDebInfo"
if [[ "$*" == *"--debug"* ]]; then
    BUILD_TYPE="Debug"
elif [[ "$*" == *"--release"* ]]; then
    BUILD_TYPE="Release"
fi

# PackagesToBuild="stream_worker psg_common psg_private_msgs psg_public_msgs video_reader"

echo "Building with BUILD_TYPE=$BUILD_TYPE"

# required by json_struct to use std data types
# note that json_struct does not name these flags consistently,
# using STD and STL in different places
json_struct_flags=" \
    -DJS_STD_OPTIONAL \
    -DJS_STD_UNORDERED_MAP \
    -DJS_STL_UNORDERED_SET \
    -DJS_STL_SET \
    -DJS_STL_MAP \
    -DJS_STL_ARRAY"

# c++ macros to control the log level
redoxi_works_flags=" \
    -DREDOXI_WORKS_LOG_IMPORTANCE_THRESHOLD_INFO=0 \
    -DREDOXI_WORKS_LOG_IMPORTANCE_THRESHOLD_DEBUG=0 \
    -DREDOXI_WORKS_LOG_IMPORTANCE_THRESHOLD_WARN=0"

colcon build --packages-up-to $PackagesToBuild \
    $VERBOSE \
    --parallel-workers $NUM_JOBS \
    --symlink-install \
    --cmake-args \
    -DCMAKE_CXX_FLAGS="$json_struct_flags $redoxi_works_flags" \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DCMAKE_CXX_STANDARD=20 \
    -DCMAKE_CXX_STANDARD_REQUIRED=ON \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DJSON_STRUCT_OPT_INSTALL=ON \
    -DJSON_STRUCT_OPT_BUILD_EXAMPLES=OFF \
    -DJSON_STRUCT_OPT_BUILD_TESTS=OFF \
    -DRKNPU_ROOT=$SCRIPT_DIR/tmp/rknpu-2.3 \
    -DCMAKE_C_COMPILER=clang \
    -DCMAKE_CXX_COMPILER=clang++

source install/setup.bash

# Set CUDA_VISIBLE_DEVICES in WSL2 environment
# windows docker has problem with multiple gpus
if grep -q "WSL2" /proc/version; then
    echo "Running in WSL2, setting CUDA_VISIBLE_DEVICES=0"
    export CUDA_VISIBLE_DEVICES=0
fi

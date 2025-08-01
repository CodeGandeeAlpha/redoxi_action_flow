#!/bin/bash

# Function to display help message
display_help() {
    echo "Usage: $0 [OPTIONS]"
    echo "Build ROS2 packages with specified options using pixi environment."
    echo
    echo "Options:"
    echo "  --help     Display this help message and exit"
    echo "  --verbose  Enable verbose output"
    echo "  --debug    Build in Debug mode"
    echo "  --release  Build in Release mode (default)"
    echo "  --packages Build only specific packages (comma-separated)"
    echo
    echo "Example:"
    echo "  $0 --verbose --debug"
    echo "  $0 --packages redoxi_example_cpp,redoxi_common_py"
}

# Check if pixi is available
if ! command -v pixi &> /dev/null; then
    echo "Error: pixi is not installed or not in PATH."
    echo "Please install pixi first: https://pixi.sh/"
    exit 1
fi

# Get the absolute path of the directory containing this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Default packages to build
PackagesToBuild="redoxi_example_cpp \
                 redoxi_common_py \
                 redoxi_inference_onnx \
                 yolo8_series \
                 redoxi_cpp_tests \
                 rosboard"

# Check if --packages flag is provided
if [[ "$*" == *"--packages"* ]]; then
    for arg in "$@"; do
        if [[ $arg == --packages=* ]]; then
            IFS=',' read -ra PACKAGES <<< "${arg#*=}"
            PackagesToBuild="${PACKAGES[*]}"
            break
        fi
    done
fi

# Check if --help flag is provided
if [[ "$*" == *"--help"* ]]; then
    display_help
    exit 0
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
echo "Packages: $PackagesToBuild"

# Check if we're already in a pixi environment
if [[ -z "$PIXI_ENVIRONMENT_NAME" ]]; then
    echo "Starting build in pixi environment..."
    # Run this script again inside pixi environment
    exec pixi run bash "$0" "$@"
fi

echo "Running in pixi environment: $PIXI_ENVIRONMENT_NAME"

# Setup Python environment variables for CMake (critical for pixi)
export PYTHON_EXECUTABLE=$(which python)
export Python_INCLUDE_DIR=$(python -c 'import sysconfig; print(sysconfig.get_path("include"))')
export Python_NumPy_INCLUDE_DIR=$(python -c 'import numpy; print(numpy.get_include())')

echo "Python executable: $PYTHON_EXECUTABLE"
echo "Python include dir: $Python_INCLUDE_DIR"
echo "NumPy include dir: $Python_NumPy_INCLUDE_DIR"

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
    -DCMAKE_CXX_COMPILER=clang++ \
    -DPython_EXECUTABLE:PATH=$PYTHON_EXECUTABLE \
    -DPython_INCLUDE_DIR:PATH=$Python_INCLUDE_DIR \
    -DPython_NumPy_INCLUDE_DIR=$Python_NumPy_INCLUDE_DIR \
    -DPython3_EXECUTABLE:PATH=$PYTHON_EXECUTABLE \
    -DPython3_INCLUDE_DIR:PATH=$Python_INCLUDE_DIR \
    -DPython3_NumPy_INCLUDE_DIR=$Python_NumPy_INCLUDE_DIR \
    -DPython_FIND_VIRTUALENV=ONLY \
    -DPython3_FIND_VIRTUALENV=ONLY

source install/setup.bash

# Set CUDA_VISIBLE_DEVICES in WSL2 environment
# windows docker has problem with multiple gpus
if grep -q "WSL2" /proc/version; then
    echo "Running in WSL2, setting CUDA_VISIBLE_DEVICES=0"
    export CUDA_VISIBLE_DEVICES=0
fi

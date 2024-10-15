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

PackagesToBuild="psg_private_msgs \
                 redoxi_public_msgs \
                 redoxi_common_cpp \
                 redoxi_video_reader_base \
                 test_package"

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

# Check if --debug or --release flag is provided, default to release
BUILD_TYPE="Release"
if [[ "$*" == *"--debug"* ]]; then
    BUILD_TYPE="Debug"
elif [[ "$*" == *"--release"* ]]; then
    BUILD_TYPE="Release"
fi

# PackagesToBuild="stream_worker psg_common psg_private_msgs psg_public_msgs video_reader"

echo "Building with BUILD_TYPE=$BUILD_TYPE"

colcon build --packages-select $PackagesToBuild \
    $VERBOSE \
    --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_CXX_STANDARD_REQUIRED=ON \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash

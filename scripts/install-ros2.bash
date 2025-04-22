#!/bin/bash

#! Usage: ./install-ros2.sh [--distro <ros_distro>] [--download-to <output_dir>]
#!
#! This script installs ROS2 with selected packages or downloads them to a specified directory.
#!
#! Options:
#!   --distro <ros_distro>      Specify the ROS2 distribution to install (default: based on Ubuntu version)
#!   --download-to <output_dir> Download packages to the specified directory without installing
#!
#! Examples:
#!   ./install-ros2.sh
#!   ./install-ros2.sh --distro humble
#!   ./install-ros2.sh --download-to /tmp/ros2-packages
#!
#! Note: This script must be run with root privileges.

# Function to print usage
print_usage() {
    echo "Usage: ./install-ros2.sh [--distro <ros_distro>] [--download-to <output_dir>]"
    echo "This script installs ROS2 with selected packages or downloads them to a specified directory."
    echo "Options:"
    echo "  --distro <ros_distro>      Specify the ROS2 distribution to install (default: based on Ubuntu version)"
    echo "  --download-to <output_dir> Download packages to the specified directory without installing"
    echo "Examples:"
    echo "  ./install-ros2.sh"
    echo "  ./install-ros2.sh --distro humble"
    echo "  ./install-ros2.sh --download-to /tmp/ros2-packages"
    echo "Note: This script must be run with root privileges."
}

# Get user preferred ROS2 distro from environment variable
ros2_prefer_distro=${ROS2_PREFER_DISTRO:-""}

# If ROS2_PREFER_DISTRO is not set, use a default value
if [ -z "$ros2_prefer_distro" ]; then
    # Check Ubuntu version
    ubuntu_version=$(lsb_release -rs)
    if [ "${ubuntu_version%.*}" -ge 24 ]; then
        ros2_prefer_distro="jazzy"  # Prefer 'jazzy' for Ubuntu 24.04 and newer
    else
        ros2_prefer_distro="iron"   # Default to 'iron' for older Ubuntu versions
    fi
fi

# Display help if --help is used
if [ "$1" == "--help" ]; then
    print_usage
    exit 0
fi

# require sudo permission
if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

# Parse command line arguments
ROS_DISTRO=$ros2_prefer_distro
DOWNLOAD_ONLY=false
DOWNLOAD_DIR=""

while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        --distro)
        ROS_DISTRO="$2"
        shift # past argument
        shift # past value
        ;;
        --download-to)
        DOWNLOAD_ONLY=true
        DOWNLOAD_DIR="$2"
        shift # past argument
        shift # past value
        ;;
        *)    # unknown option
        echo "Unknown option: $1"
        print_usage
        exit 1
        ;;
    esac
done

# Define packages to install/download
ROS_PACKAGES="ros-dev-tools \
ros-$ROS_DISTRO-ros-base \
ros-$ROS_DISTRO-rviz2 \
ros-$ROS_DISTRO-rqt \
ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
ros-$ROS_DISTRO-cyclonedds \
ros-$ROS_DISTRO-nav2-lifecycle-manager \
ros-$ROS_DISTRO-nav2-util"

export DEBIAN_FRONTEND=noninteractive
export TZ=Asia/Shanghai

if [ "$DOWNLOAD_ONLY" = true ]; then
    # Convert DOWNLOAD_DIR to absolute path
    DOWNLOAD_DIR=$(realpath "$DOWNLOAD_DIR")
    echo "Downloading ROS2 $ROS_DISTRO packages to $DOWNLOAD_DIR"
    mkdir -p "$DOWNLOAD_DIR"
    apt update
    apt-get --download-only -o Dir::Cache="$DOWNLOAD_DIR" \
        -o Dir::Cache::archives="$DOWNLOAD_DIR" install -y --no-install-recommends $ROS_PACKAGES
    echo "Packages downloaded to $DOWNLOAD_DIR"
else
    echo "Installing ROS2 $ROS_DISTRO with selected packages"
    apt update
    apt install -y --no-install-recommends $ROS_PACKAGES
fi

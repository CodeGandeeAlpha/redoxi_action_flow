#!/bin/bash

# require sudo
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root"
  exit 1
fi

# install tbb
apt-get install -y libtbb-dev xtensor-dev

# install nuget for onnx
# apt-get install -y nuget tensorrt-libs

# required by the ros2_web_bridge
PYTHON_VERSION=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
PIP_EXTRA_ARGS=""

PYTHON_MAJOR=$(echo "$PYTHON_VERSION" | cut -d. -f1)
PYTHON_MINOR=$(echo "$PYTHON_VERSION" | cut -d. -f2)

if [ "$PYTHON_MAJOR" -gt 3 ] || { [ "$PYTHON_MAJOR" -eq 3 ] && [ "$PYTHON_MINOR" -ge 11 ]; }; then
  PIP_EXTRA_ARGS="--break-system-packages"
fi

if [ ! -z "$http_proxy" ]; then
  sudo pip3 install simplejpeg tornado --proxy="$http_proxy" $PIP_EXTRA_ARGS --break-system-packages
else
  sudo pip3 install simplejpeg $PIP_EXTRA_ARGS tornado --break-system-packages
fi

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
if [ ! -z "$http_proxy" ]; then
    sudo pip3 install simplejpeg tornado --proxy="$http_proxy" --break-system-packages
else
    sudo pip3 install simplejpeg tornado --break-system-packages
fi

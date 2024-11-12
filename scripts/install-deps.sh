#!/bin/bash

# require sudo
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root"
  exit 1
fi

# install tbb
apt-get install -y libtbb-dev

# install nuget for onnx
apt-get install -y nuget tensorrt-libs

# required by the ros2_web_bridge
sudo pip3 install simplejpeg
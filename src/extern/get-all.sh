#!/bin/bash

# Clone json_struct
echo "Cloning json_struct..."
git clone --depth=1 https://github.com/jorgen/json_struct.git

# Clone vision_opencv
echo "Cloning vision_opencv..."
git clone --depth=1 https://github.com/ros-perception/vision_opencv.git -b rolling

# Clone rosboard
echo "Cloning rosboard..."
git clone --depth=1 https://github.com/dheera/rosboard.git

# redoxi tracking library
echo "Cloning redoxi tracking library..."
git clone https://github.com/CodeGandee/RedoxiTrack.git


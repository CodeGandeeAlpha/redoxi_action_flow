#!/bin/bash

colcon build --packages-select stream_worker \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_DEBUG_POSTFIX=-debug
source install/setup.bash
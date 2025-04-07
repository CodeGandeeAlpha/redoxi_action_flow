#!/bin/bash

# source /opt/ros/humble/setup.bash
# source /hard/volume/workspace/code/psf_ros2_ws/install/setup.bash

# ros2 run redoxi_samples_nodes FrameRelayNode --ros-args --params-file /hard/volume/workspace/code/psf_ros2_ws/src/example/test_package/config/lch_v2_relay.yaml

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
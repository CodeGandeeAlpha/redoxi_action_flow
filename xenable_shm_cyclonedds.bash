#!/bin/bash

# get directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

export CYCLONEDDS_URI=file://$SCRIPT_DIR/scripts/cyclonedds-shm-profile.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

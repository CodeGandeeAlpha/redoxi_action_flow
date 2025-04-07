#!/bin/bash

# get directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# enable shared memory transport in fastDDS
export FASTRTPS_DEFAULT_PROFILES_FILE=$SCRIPT_DIR/scripts/fastdds-shm-profile.xml RMW_IMPLEMENTATION=rmw_fastrtps_cpp

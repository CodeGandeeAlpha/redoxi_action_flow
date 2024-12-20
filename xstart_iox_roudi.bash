#! /bin/bash

# get directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Kill existing iox-roudi if running
pkill iox-roudi

# Start iox-roudi with config file
iox-roudi -c $SCRIPT_DIR/scripts/cyclonedds-shm-roudi-config.toml
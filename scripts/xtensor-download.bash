#!/bin/bash

# get dir of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

download_dir="${SCRIPT_DIR}/../tmp/xtensor"

# download xtensor from github
git clone --depth 1 https://github.com/xtensor-stack/xtensor.git ${download_dir}

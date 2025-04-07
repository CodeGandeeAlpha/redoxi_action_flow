#!/bin/bash

# get dir of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source_dir="${SCRIPT_DIR}/../tmp/xtensor"
build_dir="${SCRIPT_DIR}/../tmp/xtensor/build"

# check if source dir exists
if [ ! -d "${source_dir}" ]; then
    echo "Source directory ${source_dir} does not exist"
    exit 1
fi

# create build directory if it doesn't exist
if [ ! -d "${build_dir}" ]; then
    mkdir -p "${build_dir}"
fi

# go into build directory
cd "${build_dir}"

# run cmake
cmake "${source_dir}" \
    -DCMAKE_INSTALL_PREFIX="${build_dir}/install" \
    -DCMAKE_BUILD_TYPE=Release

# build and install
cmake --build . --target install


#!/bin/bash

# get dir of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
download_dir="${SCRIPT_DIR}/../tmp/miniforge"
install_dir="/soft/app/miniforge"

echo "Installing miniforge..."

# install miniforge
url_x64="https://mirrors.tuna.tsinghua.edu.cn/github-release/conda-forge/miniforge/LatestRelease/Miniforge3-Linux-x86_64.sh"
url_aarch64="https://mirrors.tuna.tsinghua.edu.cn/github-release/conda-forge/miniforge/LatestRelease/Miniforge3-Linux-aarch64.sh"

# download miniforge based on architecture
if [ "$(uname -m)" == "aarch64" ]; then
    url="${url_aarch64}"
    echo "Detected aarch64 architecture"
else
    url="${url_x64}"
    echo "Detected x86_64 architecture"
fi

echo "Downloading miniforge from ${url}..."

# create download directory if it doesn't exist
mkdir -p "${download_dir}"

# get filename from url
download_filename=$(basename "${url}")

# download miniforge installer if it doesn't exist
if [ ! -f "${download_dir}/${download_filename}" ]; then
    echo "Downloading ${download_filename}..."
    wget --progress=bar:force -O "${download_dir}/${download_filename}" "${url}"
else
    echo "Using existing ${download_filename}"
fi

echo "Installing miniforge to ${install_dir}..."
bash "${download_dir}/${download_filename}" -b -p "${install_dir}"

#!/bin/bash

# download v6d console

# Function to display usage
usage() {
    echo "Usage: $0 [--install-dir=xxx] [--help]"
    echo "  --install-dir=xxx   Specify the directory to download and install the vineyardctl."
    echo "  --help              Display this help message."
    exit 1
}

# Get the directory of the current script
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Default directory
default_install_dir="${script_dir}/../tmp"

# Parse arguments
for arg in "$@"; do
    case $arg in
        --install-dir=*)
        dst_dir="${arg#*=}"
        shift
        ;;
        --help)
        usage
        ;;
        *)
        echo "Unknown option: $arg"
        usage
        ;;
    esac
done

# Set default value if not provided
dst_dir="${dst_dir:-$default_install_dir}"

# download url
DownloadUrl="https://github.com/v6d-io/v6d/releases/download/v0.24.2/vineyardctl-v0.24.2-linux-amd64"

# Create the destination directory if it doesn't exist
mkdir -p "${dst_dir}"

# Define the destination file path
dst_file="${dst_dir}/vineyardctl"

# Download the file and rename it
echo "Downloading vineyardctl from ${DownloadUrl}"
wget --progress=bar:force "${DownloadUrl}" -O "${dst_file}"

echo "Download complete. File saved as ${dst_file}"

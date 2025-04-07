#!/bin/bash

# Function to display usage
usage() {
    echo "Usage: $0 [--install-dir=xxx] [--help]"
    echo "  --install-dir=xxx   Specify the directory to download and install the onnxruntime package."
    echo "  --help              Display this help message."
    exit 1
}

# Default directory
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
default_install_dir="${script_dir}/../tmp/onnxruntime"

OnnxVersion="1.20.1"

# onnx url
OnnxPackageName_arm64="onnxruntime-linux-aarch64-${OnnxVersion}"
OnnxPackageName_x64="onnxruntime-win-x64-gpu-${OnnxVersion}"

# Detect platform
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    if [[ $(uname -m) == "aarch64" ]]; then
        OnnxPackageName="${OnnxPackageName_arm64}"
    else
        OnnxPackageName="onnxruntime-linux-x64-gpu-${OnnxVersion}"
    fi
elif [[ "$OSTYPE" == "msys"* ]] || [[ "$OSTYPE" == "cygwin"* ]]; then
    OnnxPackageName="${OnnxPackageName_x64}"
else
    echo "Unsupported platform: $OSTYPE"
    exit 1
fi

OnnxUrl="https://github.com/microsoft/onnxruntime/releases/download/v${OnnxVersion}/${OnnxPackageName}.tgz"

OnnxTargetUrl="${OnnxUrl}"
OnnxTargetPackageName="${OnnxPackageName}"

# Parse arguments
for arg in "$@"; do
    case $arg in
        --install-dir=*)
        dst_install_dir="${arg#*=}"
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
dst_install_dir="${dst_install_dir:-$default_install_dir}"

# download and extract onnxruntime
echo "Creating directory: ${dst_install_dir}"
mkdir -p ${dst_install_dir}

# Check if onnxruntime.tgz already exists
OnnxTargetFilename="${OnnxTargetPackageName}.tgz"
if [ ! -f "${dst_install_dir}/${OnnxTargetFilename}" ]; then
    echo "Downloading onnxruntime from ${OnnxTargetUrl}"
    wget --progress=bar:force ${OnnxTargetUrl} -O ${dst_install_dir}/${OnnxTargetFilename}
else
    echo "${OnnxTargetFilename} already exists, skipping download."
fi

echo "Extracting onnxruntime to ${dst_install_dir}"
tar --checkpoint=.1000 --checkpoint-action=dot -xzf ${dst_install_dir}/${OnnxTargetFilename} -C ${dst_install_dir}
echo "Extraction complete."

# echo "Fixing onnxruntime directory structure problems"
# echo "See github issues: https://github.com/microsoft/onnxruntime/issues/22267"

OnnxRuntimeDir="${dst_install_dir}/${OnnxPackageName}"
echo "OnnxRuntimeDir: ${OnnxRuntimeDir}"

echo "Renaming ${OnnxRuntimeDir}/lib to ${OnnxRuntimeDir}/lib64"
mv "${OnnxRuntimeDir}/lib" "${OnnxRuntimeDir}/lib64"

echo "Moving ${OnnxRuntimeDir}/include to ${OnnxRuntimeDir}/include/onnxruntime"
mv "${OnnxRuntimeDir}/include" "${OnnxRuntimeDir}/onnxruntime"
mkdir -p "${OnnxRuntimeDir}/include"
mv "${OnnxRuntimeDir}/onnxruntime" "${OnnxRuntimeDir}/include/onnxruntime"


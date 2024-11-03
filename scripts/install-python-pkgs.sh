#!/bin/bash

# required root permission
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root"
    exit 1
fi

required_packages=(
    "ipykernel"
    "scipy"
    "numpy"
    "opencv-contrib-python"
    "networkx"
    "matplotlib"
    "pyyaml"
    "attrs"
    "cattrs"
    "omegaconf"
    "rich"
    "click"
)

echo "The following packages will be installed:"
for pkg in "${required_packages[@]}"; do
    echo "- $pkg"
done


for pkg in "${required_packages[@]}"; do
    pip3 install $pkg
done

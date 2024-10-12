#!/bin/bash

# Install v6d
echo "Downloading v6d..."
mkdir -p ./tmp
git clone https://github.com/v6d-io/v6d ./tmp/v6d
cd ./tmp/v6d
git checkout tags/v0.24.2  # solid version, main branch will make error
git submodule update --init
# Go back to the original directory
cd ../..

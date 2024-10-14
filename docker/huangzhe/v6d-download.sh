#!/bin/bash

# get DIR of this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Install v6d
echo "Downloading v6d..."
mkdir -p ./tmp
git clone https://github.com/v6d-io/v6d ./tmp/v6d
# Save the current directory
pushd $(pwd)

cd ./tmp/v6d
git checkout tags/v0.24.2  # solid version, main branch will make error
git submodule update --init
# Go back to the original directory
popd

# Copy v6d-build.sh to ./tmp/v6d
echo "Copying v6d-build.sh to ./tmp/v6d..."
cp "$DIR/v6d-build.sh" ./tmp/v6d/

# Make the script executable
chmod +x ./tmp/v6d/v6d-build.sh


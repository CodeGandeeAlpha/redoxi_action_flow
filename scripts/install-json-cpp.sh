#!/bin/bash

# download and install nlohmann/json latest version
# download to ../tmp/json and then cmake and install

# create a temporary directory for json
echo "Creating a temporary directory for json..."
mkdir -p ../tmp/json

# navigate to the temporary directory
echo "Navigating to the temporary directory..."
cd ../tmp/json

# clone the latest version of nlohmann/json from GitHub
echo "Cloning the latest version of nlohmann/json from GitHub..."
git clone --depth 1 https://github.com/nlohmann/json.git .

# create a build directory
echo "Creating a build directory..."
mkdir build
cd build

# run cmake to configure the project
echo "Running cmake to configure the project..."
cmake ..

# build and install the project
echo "Building and installing the project..."
make
sudo make install

# navigate back to the original directory
echo "Navigating back to the original directory..."
cd ../../..

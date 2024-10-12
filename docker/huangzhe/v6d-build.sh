#!/bin/bash

# assuming we are in v6d root directory

# create a build directory for cmake, and build the release version
cmake -E make_directory build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

# Install v6d
sudo cmake --install ./build

# Install v6d python binding and etcd
cd ..
sudo python3 setup.py bdist_wheel
sudo pip install dist/vineyard-*.whl
#!/bin/bash

# assuming we are in v6d root directory

# create a build directory for cmake, and build the release version
cmake -E make_directory build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_VINEYARD_GRAPH=OFF \
    -DBUILD_VINEYARD_GRAPH_WITH_GAR=OFF \
    -DUSE_CUDA=OFF \
    -DBUILD_VINEYARD_TESTS=OFF
cmake --build build -j$(nproc)

# Install v6d
sudo cmake --install ./build

# Install v6d python binding and etcd
sudo python3 setup.py bdist_wheel
sudo pip install dist/vineyard-*.whl
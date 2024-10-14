#!/bin/bash

# assuming we are in v6d root directory

# create a build directory for cmake, and build the release version
# we use c++17, and include cstdint to fix the error in etcd source code
# Define shared CMake options
CMAKE_COMMON_OPTIONS="-DCMAKE_BUILD_TYPE=Release \
    -DBUILD_VINEYARD_GRAPH=OFF \
    -DBUILD_VINEYARD_GRAPH_WITH_GAR=OFF \
    -DUSE_CUDA=OFF \
    -DBUILD_VINEYARD_TESTS=OFF \
    -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_CXX_STANDARD_REQUIRED=ON"

cmake -E make_directory build
if [ "$(uname -m)" = "aarch64" ]; then
    # use clang because gcc may crash compiling etcd
    
    cmake -S . -B build $CMAKE_COMMON_OPTIONS \
        -DCMAKE_C_COMPILER=clang \
        -DCMAKE_CXX_COMPILER=clang++ \
        -DCMAKE_CXX_FLAGS="-include cstdint"
else
    cmake -S . -B build $CMAKE_COMMON_OPTIONS -DCMAKE_CXX_FLAGS="-include cstdint"
fi

if cmake --build build -j$(nproc); then
    # Install v6d only if the build is successful
    sudo cmake --install ./build

    # Install v6d python binding and etcd
    sudo python3 setup.py bdist_wheel
    sudo pip3 install dist/vineyard-*.whl
else
    echo "Build failed. Skipping installation."
    exit 1
fi

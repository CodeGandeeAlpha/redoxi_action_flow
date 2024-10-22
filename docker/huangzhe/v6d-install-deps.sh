#!/bin/bash

# install v6d dependencies
echo "Installing dependencies..."
sudo apt-get install -y doxygen \
                   libboost-all-dev \
                   libcurl4-openssl-dev \
                   libgflags-dev \
                   libgoogle-glog-dev \
                   libgrpc-dev \
                   libgrpc++-dev \
                   libmpich-dev \
                   libprotobuf-dev \
                   libssl-dev \
                   libunwind-dev \
                   libz-dev \
                   protobuf-compiler-grpc \
                   wget

# allow pip3 to break the system package
sudo pip3 config --global set global.break-system-packages true
sudo pip3 install libclang

# Install libclang-dev package
sudo apt-get update
sudo apt-get -y install libclang-dev

echo "Installing apache arrow..."
wget https://apache.jfrog.io/artifactory/arrow/$(lsb_release --id --short | tr 'A-Z' 'a-z')/apache-arrow-apt-source-latest-$(lsb_release --codename --short).deb \
    -O /tmp/apache-arrow-apt-source-latest-$(lsb_release --codename --short).deb
sudo apt install -y -V /tmp/apache-arrow-apt-source-latest-$(lsb_release --codename --short).deb
sudo apt update -y
sudo apt install -y libarrow-dev

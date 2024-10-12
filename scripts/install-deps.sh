#!/bin/bash

# require sudo
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root"
  exit 1
fi

# install tbb
apt-get install -y libtbb-dev
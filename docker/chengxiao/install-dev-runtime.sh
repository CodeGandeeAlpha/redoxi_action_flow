#!/bin/bash

# get the path of this file
SCRIPTFILE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
echo "Executing $SCRIPTFILE"

# Install dev tools
export DEBIAN_FRONTEND=noninteractive

# get dir of current script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

sudo bash $DIR/../../stage-1/contrib/igamenovoer/install-cpp-libs.sh

# cp ssh
cp $DIR/../../stage-1/tmp/.ssh /home/chengxiao/.ssh -r

# Install etcd
sudo apt-get update
sudo apt-get install etcd -y

# Install pip3
sudo apt-get install python3-pip -y

# set pip mirror
echo "Configuring pip to use Tsinghua mirror"
sudo pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
sudo pip config set global.trusted-host pypi.tuna.tsinghua.edu.cn

# echo "Configuring pip to use Aliyun mirror"
# pip config set global.index-url https://mirrors.aliyun.com/pypi/simple
# pip config set global.trusted-host mirrors.aliyun.com

# Install torch
# pip install torch==2.1.2 torchvision==0.16.2 torchaudio==2.1.2 --index-url https://mirrors.aliyun.com/pytorch-wheels/cu121
sudo pip install torch==2.1.2 torchvision==0.16.2 torchaudio==2.1.2 --index-url https://download.pytorch.org/whl/cu121 --proxy http://host.docker.internal:7890

# Install onnxruntime-gpu
# sudo pip install onnxruntime-gpu==1.18.0
sudo pip install onnxruntime-gpu==1.18.0 --index-url https://aiinfra.pkgs.visualstudio.com/PublicPackages/_packaging/onnxruntime-cuda-12/pypi/simple/ --proxy http://host.docker.internal:7890 # cuda12 cudnn8

# Install mmdetection
sudo pip install -U openmim
sudo mim install mmengine
sudo mim install "mmcv>=2.0.0"
sudo mim install mmdet

# Install nvitop
sudo pip install nvitop

# Install clangd and clang
sudo apt-get install -y clang clangd

# Install boost
sudo apt-get install -y libboost-all-dev

# Install libomp
sudo apt-get install -y libomp-dev

# Install rtmlib
sudo pip install rtmlib -i https://pypi.org/simple --proxy http://host.docker.internal:7890

# Install numpy < 2.0
sudo pip install "numpy>=1.21.6,<1.25.0"
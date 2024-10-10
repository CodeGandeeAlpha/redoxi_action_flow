# Install v6d
git clone https://github.com/v6d-io/v6d /tmp/v6d
cd /tmp/v6d
git checkout tags/v0.24.2  # solid version, main branch will make error
git submodule update --init

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

sudo pip install libclang

wget https://apache.jfrog.io/artifactory/arrow/$(lsb_release --id --short | tr 'A-Z' 'a-z')/apache-arrow-apt-source-latest-$(lsb_release --codename --short).deb \
    -O /tmp/apache-arrow-apt-source-latest-$(lsb_release --codename --short).deb
sudo apt install -y -V /tmp/apache-arrow-apt-source-latest-$(lsb_release --codename --short).deb
sudo apt update -y
sudo apt install -y libarrow-dev

mkdir build
cd build
cmake ..
make -j 64
sudo make install

# Install v6d python binding and etcd
export MAKEFLAGS="-j64"
cd ..
sudo python3 setup.py bdist_wheel
sudo pip install dist/vineyard-0.24.2-cp310-cp310-linux_x86_64.whl

# add v6d library path to LD_LIBRARY_PATH
echo 'export LD_LIBRARY_PATH=/usr/local/lib:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH' >> /home/chengxiao/.bashrc
sudo echo 'export LD_LIBRARY_PATH=/usr/local/lib:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH' >> /root/.bashrc

# install ultralytics
sudo pip install ultralytics
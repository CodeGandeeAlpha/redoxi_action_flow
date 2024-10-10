# Install PassengerFlow
git clone git@codeup.aliyun.com:61adcb80e05da4a409ab67b8/intellif/PassengerFlow.git /tmp/PassengerFlow
cd /tmp/PassengerFlow
git checkout -b ig-refactor origin/ig-refactor
mkdir build
cd build
cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON
make -j 64
sudo make install
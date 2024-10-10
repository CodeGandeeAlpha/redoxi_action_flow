# get dir of current script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# cp ssh
cp $DIR/../../stage-1/tmp/ssh/* /home/chengxiao/.ssh/
sudo cp $DIR/../../stage-1/tmp/ssh/* /root/.ssh/
ssh-keyscan codeup.aliyun.com >> /home/chengxiao/.ssh/known_hosts
ssh-keyscan codeup.aliyun.com >> /root/.ssh/known_hosts

echo "Installing RedoxiTrack"

# Install RedoxiTrack
git clone git@codeup.aliyun.com:61adcb80e05da4a409ab67b8/intellif/RedoxiTracking.git /tmp/RedoxiTracking
cd /tmp/RedoxiTracking
git checkout -b ig-refactor origin/ig-refactor
mkdir build
cd build
cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON
make -j 64
sudo make install
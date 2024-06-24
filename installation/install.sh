#!/bin/bash

# Install Livox SDK2
echo "################################################"
SDK_VERSION="1.2.5"
echo "[Begin] Installing Livox SDK2 [version: ${SDK_VERSION}]"
cd /tmp && wget https://github.com/Livox-SDK/Livox-SDK2/archive/refs/tags/v${SDK_VERSION}.tar.gz
tar -xvf v${SDK_VERSION}.tar.gz
mv Livox-SDK2-${SDK_VERSION} ~/git/Livox-SDK2
cd ~/git/Livox-SDK2 && mkdir build && cd build
cmake .. && make -j
sudo make install

# How to remove it:
# sudo rm -rf /usr/local/lib/liblivox_lidar_sdk_*
# sudo rm -rf /usr/local/include/livox_lidar_*

echo "[End] Installing Livox SDK2"
echo "################################################"

# Install Livox ROS Driver 2
echo "################################################"
echo "[Begin] Installing Livox ROS Driver 2"

readonly VERSION_ROS1="ROS1"
readonly VERSION_ROS2="ROS2"
readonly VERSION_HUMBLE="humble"

ROS_VERSION=""
ROS_HUMBLE=""

# Set working ROS version
if [ "$1" = "ROS2" ]; then
    ROS_VERSION=${VERSION_ROS2}
elif [ "$1" = "humble" ]; then
    ROS_VERSION=${VERSION_ROS2}
    ROS_HUMBLE=${VERSION_HUMBLE}
elif [ "$1" = "ROS1" ]; then
    ROS_VERSION=${VERSION_ROS1}
else
    echo "Invalid Argument"
    exit
fi
echo "ROS version is: "$ROS_VERSION

# substitute the files/folders: CMakeList.txt, package.xml(s)
if [ -f package.xml ]; then
    rm package.xml
fi
if [ -f CMakeLists.txt ]; then
    rm CMakeLists.txt
fi
cd ..
if [ ${ROS_VERSION} = ${VERSION_ROS1} ]; then
    ln -sf package_ROS1.xml package.xml
    ln -sf CMakeLists_ROS1.txt CMakeLists.txt
    ln -sf launch_ROS1 launch
elif [ ${ROS_VERSION} = ${VERSION_ROS2} ]; then
    ln -sf package_ROS2.xml package.xml
    ln -sf CMakeLists_ROS2.txt CMakeLists.txt
    ln -sf launch_ROS2 launch
fi
echo "[End] Installing Livox ROS Driver 2"
echo "################################################"

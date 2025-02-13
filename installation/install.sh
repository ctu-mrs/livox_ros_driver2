#!/bin/bash

SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"

ROS_DISTRO="ROS1"
if [[ "$#" -eq 1 ]]; then
  ROS_DISTRO="$1"
elif [[ "$#" -gt 1 ]]; then
  echo "Usage: ./install.sh [ ROS1 (default) | ROS2 ]"
  exit -1
fi

if [[ "$ROS_DISTRO" != "ROS1" && "$ROS_DISTRO" != "ROS2" ]]; then
  echo "Usage: ./install.sh [ ROS1 (default) | ROS2 ]"
  exit -1
fi

echo "Installing for ROS version: $ROS_DISTRO"

# Install Livox SDK2
echo "################################################"
SDK_VERSION="1.2.5"
echo "[Begin] Installing Livox SDK2"
if [[ -d "$SCRIPT_PATH/../3rdparty/Livox-SDK2" ]]; then
  echo "$SCRIPT_PATH/../3rdparty/Livox-SDK2 found -> using this version"
else
  echo "$SCRIPT_PATH/../3rdparty/Livox-SDK2 NOT found -> downloading SDK (version: ${SDK_VERSION})"
  cd /tmp && wget https://github.com/Livox-SDK/Livox-SDK2/archive/refs/tags/v${SDK_VERSION}.tar.gz
  tar -xf v${SDK_VERSION}.tar.gz
  mv Livox-SDK2-${SDK_VERSION} $SCRIPT_PATH/../3rdparty/Livox-SDK2
fi

cd $SCRIPT_PATH/../3rdparty/Livox-SDK2 && mkdir -p build && cd build
cmake .. && make -j

sudo make install 2>/dev/null
if [ $? -ne 0 ]; then
  echo -e "\e[1;31mFailed make. Livox driver will not be available\e[0m"
  exit 1
fi


# How to remove it:
# sudo rm -rf /usr/local/lib/liblivox_lidar_sdk_*
# sudo rm -rf /usr/local/include/livox_lidar_*

echo "[End] Installing Livox SDK2"
echo "################################################"

# Install Livox ROS Driver 2
echo "################################################"
echo "[Begin] Installing Livox ROS Driver 2"

# substitute the files/folders: CMakeList.txt, package.xml(s)
cd ${SCRIPT_PATH}/..
if [[ ${ROS_DISTRO} == "ROS1" ]]; then
    ln -sf package_ROS1.xml package.xml
    ln -sf CMakeLists_ROS1.txt CMakeLists.txt
    # ln -sf launch_ROS1 launch
elif [[ ${ROS_DISTRO} == "ROS2" ]]; then
    ln -sf package_ROS2.xml package.xml
    ln -sf CMakeLists_ROS2.txt CMakeLists.txt
    # ln -sf launch_ROS2 launch
fi
echo "[End] Installing Livox ROS Driver 2"
echo "################################################"

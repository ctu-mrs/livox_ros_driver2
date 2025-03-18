#!/bin/bash

SCRIPT_PATH="$( cd "$(dirname "$0")" || exit ; pwd -P )"

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
echo "Installing Livox SDK2"
echo "################################################"

cd $SCRIPT_PATH/../3rdparty/Livox-SDK2-v1.2.5 && rm -r build && mkdir -p build && cd build
cmake \
  -DCMAKE_C_COMPILER=/usr/bin/gcc-9 \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++-9 \
  -DCMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS} -fPIC" \
  -DCMAKE_C_FLAGS="${CMAKE_C_FLAGS} -fPIC" \
  -DCMAKE_INSTALL_PREFIX="$SCRIPT_PATH/../3rdparty/install" \
  -DBUILD_SHARED_LIBS=ON .. \
  && make -j clean install

echo "################################################"

echo "Linking the correct CMakeLists.txt and package.xml for $ROS_DISTRO"

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
echo "################################################"

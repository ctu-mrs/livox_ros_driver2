#!/bin/bash
SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"

sudo rm -rf /usr/local/lib/liblivox_lidar_sdk_*
sudo rm -rf /usr/local/include/livox_lidar_*

rm package.xml
rm CMakeLists.xml

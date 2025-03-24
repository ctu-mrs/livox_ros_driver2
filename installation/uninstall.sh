#!/bin/bash

SCRIPT_PATH="$( cd "$(dirname "$0")" || exit ; pwd -P )"

echo "Removing links to CMakeLists.txt and package.xml"
rm ${SCRIPT_PATH}/../package.xml
rm ${SCRIPT_PATH}/../CMakeLists.txt

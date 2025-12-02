cmake_minimum_required(VERSION 3.14)
project(livox_ros_driver2)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake/modules")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# Printf version info
include(cmake/version.cmake)
project(${PROJECT_NAME} VERSION ${LIVOX_ROS_DRIVER2_VERSION} LANGUAGES CXX)
message(STATUS "${PROJECT_NAME} version: ${LIVOX_ROS_DRIVER2_VERSION}")

add_definitions(-DBUILDING_ROS2)

SET(DEPENDENCIES
  rclcpp
  rclcpp_components
  PCL
  pcl_ros
  std_msgs
  builtin_interfaces
  rosidl_default_generators
  mrs_lib
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

# check apr
find_package(PkgConfig)
pkg_check_modules(APR apr-1)
if (APR_FOUND)
  message(${APR_INCLUDE_DIRS})
  message(${APR_LIBRARIES})
endif (APR_FOUND)

find_package(PCL 1.8 REQUIRED)

# generate custom msg headers
set(LIVOX_INTERFACES livox_interfaces2)
rosidl_generate_interfaces(${LIVOX_INTERFACES}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES builtin_interfaces std_msgs
  LIBRARY_NAME ${PROJECT_NAME}
)

# Livox SDK
find_library(LIVOX_LIDAR_SDK_LIBRARY liblivox_lidar_sdk_shared.so /usr/local/lib REQUIRED)

find_path(LIVOX_LIDAR_SDK_INCLUDE_DIR NAMES "livox_lidar_api.h" "livox_lidar_def.h" REQUIRED)

set(LIBRARIES
  LivoxRosDriver2_Driver
)

## --------------------------------------------------------------
## |                           compile                          |
## --------------------------------------------------------------

add_library(LivoxRosDriver2_Driver SHARED
  src/livox_ros_driver2.cpp
  src/lddc.cpp
  src/lds.cpp
  src/lds_lidar.cpp

  src/comm/comm.cpp
  src/comm/ldq.cpp
  src/comm/semaphore.cpp
  src/comm/lidar_imu_data_queue.cpp
  src/comm/cache_index.cpp
  src/comm/pub_handler.cpp

  src/parse_cfg_file/parse_cfg_file.cpp
  src/parse_cfg_file/parse_livox_lidar_cfg.cpp

  src/call_back/lidar_common_callback.cpp
  src/call_back/livox_lidar_callback.cpp
)

ament_target_dependencies(LivoxRosDriver2_Driver
  ${DEPENDENCIES}
)

target_include_directories(LivoxRosDriver2_Driver
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(LivoxRosDriver2_Driver
  PRIVATE
    include
    ${PCL_INCLUDE_DIRS}
    ${APR_INCLUDE_DIRS}
    ${LIVOX_LIDAR_SDK_INCLUDE_DIR}
    ${mrs_lib_INCLUDE_DIRS}
    # ${LIVOX_INTERFACES_INCLUDE_DIRECTORIES}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${LIVOX_INTERFACES} "rosidl_typesupport_cpp")
target_link_libraries(LivoxRosDriver2_Driver "${cpp_typesupport_target}")

# link libraries
target_link_libraries(LivoxRosDriver2_Driver
  ${LIVOX_LIDAR_SDK_LIBRARY}
  ${LIVOX_INTERFACE_TARGET}   # for custom msgs
  ${PPT_LIBRARY}
  ${Boost_LIBRARY}
  ${PCL_LIBRARIES}
  ${APR_LIBRARIES}
)

rclcpp_components_register_nodes(LivoxRosDriver2_Driver PLUGIN "livox_ros::DriverNode" EXECUTABLE LivoxRosDriver2_Driver)

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY tmux
  USE_SOURCE_PERMISSIONS
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()

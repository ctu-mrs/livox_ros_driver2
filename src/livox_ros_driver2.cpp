//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <iostream>
#include <chrono>
#include <vector>
#include <csignal>
#include <thread>

#include <livox_ros_driver2.h>
#include <lddc.h>
#include <lds_lidar.h>

#include <mrs_lib/node.h>
#include <mrs_lib/param_loader.h>

using namespace livox_ros;

namespace livox_ros
{

class DriverNode : public mrs_lib::Node {
public:
  DriverNode(rclcpp::NodeOptions options);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  void shutdown();

  void PointCloudDataPollThread();
  void ImuDataPollThread();

  std::unique_ptr<Lddc>        lddc_ptr_;
  std::shared_ptr<std::thread> pointclouddata_poll_thread_;
  std::shared_ptr<std::thread> imudata_poll_thread_;
  std::shared_future<void>     future_;
  std::promise<void>           exit_signal_;
};

void DriverNode::shutdown() {

  lddc_ptr_->lds_->RequestExit();
  exit_signal_.set_value();
  pointclouddata_poll_thread_->join();
  imudata_poll_thread_->join();
}

DriverNode::DriverNode(rclcpp::NodeOptions node_options) : mrs_lib::Node("livox_driver_node", node_options) {

  node_  = this->this_node_ptr();
  clock_ = node_->get_clock();

  RCLCPP_INFO(node_->get_logger(), "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);

  rclcpp::on_shutdown([this]() { this->shutdown(); });

  /** Init default system parameter */
  int         xfer_format  = kPointCloud2Msg;
  int         multi_topic  = 0;
  int         data_src     = kSourceRawLidar;
  double      publish_freq = 10.0; /* Hz */
  int         output_type  = kOutputToRos;
  std::string frame_id;

  mrs_lib::ParamLoader param_loader(node_);

  // OLD param declaration with defaults
  /* node_->declare_parameter("xfer_format", xfer_format); */
  /* node_->declare_parameter("multi_topic", 0); */
  /* node_->declare_parameter("data_src", data_src); */
  /* node_->declare_parameter("publish_freq", 10.0); */
  /* node_->declare_parameter("output_data_type", output_type); */
  /* node_->declare_parameter("frame_id", "frame_default"); */
  /* node_->declare_parameter("user_config_path", "path_default"); */
  /* node_->declare_parameter("cmdline_input_bd_code", "000000000000001"); */
  /* node_->declare_parameter("lvx_file_path", "/home/livox/livox_test.lvx"); */

  param_loader.loadParam("xfer_format", xfer_format);
  param_loader.loadParam("multi_topic", multi_topic);
  param_loader.loadParam("data_src", data_src);
  param_loader.loadParam("publish_freq", publish_freq);
  param_loader.loadParam("output_data_type", output_type);
  param_loader.loadParam("frame_id", frame_id);

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;
  } else {
    publish_freq = publish_freq;
  }

  future_ = exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type, publish_freq, frame_id);
  lddc_ptr_->SetRosNode(node_);

  if (data_src == kSourceRawLidar) {
    RCLCPP_INFO(node_->get_logger(), "Data Source is raw lidar.");

    std::string user_config_path;
    param_loader.loadParam("user_config_path", user_config_path);
    RCLCPP_INFO(node_->get_logger(), "Config file : %s", user_config_path.c_str());

    std::string cmdline_bd_code;
    param_loader.loadParam("cmdline_input_bd_code", cmdline_bd_code);

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
    lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar(user_config_path))) {
      RCLCPP_INFO(node_->get_logger(), "Init lds lidar success!");
    } else {
      RCLCPP_INFO(node_->get_logger(), "Init lds lidar fail!");
    }
  } else {
    RCLCPP_INFO(node_->get_logger(), "Invalid data src (%d), please check the launch file", data_src);
  }

  pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, this);
  imudata_poll_thread_        = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, this);

  RCLCPP_INFO(node_->get_logger(), "initialized");
}

}  // namespace livox_ros

/* PointCloudDataPollThread() //{ */

void DriverNode::PointCloudDataPollThread() {
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributePointCloudData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}

//}

/* PointCloudDataPollThread() //{ */

void DriverNode::ImuDataPollThread() {
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributeImuData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}

//}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::DriverNode)

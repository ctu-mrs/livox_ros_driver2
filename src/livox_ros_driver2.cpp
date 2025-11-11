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

/* includes //{ */

#include <chrono>
#include <csignal>
#include <thread>

#include <livox_ros_driver2.h>
#include <lddc.h>
#include <lds_lidar.h>

#include <mrs_lib/node.h>
#include <mrs_lib/param_loader.h>

//}

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

/* shutdown() //{ */

void DriverNode::shutdown() {

  lddc_ptr_->lds_->RequestExit();
  exit_signal_.set_value();
  pointclouddata_poll_thread_->join();
  imudata_poll_thread_->join();
}

//}

/* constructor DriverNode //{ */

DriverNode::DriverNode(rclcpp::NodeOptions node_options) : mrs_lib::Node("livox_driver_node", node_options) {

  node_  = this->this_node_ptr();
  clock_ = node_->get_clock();

  RCLCPP_INFO(node_->get_logger(), "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);

  rclcpp::on_shutdown([this]() { this->shutdown(); });

  /** Init default system parameter */
  bool        multi_topic;
  double      publish_freq;
  std::string frame_id;
  std::string user_config_path;

  mrs_lib::ParamLoader param_loader(node_);

  param_loader.loadParam("multi_topic", multi_topic);
  param_loader.loadParam("publish_freq", publish_freq);
  param_loader.loadParam("frame_id", frame_id);
  param_loader.loadParam("user_config_path", user_config_path);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "failed to load non-optional parameters!");
    rclcpp::shutdown();
    exit(1);
  }

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
    RCLCPP_WARN(node_->get_logger(), "capping publisher frequency to 100 Hz");
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;
    RCLCPP_WARN(node_->get_logger(), "capping publisher frequency to 0.5 Hz");
  }

  future_ = exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  lddc_ptr_ = std::make_unique<Lddc>(node_, multi_topic, publish_freq, frame_id);

  RCLCPP_INFO(node_->get_logger(), "config file: %s", user_config_path.c_str());

  LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
  lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

  if ((read_lidar->InitLdsLidar(user_config_path))) {
    RCLCPP_INFO(node_->get_logger(), "succeeded to initialize LiDARs");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "failed to initialize LiDARs");
    rclcpp::shutdown();
    exit(1);
  }

  pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, this);
  imudata_poll_thread_        = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, this);

  RCLCPP_INFO(node_->get_logger(), "initialized");
}

//}

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

/* ImuDataPollThread() //{ */

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

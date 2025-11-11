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

#ifndef LIVOX_ROS_DRIVER2_LDDC_H_
#define LIVOX_ROS_DRIVER2_LDDC_H_

#include <lds.h>

#include <rclcpp/rclcpp.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <livox_ros_driver2/msg/custom_point.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include <mrs_lib/publisher_handler.h>

namespace livox_ros
{

/** Send pointcloud message Data to ros subscriber or save them in rosbag file */
typedef enum
{
  kOutputToRos        = 0,
  kOutputToRosBagFile = 1,
} DestinationOfMessageOutput;

/** Type-Definitions based on ROS versions */
template <typename MessageT>
using Publisher    = rclcpp::Publisher<MessageT>;
using PublisherPtr = std::shared_ptr<rclcpp::PublisherBase>;
using PointCloud2  = sensor_msgs::msg::PointCloud2;
using PointField   = sensor_msgs::msg::PointField;
using CustomMsg    = livox_ros_driver2::msg::CustomMsg;
using CustomPoint  = livox_ros_driver2::msg::CustomPoint;
using ImuMsg       = sensor_msgs::msg::Imu;

class DriverNode;

class Lddc final {
public:
  Lddc(rclcpp::Node::SharedPtr node, int multi_topic, double frq, std::string& frame_id);

  ~Lddc();

  int  RegisterLds(Lds* lds);
  void DistributePointCloudData(void);
  void DistributeImuData(void);
  void PrepareExit(void);

  // void SetRosPub(ros::Publisher *pub) { global_pub_ = pub; };  // NOT USED
  void SetPublishFrq(uint32_t frq) {
    publish_frq_ = frq;
  }

public:
  Lds* lds_;

private:
  void PollingLidarPointCloudData(uint8_t index, LidarDevice* lidar);
  void PollingLidarImuData(uint8_t index, LidarDevice* lidar);

  void PublishImuData(LidarImuDataQueue& imu_data_queue, const uint8_t index);

  void InitPointcloud2MsgHeader(const uint8_t index, PointCloud2& cloud);
  void InitPointcloud2Msg(const uint8_t index, const StoragePacket& pkg, PointCloud2& cloud, uint64_t& timestamp);
  void PublishPointcloud2Data(const uint8_t index, uint64_t timestamp, const PointCloud2& cloud);

  void InitCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg, uint8_t index);
  void FillPointsToCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg);
  void PublishCustomPointData(const CustomMsg& livox_msg, const uint8_t index);

  void InitImuMsg(const uint8_t index, const ImuData& imu_data, ImuMsg& imu_msg, uint64_t& timestamp);

  void FillPointsToCustomMsg(CustomMsg& livox_msg, LivoxPointXyzrtlt* src_point, uint32_t num, uint32_t offset_time, uint32_t point_interval,
                             uint32_t echo_num);

  PublisherPtr CreatePublisher(uint8_t msg_type, std::string& topic_name, uint32_t queue_size);

  std::shared_ptr<mrs_lib::PublisherHandler<PointCloud2>> GetCurrentPcPublisher(uint8_t index);
  std::shared_ptr<mrs_lib::PublisherHandler<CustomMsg>>   GetCurrentCustomPublisher(uint8_t index);
  std::shared_ptr<mrs_lib::PublisherHandler<ImuMsg>>      GetCurrentImuPublisher(uint8_t index);

private:
  rclcpp::Node::SharedPtr node_;

  bool     use_multi_topic_;
  double   publish_frq_;
  uint32_t publish_period_ns_;

  std::string frame_id_;

  /* PublisherPtr private_pub_[kMaxSourceLidar]; */
  /* PublisherPtr global_pub_; */
  /* PublisherPtr private_imu_pub_[kMaxSourceLidar]; */
  /* PublisherPtr global_imu_pub_; */

  std::vector<std::shared_ptr<mrs_lib::PublisherHandler<PointCloud2>>> private_pc_pubs_;
  std::vector<std::shared_ptr<mrs_lib::PublisherHandler<CustomMsg>>>   private_custom_pubs_;
  std::vector<std::shared_ptr<mrs_lib::PublisherHandler<ImuMsg>>>      private_imu_pubs_;

  std::shared_ptr<mrs_lib::PublisherHandler<PointCloud2>> global_pc_pub_;
  std::shared_ptr<mrs_lib::PublisherHandler<CustomMsg>>   global_custom_pub_;
  std::shared_ptr<mrs_lib::PublisherHandler<ImuMsg>>      global_imu_pub_;
};

}  // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER2_LDDC_H_

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

#include <lddc.h>
#include <comm/ldq.h>
#include <comm/comm.h>

#include <inttypes.h>
#include <iostream>
#include <math.h>
#include <stdint.h>

#include <lds_lidar.h>

//}

namespace livox_ros
{

/* Lddc() //{ */

/** Lidar Data Distribute Control--------------------------------------------*/
Lddc::Lddc(rclcpp::Node::SharedPtr node, int multi_topic, double radius_invalid, std::string& frame_id, bool publish_invalid)
    : node_(node), use_multi_topic_(multi_topic), radius_invalid_(radius_invalid), frame_id_(frame_id), publish_invalid_(publish_invalid) {

  lds_ = nullptr;
}

//}

/* ~Lddc() //{ */

Lddc::~Lddc() {

  PrepareExit();

  std::cout << "~Lddc() called" << std::endl;
}

//}

/* RegisterLds() //{ */

int Lddc::RegisterLds(Lds* lds) {

  if (lds_ == nullptr) {
    lds_ = lds;
    return 0;
  } else {
    return -1;
  }
}

//}

/* DistributePointCloudData() //{ */

void Lddc::DistributePointCloudData(void) {

  if (!lds_) {
    RCLCPP_ERROR(node_->get_logger(), "lds_ is not registered");
    return;
  }

  if (lds_->IsRequestExit()) {
    RCLCPP_INFO(node_->get_logger(), "DistributePointCloudData(): isRequestExit()");
    return;
  }

  lds_->pcd_semaphore_.Wait();

  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {

    uint32_t        lidar_id = i;
    LidarDevice*    lidar    = &lds_->lidars_[lidar_id];
    LidarDataQueue* p_queue  = &lidar->data;

    if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr)) {
      continue;
    }

    PollingLidarPointCloudData(lidar_id, lidar);
  }
}

//}

/* DistributeImuData() //{ */

void Lddc::DistributeImuData(void) {

  if (!lds_) {
    RCLCPP_ERROR(node_->get_logger(), "lds_ is not registered");
    return;
  }

  if (lds_->IsRequestExit()) {
    RCLCPP_INFO(node_->get_logger(), "DistributeImuData(): isRequestExit()");
    return;
  }

  lds_->imu_semaphore_.Wait();

  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {

    uint32_t           lidar_id = i;
    LidarDevice*       lidar    = &lds_->lidars_[lidar_id];
    LidarImuDataQueue* p_queue  = &lidar->imu_data;

    if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr)) {
      continue;
    }

    PollingLidarImuData(lidar_id, lidar);
  }
}

//}

/* PollingLidarPointCloudData() //{ */

void Lddc::PollingLidarPointCloudData(uint8_t index, LidarDevice* lidar) {

  LidarDataQueue* p_queue = &lidar->data;

  if (p_queue == nullptr || p_queue->storage_packet == nullptr) {
    return;
  }

  while (!lds_->IsRequestExit() && !QueueIsEmpty(p_queue)) {

    while (!QueueIsEmpty(p_queue)) {

      StoragePacket pkg;
      QueuePop(p_queue, &pkg);

      if (pkg.points.empty() && pkg.points_invalid.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Publish point cloud2 failed, the pkg points is empty.");
        continue;
      }

      {
        {
          uint64_t    timestamp = 0;
          PointCloud2 cloud;
          InitPointcloud2Msg(index, pkg, cloud, timestamp);
          PublishPointcloud2Data(index, timestamp, cloud);
        }

        if (publish_invalid_) {

          uint64_t    timestamp = 0;
          PointCloud2 cloud_invalid;
          InitPointcloud2Msg(index, pkg, cloud_invalid, timestamp);
          PublishInvalidPointcloud2Data(index, timestamp, cloud_invalid);
        }
      }

      {
        CustomMsg livox_msg;

        InitCustomMsg(livox_msg, pkg, index);
        FillPointsToCustomMsg(livox_msg, pkg);
        PublishCustomPointData(livox_msg, index);
      }
    }
  }
}

//}

/* PollingLidarImuData() //{ */

void Lddc::PollingLidarImuData(uint8_t index, LidarDevice* lidar) {

  LidarImuDataQueue& p_queue = lidar->imu_data;

  while (!lds_->IsRequestExit() && !p_queue.Empty()) {
    PublishImuData(p_queue, index);
  }
}

//}

/* PrepareExit() //{ */

void Lddc::PrepareExit(void) {

  if (lds_) {
    lds_->PrepareExit();
    lds_ = nullptr;
  }
}

//}

/* InitPointcloud2MsgHeader() //{ */

void Lddc::InitPointcloud2MsgHeader(const uint8_t index, PointCloud2& cloud) {

  std::stringstream ss;

  if (use_multi_topic_) {
    ss << frame_id_ << "_" << index;
  } else {
    ss << frame_id_;
  }

  cloud.header.frame_id.assign(ss.str());

  cloud.height = 1;
  cloud.width  = 0;
  cloud.fields.resize(7);
  cloud.fields[0].offset   = 0;
  cloud.fields[0].name     = "x";
  cloud.fields[0].count    = 1;
  cloud.fields[0].datatype = PointField::FLOAT32;
  cloud.fields[1].offset   = 4;
  cloud.fields[1].name     = "y";
  cloud.fields[1].count    = 1;
  cloud.fields[1].datatype = PointField::FLOAT32;
  cloud.fields[2].offset   = 8;
  cloud.fields[2].name     = "z";
  cloud.fields[2].count    = 1;
  cloud.fields[2].datatype = PointField::FLOAT32;
  cloud.fields[3].offset   = 12;
  cloud.fields[3].name     = "intensity";
  cloud.fields[3].count    = 1;
  cloud.fields[3].datatype = PointField::FLOAT32;
  cloud.fields[4].offset   = 16;
  cloud.fields[4].name     = "tag";
  cloud.fields[4].count    = 1;
  cloud.fields[4].datatype = PointField::UINT8;
  cloud.fields[5].offset   = 17;
  cloud.fields[5].name     = "line";
  cloud.fields[5].count    = 1;
  cloud.fields[5].datatype = PointField::UINT8;
  cloud.fields[6].offset   = 18;
  cloud.fields[6].name     = "timestamp";
  cloud.fields[6].count    = 1;
  cloud.fields[6].datatype = PointField::FLOAT64;

  cloud.point_step = sizeof(LivoxPointXyzrtlt);
}

//}

/* InitPointcloud2Msg() //{ */

void Lddc::InitPointcloud2Msg(const uint8_t index, const StoragePacket& pkg, PointCloud2& cloud, uint64_t& timestamp) {

  InitPointcloud2MsgHeader(index, cloud);

  cloud.point_step = sizeof(LivoxPointXyzrtlt);

  cloud.width    = pkg.points_num;
  cloud.row_step = cloud.width * cloud.point_step;

  cloud.is_bigendian = false;
  cloud.is_dense     = true;

  if (!pkg.points.empty()) {
    timestamp = pkg.base_time;
  }

  cloud.header.stamp = rclcpp::Time(timestamp);

  std::vector<LivoxPointXyzrtlt> points;

  for (size_t i = 0; i < pkg.points_num; ++i) {

    LivoxPointXyzrtlt point;

    point.x            = pkg.points[i].x;
    point.y            = pkg.points[i].y;
    point.z            = pkg.points[i].z;
    point.reflectivity = pkg.points[i].intensity;
    point.tag          = pkg.points[i].tag;
    point.line         = pkg.points[i].line;
    point.timestamp    = static_cast<double>(pkg.points[i].offset_time);

    points.push_back(std::move(point));
  }

  cloud.data.resize(pkg.points_num * sizeof(LivoxPointXyzrtlt));
  memcpy(cloud.data.data(), points.data(), pkg.points_num * sizeof(LivoxPointXyzrtlt));
}

//}

/* InitInvalidPointcloud2Msg() //{ */

void Lddc::InitInvalidPointcloud2Msg(const uint8_t index, const StoragePacket& pkg, PointCloud2& cloud_invalid, uint64_t& timestamp) {

  InitPointcloud2MsgHeader(index, cloud_invalid);

  cloud_invalid.point_step = sizeof(LivoxPointXyzrtlt);

  cloud_invalid.width    = pkg.points_invalid_num;
  cloud_invalid.row_step = cloud_invalid.width * cloud_invalid.point_step;

  cloud_invalid.is_bigendian = false;
  cloud_invalid.is_dense     = true;

  if (!pkg.points_invalid.empty()) {
    timestamp = pkg.base_time;
  }

  cloud_invalid.header.stamp = rclcpp::Time(timestamp);

  std::vector<LivoxPointXyzrtlt> points_invalid;

  for (size_t i = 0; i < pkg.points_invalid_num; ++i) {

    LivoxPointXyzrtlt point;

    point.x            = pkg.points_invalid[i].x;
    point.y            = pkg.points_invalid[i].y;
    point.z            = pkg.points_invalid[i].z;
    point.reflectivity = pkg.points_invalid[i].intensity;
    point.tag          = pkg.points_invalid[i].tag;
    point.line         = pkg.points_invalid[i].line;
    point.timestamp    = static_cast<double>(pkg.points_invalid[i].offset_time);

    points_invalid.push_back(std::move(point));
  }

  cloud_invalid.data.resize(pkg.points_invalid_num * sizeof(LivoxPointXyzrtlt));
  memcpy(cloud_invalid.data.data(), points_invalid.data(), pkg.points_invalid_num * sizeof(LivoxPointXyzrtlt));
}

//}

/* PublishPointcloud2Data() //{ */

void Lddc::PublishPointcloud2Data(const uint8_t index, const uint64_t timestamp, const PointCloud2& cloud) {

  auto publisher_ptr = GetCurrentPcPublisher(index);

  publisher_ptr->publish(cloud);
}

//}

/* PublishInvalidPointcloud2Data() //{ */

void Lddc::PublishInvalidPointcloud2Data(const uint8_t index, const uint64_t timestamp, const PointCloud2& cloud) {

  auto publisher_ptr = GetCurrentInvalidPcPublisher(index);

  publisher_ptr->publish(cloud);
}

//}

/* InitCustomMsg() //{ */

void Lddc::InitCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg, uint8_t index) {

  std::stringstream ss;

  if (use_multi_topic_) {
    ss << frame_id_ << "_" << index;
  } else {
    ss << frame_id_;
  }

  livox_msg.header.frame_id.assign(ss.str());

  uint64_t timestamp = 0;

  if (!pkg.points.empty()) {
    timestamp = pkg.base_time;
  }

  livox_msg.timebase = timestamp;

  livox_msg.header.stamp = rclcpp::Time(timestamp);

  livox_msg.point_num = pkg.points_num;

  if (lds_->lidars_[index].lidar_type == kLivoxLidarType) {
    livox_msg.lidar_id = lds_->lidars_[index].handle;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Init custom msg lidar id failed, the index: %u.", index);
    livox_msg.lidar_id = 0;
  }
}

//}

/* FillPointsToCustomMsg() //{ */

void Lddc::FillPointsToCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg) {

  uint32_t                       points_num = pkg.points_num;
  const std::vector<PointXyzlt>& points     = pkg.points;

  for (uint32_t i = 0; i < points_num; ++i) {

    CustomPoint point;

    point.x            = points[i].x;
    point.y            = points[i].y;
    point.z            = points[i].z;
    point.reflectivity = points[i].intensity;
    point.tag          = points[i].tag;
    point.line         = points[i].line;
    point.offset_time  = static_cast<uint32_t>(points[i].offset_time - pkg.base_time);

    livox_msg.points.push_back(std::move(point));
  }
}

//}

/* PublishCustomPointData() //{ */

void Lddc::PublishCustomPointData(const CustomMsg& livox_msg, const uint8_t index) {

  auto publisher_ptr = GetCurrentCustomPublisher(index);

  publisher_ptr->publish(livox_msg);
}

//}

/* InitImuMsg() //{ */

void Lddc::InitImuMsg(uint8_t index, const ImuData& imu_data, ImuMsg& imu_msg, uint64_t& timestamp) {

  std::stringstream ss;

  if (use_multi_topic_) {
    ss << frame_id_ << "_" << index;
  } else {
    ss << frame_id_;
  }

  imu_msg.header.frame_id = ss.str();

  timestamp            = imu_data.time_stamp;
  imu_msg.header.stamp = rclcpp::Time(timestamp);

  imu_msg.angular_velocity.x    = imu_data.gyro_x;
  imu_msg.angular_velocity.y    = imu_data.gyro_y;
  imu_msg.angular_velocity.z    = imu_data.gyro_z;
  imu_msg.linear_acceleration.x = imu_data.acc_x * 9.81;
  imu_msg.linear_acceleration.y = imu_data.acc_y * 9.81;
  imu_msg.linear_acceleration.z = imu_data.acc_z * 9.81;
}

//}

/* PublishImuData() //{ */

void Lddc::PublishImuData(LidarImuDataQueue& imu_data_queue, const uint8_t index) {

  ImuData imu_data;

  if (!imu_data_queue.Pop(imu_data)) {
    // printf("Publish imu data failed, imu data queue pop failed.\n");
    return;
  }

  ImuMsg   imu_msg;
  uint64_t timestamp;
  InitImuMsg(index, imu_data, imu_msg, timestamp);

  auto publisher_ptr = GetCurrentImuPublisher(index);

  publisher_ptr->publish(imu_msg);
}

//}

/* GetCurrentPcPublisher() //{ */

std::shared_ptr<mrs_lib::PublisherHandler<PointCloud2>> Lddc::GetCurrentPcPublisher(uint8_t handle) {

  /* uint32_t queue_size = kMinEthPacketQueueSize; */

  if (use_multi_topic_) {

    if (!private_pc_pubs_[handle]) {

      std::stringstream ss;

      ss << "~/lidar_" << lds_->lidars_[handle].handle << "/points";

      RCLCPP_INFO(node_->get_logger(), "creating publisher for lidar %d PointCloud2 on topic '%s'", handle, ss.str().c_str());

      mrs_lib::PublisherHandlerOptions opts;

      opts.node = node_;
      opts.qos  = rclcpp::SensorDataQoS();

      private_pc_pubs_[handle] = std::make_shared<mrs_lib::PublisherHandler<PointCloud2>>(opts, ss.str());
    }

    return private_pc_pubs_[handle];

  } else {

    if (!global_pc_pub_) {

      std::string topic_name("~/points");

      RCLCPP_INFO(node_->get_logger(), "creating publisher for PointCloud2 on topic '%s'", topic_name.c_str());

      mrs_lib::PublisherHandlerOptions opts;

      opts.node = node_;
      opts.qos  = rclcpp::SensorDataQoS();

      global_pc_pub_ = std::make_shared<mrs_lib::PublisherHandler<PointCloud2>>(opts, topic_name);
    }

    return global_pc_pub_;
  }
}

//}

/* GetCurrentInvalidPcPublisher() //{ */

std::shared_ptr<mrs_lib::PublisherHandler<PointCloud2>> Lddc::GetCurrentInvalidPcPublisher(uint8_t handle) {

  /* uint32_t queue_size = kMinEthPacketQueueSize; */

  if (use_multi_topic_) {

    if (!private_invalid_pc_pubs_[handle]) {

      std::stringstream ss;

      ss << "~/lidar_" << lds_->lidars_[handle].handle << "/invalid_points";

      RCLCPP_INFO(node_->get_logger(), "creating publisher for lidar %d invalid PointCloud2 on topic '%s'", handle, ss.str().c_str());

      mrs_lib::PublisherHandlerOptions opts;

      opts.node = node_;
      opts.qos  = rclcpp::SensorDataQoS();

      private_invalid_pc_pubs_[handle] = std::make_shared<mrs_lib::PublisherHandler<PointCloud2>>(opts, ss.str());
    }

    return private_invalid_pc_pubs_[handle];

  } else {

    if (!global_invalid_pc_pub_) {

      std::string topic_name("~/invalid_points");

      RCLCPP_INFO(node_->get_logger(), "creating publisher for invalid PointCloud2 on topic '%s'", topic_name.c_str());

      mrs_lib::PublisherHandlerOptions opts;

      opts.node = node_;
      opts.qos  = rclcpp::SensorDataQoS();

      global_invalid_pc_pub_ = std::make_shared<mrs_lib::PublisherHandler<PointCloud2>>(opts, topic_name);
    }

    return global_invalid_pc_pub_;
  }
}

//}

/* GetCurrentCustomPublisher() //{ */

std::shared_ptr<mrs_lib::PublisherHandler<CustomMsg>> Lddc::GetCurrentCustomPublisher(uint8_t handle) {

  /* uint32_t queue_size = kMinEthPacketQueueSize; */

  if (use_multi_topic_) {

    if (!private_custom_pubs_[handle]) {

      std::stringstream ss;

      ss << "~/lidar_" << lds_->lidars_[handle].handle << "/custom";

      RCLCPP_INFO(node_->get_logger(), "creating publisher for lidar %d CustomMsg on topic '%s'", handle, ss.str().c_str());

      mrs_lib::PublisherHandlerOptions opts;

      opts.node = node_;
      opts.qos  = rclcpp::SensorDataQoS();

      private_custom_pubs_[handle] = std::make_shared<mrs_lib::PublisherHandler<CustomMsg>>(opts, ss.str());
    }

    return private_custom_pubs_[handle];

  } else {

    if (!global_custom_pub_) {

      std::string topic_name("~/custom");

      RCLCPP_INFO(node_->get_logger(), "creating publisher for CustomMsg on topic '%s'", topic_name.c_str());

      mrs_lib::PublisherHandlerOptions opts;

      opts.node = node_;
      opts.qos  = rclcpp::SensorDataQoS();

      global_custom_pub_ = std::make_shared<mrs_lib::PublisherHandler<CustomMsg>>(opts, topic_name);
    }

    return global_custom_pub_;
  }
}

//}

/* GetCurrentImuPublisher() //{ */

std::shared_ptr<mrs_lib::PublisherHandler<ImuMsg>> Lddc::GetCurrentImuPublisher(uint8_t handle) {

  /* uint32_t queue_size = kMinEthPacketQueueSize; */

  if (use_multi_topic_) {

    if (!private_imu_pubs_[handle]) {

      std::stringstream ss;

      ss << "~/lidar_" << lds_->lidars_[handle].handle << "/imu";

      RCLCPP_INFO(node_->get_logger(), "creating publisher for lidar %d IMU on topic '%s'", handle, ss.str().c_str());

      mrs_lib::PublisherHandlerOptions opts;

      opts.node = node_;
      opts.qos  = rclcpp::SensorDataQoS();

      private_imu_pubs_[handle] = std::make_shared<mrs_lib::PublisherHandler<ImuMsg>>(opts, ss.str());
    }

    return private_imu_pubs_[handle];

  } else {

    if (!global_imu_pub_) {

      std::string topic_name("~/imu");

      mrs_lib::PublisherHandlerOptions opts;

      opts.node = node_;
      opts.qos  = rclcpp::SensorDataQoS();

      global_imu_pub_ = std::make_shared<mrs_lib::PublisherHandler<ImuMsg>>(opts, topic_name);
    }

    return global_imu_pub_;
  }
}

//}

}  // namespace livox_ros

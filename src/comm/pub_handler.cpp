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

#include "pub_handler.h"

#include <cstdlib>
#include <chrono>
#include <iostream>
#include <limits>

namespace livox_ros
{

std::atomic<bool> PubHandler::is_timestamp_sync_;

PubHandler& pub_handler() {
  static PubHandler handler;
  return handler;
}

void PubHandler::Init() {
}

void PubHandler::Uninit() {
  if (lidar_listen_id_ > 0) {
    LivoxLidarRemovePointCloudObserver(lidar_listen_id_);
    lidar_listen_id_ = 0;
  }

  RequestExit();

  if (point_process_thread_ && point_process_thread_->joinable()) {
    point_process_thread_->join();
    point_process_thread_ = nullptr;
  } else {
    /* */
  }
}

void PubHandler::RequestExit() {
  is_quit_.store(true);
}

void PubHandler::SetPointCloudConfig(const double publish_freq) {
  publish_interval_           = (kNsPerSecond / (publish_freq * 10)) * 10;
  publish_interval_tolerance_ = publish_interval_ - kNsTolerantFrameTimeDeviation;
  publish_interval_ms_        = publish_interval_ / kRatioOfMsToNs;
  if (!point_process_thread_) {
    point_process_thread_ = std::make_shared<std::thread>(&PubHandler::RawDataProcess, this);
  }
  return;
}

void PubHandler::SetImuDataCallback(ImuDataCallback cb, void* client_data) {
  imu_client_data_ = client_data;
  imu_callback_    = cb;
}

void PubHandler::AddLidarsParams(LidarExtParameter& params_extr, const ParamsHandler &params_hand) {
  std::unique_lock<std::mutex> lock(packet_mutex_);
  uint32_t                     id = 0;
  GetLidarId(params_extr.lidar_type, params_extr.handle, id);
  lidar_extrinsics_[id] = params_extr;
  lidar_params_[id]     = params_hand;
}

void PubHandler::ClearAllLidarsExtrinsicParams() {
  std::unique_lock<std::mutex> lock(packet_mutex_);
  lidar_extrinsics_.clear();
}

void PubHandler::SetPointCloudsCallback(PointCloudsCallback cb, void* client_data) {
  pub_client_data_ = client_data;
  points_callback_ = cb;
  lidar_listen_id_ = LivoxLidarAddPointCloudObserver(OnLivoxLidarPointCloudCallback, this);
}

void PubHandler::OnLivoxLidarPointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
  PubHandler* self = (PubHandler*)client_data;
  if (!self) {
    return;
  }

  if (data->time_type != kTimestampTypeNoSync) {
    is_timestamp_sync_.store(true);
  } else {
    is_timestamp_sync_.store(false);
  }

  if (data->data_type == kLivoxLidarImuData) {
    if (self->imu_callback_) {
      RawImuPoint* imu = (RawImuPoint*)data->data;
      ImuData      imu_data;
      imu_data.lidar_type = static_cast<uint8_t>(LidarProtoType::kLivoxLidarType);
      imu_data.handle     = handle;
      imu_data.time_stamp = GetEthPacketTimestamp(data->time_type, data->timestamp, sizeof(data->timestamp));
      imu_data.gyro_x     = imu->gyro_x;
      imu_data.gyro_y     = imu->gyro_y;
      imu_data.gyro_z     = imu->gyro_z;
      imu_data.acc_x      = imu->acc_x;
      imu_data.acc_y      = imu->acc_y;
      imu_data.acc_z      = imu->acc_z;
      self->imu_callback_(&imu_data, self->imu_client_data_);
    }
    return;
  }
  RawPacket packet        = {};
  packet.handle           = handle;
  packet.lidar_type       = LidarProtoType::kLivoxLidarType;
  packet.extrinsic_enable = false;
  if (dev_type == LivoxLidarDeviceType::kLivoxLidarTypeIndustrialHAP) {
    packet.line_num = kLineNumberHAP;
  } else if (dev_type == LivoxLidarDeviceType::kLivoxLidarTypeMid360) {
    packet.line_num = kLineNumberMid360;
  } else {
    packet.line_num = kLineNumberDefault;
  }
  packet.data_type      = data->data_type;
  packet.point_num      = data->dot_num;
  packet.point_interval = data->time_interval * 100 / data->dot_num;  // ns
  packet.time_stamp     = GetEthPacketTimestamp(data->time_type, data->timestamp, sizeof(data->timestamp));
  uint32_t length       = data->length - sizeof(LivoxLidarEthernetPacket) + 1;
  packet.raw_data.insert(packet.raw_data.end(), data->data, data->data + length);
  {
    std::unique_lock<std::mutex> lock(self->packet_mutex_);
    self->raw_packet_queue_.push_back(packet);
  }
  self->packet_condition_.notify_one();

  return;
}

void PubHandler::PublishPointCloud() {
  // publish point
  if (points_callback_) {
    points_callback_(&frame_, pub_client_data_);
  }
}

void PubHandler::CheckTimer(uint32_t id) {

  if (PubHandler::is_timestamp_sync_.load()) {  // Enable time synchronization
    auto&    process_handler = lidar_process_handlers_[id];
    uint64_t recent_time_ms  = process_handler->GetRecentTimeStamp() / kRatioOfMsToNs;
    if ((recent_time_ms % publish_interval_ms_ != 0) || recent_time_ms == 0) {
      return;
    }

    uint64_t diff = process_handler->GetRecentTimeStamp() - process_handler->GetLidarBaseTime();
    if (diff < publish_interval_tolerance_) {
      return;
    }

    frame_.base_time[frame_.lidar_num] = process_handler->GetLidarBaseTime();
    points_[id].clear();
    points_invalid_[id].clear();
    process_handler->GetLidarPointClouds(points_[id]);
    process_handler->GetLidarPointCloudsInvalid(points_invalid_[id]);
    if (points_[id].empty() && points_invalid_[id].empty()) {
      return;
    }
    PointPacket& lidar_point       = frame_.lidar_point[frame_.lidar_num];
    lidar_point.lidar_type         = LidarProtoType::kLivoxLidarType;  // TODO:
    lidar_point.handle             = id;
    lidar_point.points_num         = points_[id].size();
    lidar_point.points             = points_[id].data();
    lidar_point.points_invalid_num = points_invalid_[id].size();
    lidar_point.points_invalid     = points_invalid_[id].data();
    frame_.lidar_num++;

    if (frame_.lidar_num != 0) {
      PublishPointCloud();
      frame_.lidar_num = 0;
    }
  } else {  // Disable time synchronization
    auto now_time = std::chrono::high_resolution_clock::now();
    // First Set
    static bool first = true;
    if (first) {
      last_pub_time_ = now_time;
      first          = false;
      return;
    }
    if (now_time - last_pub_time_ < std::chrono::nanoseconds(publish_interval_)) {
      return;
    }
    last_pub_time_ += std::chrono::nanoseconds(publish_interval_);
    for (auto& process_handler : lidar_process_handlers_) {
      frame_.base_time[frame_.lidar_num] = process_handler.second->GetLidarBaseTime();
      uint32_t handle                    = process_handler.first;
      points_[handle].clear();
      points_invalid_[handle].clear();
      process_handler.second->GetLidarPointClouds(points_[handle]);
      process_handler.second->GetLidarPointCloudsInvalid(points_invalid_[handle]);
      if (points_[handle].empty() && points_invalid_[handle].empty()) {
        continue;
      }

      PointPacket& lidar_point       = frame_.lidar_point[frame_.lidar_num];
      lidar_point.lidar_type         = LidarProtoType::kLivoxLidarType;  // TODO:
      lidar_point.handle             = handle;
      lidar_point.points_num         = points_[handle].size();
      lidar_point.points             = points_[handle].data();
      lidar_point.points_invalid_num = points_invalid_[handle].size();
      lidar_point.points_invalid     = points_invalid_[handle].data();
      frame_.lidar_num++;
    }

    PublishPointCloud();
    frame_.lidar_num = 0;
  }
}

void PubHandler::RawDataProcess() {
  RawPacket raw_data;
  while (!is_quit_.load()) {
    {
      std::unique_lock<std::mutex> lock(packet_mutex_);
      if (raw_packet_queue_.empty()) {
        packet_condition_.wait_for(lock, std::chrono::milliseconds(500));
        if (raw_packet_queue_.empty()) {
          continue;
        }
      }
      raw_data = raw_packet_queue_.front();
      raw_packet_queue_.pop_front();
    }
    uint32_t id = 0;
    GetLidarId(raw_data.lidar_type, raw_data.handle, id);
    if (lidar_process_handlers_.find(id) == lidar_process_handlers_.end()) {
      lidar_process_handlers_[id].reset(new LidarPubHandler());
      lidar_process_handlers_[id]->SetParamsHandler(lidar_params_[id]);
    }
    auto& process_handler = lidar_process_handlers_[id];
    if (lidar_extrinsics_.find(id) != lidar_extrinsics_.end()) {
      lidar_process_handlers_[id]->SetLidarsExtParam(lidar_extrinsics_[id]);
    }
    process_handler->PointCloudProcess(raw_data);
    CheckTimer(id);
  }
}

bool PubHandler::GetLidarId(LidarProtoType lidar_type, uint32_t handle, uint32_t& id) {
  if (lidar_type == kLivoxLidarType) {
    id = handle;
    return true;
  }
  return false;
}

uint64_t PubHandler::GetEthPacketTimestamp(uint8_t timestamp_type, uint8_t* time_stamp, uint8_t size) {
  LdsStamp time;
  memcpy(time.stamp_bytes, time_stamp, size);

  if (timestamp_type == kTimestampTypeGptpOrPtp || timestamp_type == kTimestampTypeGps) {
    return time.stamp;
  }

  return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}

/*******************************/
/*  LidarPubHandler Definitions*/
LidarPubHandler::LidarPubHandler() : is_set_extrinsic_params_(false) {
}

uint64_t LidarPubHandler::GetLidarBaseTime() {
  if (points_clouds_.empty()) {
    return 0;
  }
  return points_clouds_.at(0).offset_time;
}

void LidarPubHandler::GetLidarPointClouds(std::vector<PointXyzlt>& points_clouds) {
  std::lock_guard<std::mutex> lock(mutex_);
  points_clouds.swap(points_clouds_);
}

void LidarPubHandler::GetLidarPointCloudsInvalid(std::vector<PointXyzlt>& points_clouds_invalid) {
  std::lock_guard<std::mutex> lock(mutex_invalid_);
  points_clouds_invalid.swap(points_clouds_invalid_);
}

uint64_t LidarPubHandler::GetRecentTimeStamp() {
  if (points_clouds_.empty()) {
    return 0;
  }
  return points_clouds_.back().offset_time;
}

uint32_t LidarPubHandler::GetLidarPointCloudsSize() {
  std::lock_guard<std::mutex> lock(mutex_);
  return points_clouds_.size();
}

// convert to standard format and extrinsic compensate
void LidarPubHandler::PointCloudProcess(RawPacket& pkt) {
  if (pkt.lidar_type == LidarProtoType::kLivoxLidarType) {
    LivoxLidarPointCloudProcess(pkt);
  } else {
    static bool flag = false;
    if (!flag) {
      std::cout << "error, unsupported protocol type: " << static_cast<int>(pkt.lidar_type) << std::endl;
      flag = true;
    }
  }
}

void LidarPubHandler::LivoxLidarPointCloudProcess(RawPacket& pkt) {
  switch (pkt.data_type) {
    case kLivoxLidarCartesianCoordinateHighData:
      ProcessCartesianHighPoint(pkt);
      break;
    case kLivoxLidarCartesianCoordinateLowData:
      ProcessCartesianLowPoint(pkt);
      break;
    case kLivoxLidarSphericalCoordinateData:
      ProcessSphericalPoint(pkt);
      break;
    default:
      std::cout << "unknown data type: " << static_cast<int>(pkt.data_type) << " !!" << std::endl;
      break;
  }
}

void LidarPubHandler::SetLidarsExtParam(LidarExtParameter lidar_param) {
  if (is_set_extrinsic_params_) {
    return;
  }
  extrinsic_.trans[0] = lidar_param.param.x;
  extrinsic_.trans[1] = lidar_param.param.y;
  extrinsic_.trans[2] = lidar_param.param.z;

  double cos_roll  = cos(static_cast<double>(lidar_param.param.roll * PI / 180.0));
  double cos_pitch = cos(static_cast<double>(lidar_param.param.pitch * PI / 180.0));
  double cos_yaw   = cos(static_cast<double>(lidar_param.param.yaw * PI / 180.0));
  double sin_roll  = sin(static_cast<double>(lidar_param.param.roll * PI / 180.0));
  double sin_pitch = sin(static_cast<double>(lidar_param.param.pitch * PI / 180.0));
  double sin_yaw   = sin(static_cast<double>(lidar_param.param.yaw * PI / 180.0));

  extrinsic_.rotation[0][0] = cos_pitch * cos_yaw;
  extrinsic_.rotation[0][1] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
  extrinsic_.rotation[0][2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;

  extrinsic_.rotation[1][0] = cos_pitch * sin_yaw;
  extrinsic_.rotation[1][1] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
  extrinsic_.rotation[1][2] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;

  extrinsic_.rotation[2][0] = -sin_pitch;
  extrinsic_.rotation[2][1] = sin_roll * cos_pitch;
  extrinsic_.rotation[2][2] = cos_roll * cos_pitch;

  is_set_extrinsic_params_ = true;
}

void LidarPubHandler::SetParamsHandler(const ParamsHandler &params) {
  filters_params_ = params;
}

void LidarPubHandler::ProcessCartesianHighPoint(RawPacket& pkt) {
  LivoxLidarCartesianHighRawPoint* raw   = (LivoxLidarCartesianHighRawPoint*)pkt.raw_data.data();
  PointXyzlt                       point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    if (pkt.extrinsic_enable) {
      point.x = raw[i].x / 1000.0;
      point.y = raw[i].y / 1000.0;
      point.z = raw[i].z / 1000.0;
    } else {
      point.x =
          (raw[i].x * extrinsic_.rotation[0][0] + raw[i].y * extrinsic_.rotation[0][1] + raw[i].z * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) / 1000.0;
      point.y =
          (raw[i].x * extrinsic_.rotation[1][0] + raw[i].y * extrinsic_.rotation[1][1] + raw[i].z * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) / 1000.0;
      point.z =
          (raw[i].x * extrinsic_.rotation[2][0] + raw[i].y * extrinsic_.rotation[2][1] + raw[i].z * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) / 1000.0;
    }

    const double dist = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

    // skip frame measurements
    if (dist < filters_params_.range_min) {
      continue;
    }

    // skip dust detections
    point.intensity = raw[i].reflectivity;
    if (dist < filters_params_.intensity_range && raw[i].reflectivity < filters_params_.intensity_min_value) {
      continue;
    }

    point.line        = i % pkt.line_num;
    point.tag         = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;

    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

void LidarPubHandler::ProcessCartesianLowPoint(RawPacket& pkt) {
  LivoxLidarCartesianLowRawPoint* raw   = (LivoxLidarCartesianLowRawPoint*)pkt.raw_data.data();
  PointXyzlt                      point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    if (pkt.extrinsic_enable) {
      point.x = raw[i].x / 100.0;
      point.y = raw[i].y / 100.0;
      point.z = raw[i].z / 100.0;
    } else {
      point.x =
          (raw[i].x * extrinsic_.rotation[0][0] + raw[i].y * extrinsic_.rotation[0][1] + raw[i].z * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) / 100.0;
      point.y =
          (raw[i].x * extrinsic_.rotation[1][0] + raw[i].y * extrinsic_.rotation[1][1] + raw[i].z * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) / 100.0;
      point.z =
          (raw[i].x * extrinsic_.rotation[2][0] + raw[i].y * extrinsic_.rotation[2][1] + raw[i].z * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) / 100.0;
    }

    const double dist = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

    // skip frame measurements
    if (dist < filters_params_.range_min) {
      continue;
    }

    // skip dust detections
    point.intensity = raw[i].reflectivity;
    if (dist < filters_params_.intensity_range && raw[i].reflectivity < filters_params_.intensity_min_value) {
      continue;
    }

    point.line        = i % pkt.line_num;
    point.tag         = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;

    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

void LidarPubHandler::ProcessSphericalPoint(RawPacket& pkt) {
  LivoxLidarSpherPoint* raw     = (LivoxLidarSpherPoint*)pkt.raw_data.data();
  const double          rad2deg = PI / (100.0 * 180.0);

  std::vector<PointXyzlt> valid_points_buffer_;
  std::vector<PointXyzlt> invalid_points_buffer_;

  valid_points_buffer_.reserve(pkt.point_num);
  invalid_points_buffer_.reserve(pkt.point_num);

  for (size_t i = 0; i < pkt.point_num; i++) {

    double       depth = std::fabs(double(raw[i].depth * 0.001));
    const double theta  = raw[i].theta * rad2deg;
    const double phi    = raw[i].phi * rad2deg;

    const bool pt_invalid = depth < 0.001f;

    if (pt_invalid) {

      depth = 1.0;

    } else {

      // skip frame measurements
      if (depth < filters_params_.range_min) {
        continue;
      }

      // skip dust detections
      if (depth < filters_params_.intensity_range && raw[i].reflectivity < filters_params_.intensity_min_value) {
        continue;
      }
    }

    const double src_x = depth * sin(theta) * cos(phi);
    const double src_y = depth * sin(theta) * sin(phi);
    const double src_z = depth * cos(theta);

    PointXyzlt pt;
    if (pkt.extrinsic_enable) {
      pt.x = src_x;
      pt.y = src_y;
      pt.z = src_z;
    } else {
      pt.x = src_x * extrinsic_.rotation[0][0] + src_y * extrinsic_.rotation[0][1] + src_z * extrinsic_.rotation[0][2] + (extrinsic_.trans[0] * 0.001);
      pt.y = src_x * extrinsic_.rotation[1][0] + src_y * extrinsic_.rotation[1][1] + src_z * extrinsic_.rotation[1][2] + (extrinsic_.trans[1] * 0.001);
      pt.z = src_x * extrinsic_.rotation[2][0] + src_y * extrinsic_.rotation[2][1] + src_z * extrinsic_.rotation[2][2] + (extrinsic_.trans[2] * 0.001);
    }

    pt.intensity   = raw[i].reflectivity;
    pt.line        = i % pkt.line_num;
    pt.tag         = raw[i].tag;
    pt.offset_time = pkt.time_stamp + i * pkt.point_interval;

    if (pt_invalid) {
      invalid_points_buffer_.push_back(pt);
    } else {
      valid_points_buffer_.push_back(pt);
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.insert(points_clouds_.end(), valid_points_buffer_.begin(), valid_points_buffer_.end());
  }

  {
    std::lock_guard<std::mutex> lock(mutex_invalid_);
    points_clouds_invalid_.insert(points_clouds_invalid_.end(), invalid_points_buffer_.begin(), invalid_points_buffer_.end());
  }
}

}  // namespace livox_ros

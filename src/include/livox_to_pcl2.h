#pragma once

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver2/CustomMsg.h>

//}

namespace livox_ros
{
/* class LivoxPCLtoPCL2 //{ */
class LivoxPCLtoPCL2 : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized = false;

  ros::Subscriber _sub_lidar_livox_pcl;

  ros::Publisher _pub_lidar_pcl2;

  void lidarLivoxCallback(const livox_ros_driver2::CustomMsg::ConstPtr &msg);
  void convertLivoxPCLtoPCL2(const sensor_msgs::PointCloud2::Ptr &cloud, const livox_ros_driver2::CustomMsg::ConstPtr &livox_msg);
};
//}

}  // namespace livox_ros

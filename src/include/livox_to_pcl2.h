#pragma once

/* includes //{ */

#include <nodelet/nodelet.h>
#include <mrs_lib/subscribe_handler.h>

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

  mrs_lib::SubscribeHandler<livox_ros_driver2::CustomMsg> _sub_lidar_livox_pcl;

  ros::Publisher _pub_lidar_pcl2;

  void lidarLivoxCallback(const livox_ros_driver2::CustomMsg::ConstPtr msg);
  void convertLivoxPCLtoPCL2(const sensor_msgs::PointCloud2::Ptr &cloud, const livox_ros_driver2::CustomMsg::ConstPtr &livox_msg);
};
//}

}  // namespace livox_ros

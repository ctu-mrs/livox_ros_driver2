#include <livox_to_pcl2.h>

namespace livox_ros
{

/* onInit() //{ */
void LivoxPCLtoPCL2::onInit() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  _sub_lidar_livox_pcl = nh.subscribe("lidar_in", 1, &LivoxPCLtoPCL2::lidarLivoxCallback, this, ros::TransportHints().tcpNoDelay());

  _pub_lidar_pcl2 = nh.advertise<sensor_msgs::PointCloud2>("lidar_out", 1);

  NODELET_INFO_ONCE("[LivoxPCLtoPCL2] Nodelet initialized");

  is_initialized = true;
}
//}

/* lidarLivoxCallback() //{ */
// Convert livox_ros_driver2::CustomMsg to sensor_msgs::PointCloud2 message type
void LivoxPCLtoPCL2::lidarLivoxCallback(const livox_ros_driver2::CustomMsg::ConstPtr& msg) {

  if (!is_initialized) {
    return;
  }

  if (_pub_lidar_pcl2.getNumSubscribers() > 0) {

    const sensor_msgs::PointCloud2::Ptr cloud_ros = boost::make_shared<sensor_msgs::PointCloud2>();
    convertLivoxPCLtoPCL2(cloud_ros, msg);

    _pub_lidar_pcl2.publish(cloud_ros);
  }
}
//}

/*//{ convertLivoxPCLtoPCL2() */
void LivoxPCLtoPCL2::convertLivoxPCLtoPCL2(const sensor_msgs::PointCloud2::Ptr& cloud, const livox_ros_driver2::CustomMsg::ConstPtr& livox_msg) {
  cloud->header = livox_msg->header;
  cloud->fields.resize(6);

  cloud->fields[0].offset   = 0;
  cloud->fields[0].name     = "x";
  cloud->fields[0].count    = 1;
  cloud->fields[0].datatype = sensor_msgs::PointField::FLOAT32;

  cloud->fields[1].offset   = 4;
  cloud->fields[1].name     = "y";
  cloud->fields[1].count    = 1;
  cloud->fields[1].datatype = sensor_msgs::PointField::FLOAT32;

  cloud->fields[2].offset   = 8;
  cloud->fields[2].name     = "z";
  cloud->fields[2].count    = 1;
  cloud->fields[2].datatype = sensor_msgs::PointField::FLOAT32;

  cloud->fields[3].offset   = 12;
  cloud->fields[3].name     = "intensity";
  cloud->fields[3].count    = 1;
  cloud->fields[3].datatype = sensor_msgs::PointField::FLOAT32;

  cloud->fields[4].offset   = 16;
  cloud->fields[4].name     = "tag";
  cloud->fields[4].count    = 1;
  cloud->fields[4].datatype = sensor_msgs::PointField::UINT8;

  cloud->fields[5].offset   = 17;
  cloud->fields[5].name     = "line";
  cloud->fields[5].count    = 1;
  cloud->fields[5].datatype = sensor_msgs::PointField::UINT8;

  cloud->point_step = 18;
  cloud->row_step   = cloud->point_step * livox_msg->point_num;
  cloud->data.resize(cloud->row_step);

  uint8_t* livox_data_ptr = cloud->data.data();
  for (const auto& point : livox_msg->points) {
    *(reinterpret_cast<float*>(livox_data_ptr + 0))  = point.x;
    *(reinterpret_cast<float*>(livox_data_ptr + 4))  = point.y;
    *(reinterpret_cast<float*>(livox_data_ptr + 8))  = point.z;
    *(reinterpret_cast<float*>(livox_data_ptr + 12)) = static_cast<float>(point.reflectivity);
    *(livox_data_ptr + 16)                           = point.tag;
    *(livox_data_ptr + 17)                           = point.line;

    livox_data_ptr += cloud->point_step;
  }

  cloud->width        = livox_msg->point_num;
  cloud->height       = 1;
  cloud->is_bigendian = false;
  cloud->is_dense     = true;
}
/*//}*/

}  // namespace livox_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(livox_ros::LivoxPCLtoPCL2, nodelet::Nodelet);

#include <glog/logging.h>
#include <ros/ros.h>
#include "lidar_undistortion/lidar_undistorter.h"

int main(int argc, char **argv) {
  // Start logging
  google::InitGoogleLogging(argv[0]);

  // Register with ROS master
  ros::init(argc, argv, "lidar_undistortion");

  // Create node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Launch the lidar undistorter
  lidar_undistortion::LidarUndistorter lidar_undistorter(nh, nh_private);

  // Spin
  ros::spin();

  // Exit normally
  return 0;
}

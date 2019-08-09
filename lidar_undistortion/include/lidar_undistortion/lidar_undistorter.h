#ifndef LIDAR_UNDISTORTION_LIDAR_UNDISTORTER_H_
#define LIDAR_UNDISTORTION_LIDAR_UNDISTORTER_H_

#include <ouster_ros/point_os1.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2_ros/transform_listener.h>

#include <minkindr_conversions/kindr_msg.h>
// TODO(victorr): Look into using kindr_tf.h

namespace lidar_undistortion {
class LidarUndistorter {
 public:
  LidarUndistorter(ros::NodeHandle nh, ros::NodeHandle nh_private);

  void pointcloudCallback(const sensor_msgs::PointCloud2::Ptr &pointcloud_msg);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber pointcloud_sub_;
  ros::Publisher corrected_pointcloud_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
}  // namespace lidar_undistortion

#endif  // LIDAR_UNDISTORTION_LIDAR_UNDISTORTER_H_

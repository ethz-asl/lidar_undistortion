#ifndef LIDAR_UNDISTORTION_LIDAR_UNDISTORTER_H_
#define LIDAR_UNDISTORTION_LIDAR_UNDISTORTER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Eigen>

namespace lidar_undistortion {
class LidarUndistorter {
 public:
  LidarUndistorter(ros::NodeHandle nh, ros::NodeHandle nh_private);

  void pointcloudCallback(const sensor_msgs::PointCloud2 &pointcloud_msg);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber pointcloud_sub_;
  ros::Publisher corrected_pointcloud_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // TODO(victorr): Explain why this is here
  template <typename T>
  void transformMsgToEigen(const geometry_msgs::Transform &transform_msg,
                           T &transform) {  // NOLINT
    transform =
        Eigen::Translation3f(transform_msg.translation.x,
                             transform_msg.translation.y,
                             transform_msg.translation.z) *
        Eigen::Quaternionf(transform_msg.rotation.w, transform_msg.rotation.x,
                           transform_msg.rotation.y, transform_msg.rotation.z);
  }
};
}  // namespace lidar_undistortion

#endif  // LIDAR_UNDISTORTION_LIDAR_UNDISTORTER_H_

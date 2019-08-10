#include "lidar_undistortion/lidar_undistorter.h"

#include <ouster_ros/point_os1.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>

namespace lidar_undistortion {
LidarUndistorter::LidarUndistorter(ros::NodeHandle nh,
                                   ros::NodeHandle nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      tf_buffer_(ros::Duration(10)),
      tf_listener_(tf_buffer_) {
  pointcloud_sub_ = nh_.subscribe("pointcloud", 100,
                                  &LidarUndistorter::pointcloudCallback, this);
  corrected_pointcloud_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>(
      "pointcloud_corrected", 100, false);
}

void LidarUndistorter::pointcloudCallback(
    const sensor_msgs::PointCloud2 &pointcloud_msg) {
  const std::string odom_frame = "odom";
  const std::string lidar_frame = "os1_lidar";

  // Convert the pointcloud to PCL
  pcl::PointCloud<ouster_ros::OS1::PointOS1> pointcloud;
  pcl::fromROSMsg(pointcloud_msg, pointcloud);

  // Assert that the pointcloud is not empty
  if (pointcloud.empty()) return;

  // Get the start and end times of the pointcloud
  ros::Time t_start = pointcloud_msg.header.stamp +
                      ros::Duration(pointcloud.points.begin()->t * 1e-9);
  ros::Time t_end = pointcloud_msg.header.stamp +
                    ros::Duration((--pointcloud.points.end())->t * 1e-9);

  try {
    // Wait for all transforms to become available
    if (!tf_buffer_.canTransform(odom_frame, lidar_frame, t_end,
                                 ros::Duration(0.25))) {
      ROS_WARN("Could not lookup transform to correct pointcloud.");
      return;
    }

    // Get the frame that the cloud should be expressed in
    geometry_msgs::TransformStamped msg_T_O_C_original =
        tf_buffer_.lookupTransform(odom_frame, lidar_frame, t_start);
    Eigen::Affine3f T_O_C_original;
    transformMsgToEigen(msg_T_O_C_original.transform, T_O_C_original);
    Eigen::Affine3f T_C_O_original = T_O_C_original.inverse();

    // Correct the distortion on all points, using the LiDAR's true pose at
    // each point's timestamp
    uint32_t last_transform_update_t = 0;
    Eigen::Affine3f T_O_C_correct = T_C_O_original;
    for (ouster_ros::OS1::PointOS1 &point : pointcloud.points) {
      // Check if the current point's timestamp differs from the previous one
      // If so, lookup the new corresponding transform
      if (point.t != last_transform_update_t) {
        last_transform_update_t = point.t;
        ros::Time point_t =
            pointcloud_msg.header.stamp + ros::Duration(0, point.t);
        geometry_msgs::TransformStamped msg_T_O_C_correct =
            tf_buffer_.lookupTransform(odom_frame, lidar_frame, point_t);
        transformMsgToEigen(msg_T_O_C_correct.transform, T_O_C_correct);
      }

      // Correct the point's distortion, by transforming it into the fixed
      // frame based on the LiDAR sensor's current true pose, and then transform
      // it back into the lidar scan frame
      point = pcl::transformPoint(point, T_C_O_original * T_O_C_correct);
    }
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  // Publish the corrected pointcloud
  sensor_msgs::PointCloud2 pointcloud_corrected_msg;
  pcl::toROSMsg(pointcloud, pointcloud_corrected_msg);
  corrected_pointcloud_pub_.publish(pointcloud_corrected_msg);
}
}  // namespace lidar_undistortion

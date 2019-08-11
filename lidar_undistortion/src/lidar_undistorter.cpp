#include "lidar_undistortion/lidar_undistorter.h"
#include <ouster_ros/point_os1.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>

namespace lidar_undistortion {
LidarUndistorter::LidarUndistorter(ros::NodeHandle nh,
                                   ros::NodeHandle nh_private)
    : fixed_frame_id_("odom"),
      lidar_frame_id_("os1_lidar"),
      tf_buffer_(ros::Duration(10)),
      tf_listener_(tf_buffer_) {
  // Subscribe to the undistorted pointcloud topic
  pointcloud_sub_ = nh.subscribe("pointcloud", 100,
                                 &LidarUndistorter::pointcloudCallback, this);

  // Advertise the corrected pointcloud topic
  corrected_pointcloud_pub_ = nh_private.advertise<sensor_msgs::PointCloud2>(
      "pointcloud_corrected", 100, false);

  // Read the odom and lidar frame names from ROS params
  nh_private.param("odom_frame_id", fixed_frame_id_, fixed_frame_id_);
  nh_private.param("lidar_frame_id", lidar_frame_id_, lidar_frame_id_);
}

void LidarUndistorter::pointcloudCallback(
    const sensor_msgs::PointCloud2 &pointcloud_msg) {
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
    if (!waitForTransform(lidar_frame_id_, fixed_frame_id_, t_end, 0.05,
                          0.25)) {
      ROS_WARN(
          "Could not get correction transform within allotted time. "
          "Skipping pointcloud.");
      return;
    }

    // Get the frame that the cloud should be expressed in
    geometry_msgs::TransformStamped msg_T_F_S_original =
        tf_buffer_.lookupTransform(fixed_frame_id_, lidar_frame_id_, t_start);
    Eigen::Affine3f T_F_S_original;
    transformMsgToEigen(msg_T_F_S_original.transform, T_F_S_original);

    // Compute the transform used to project the corrected pointcloud back into
    // lidar's scan frame, for more info see this header for this class.
    Eigen::Affine3f T_S_F_original = T_F_S_original.inverse();

    // Correct the distortion on all points, using the LiDAR's true pose at
    // each point's timestamp
    uint32_t last_transform_update_t = 0;
    Eigen::Affine3f T_S_original__S_corrected = Eigen::Affine3f::Identity();
    for (ouster_ros::OS1::PointOS1 &point : pointcloud.points) {
      // Check if the current point's timestamp differs from the previous one
      // If so, lookup the new corresponding transform
      if (point.t != last_transform_update_t) {
        last_transform_update_t = point.t;
        ros::Time point_t =
            pointcloud_msg.header.stamp + ros::Duration(0, point.t);
        geometry_msgs::TransformStamped msg_T_F_S_correct =
            tf_buffer_.lookupTransform(fixed_frame_id_, lidar_frame_id_,
                                       point_t);
        Eigen::Affine3f T_F_S_correct;
        transformMsgToEigen(msg_T_F_S_correct.transform, T_F_S_correct);
        T_S_original__S_corrected = T_S_F_original * T_F_S_correct;
      }

      // Correct the point's distortion, by transforming it into the fixed
      // frame based on the LiDAR sensor's current true pose, and then transform
      // it back into the lidar scan frame
      point = pcl::transformPoint(point, T_S_original__S_corrected);
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

bool LidarUndistorter::waitForTransform(const std::string &from_frame_id,
                                        const std::string &to_frame_id,
                                        const ros::Time &frame_timestamp,
                                        const double &sleep_between_retries__s,
                                        const double &timeout__s) {
  // Total time spent waiting for the updated pose
  ros::WallDuration t_waited(0.0);
  // Maximum time to wait before giving up
  ros::WallDuration t_max(timeout__s);
  // Timeout between each update attempt
  const ros::WallDuration t_sleep(sleep_between_retries__s);
  while (t_waited < t_max) {
    if (tf_buffer_.canTransform(fixed_frame_id_, lidar_frame_id_,
                                frame_timestamp)) {
      return true;
    }
    t_sleep.sleep();
    t_waited += t_sleep;
  }
  ROS_WARN("Waited %.3fs, but still could not get the TF from %s to %s",
           t_waited.toSec(), lidar_frame_id_.c_str(), fixed_frame_id_.c_str());
  return false;
}
}  // namespace lidar_undistortion

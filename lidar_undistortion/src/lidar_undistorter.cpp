#include "lidar_undistortion/lidar_undistorter.h"

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
    const sensor_msgs::PointCloud2::Ptr &pointcloud_msg) {
  std::string odom_frame = "odom";
  std::string lidar_frame = "os1_lidar";

  // Conver the pointcloud to PCL
  pcl::PointCloud<ouster_ros::OS1::PointOS1> pointcloud_out;
  pcl::fromROSMsg(*pointcloud_msg, pointcloud_out);

  // Get the start and end times of the pointcloud
  // TODO(victorr): Assert that the pointcloud isn't empty
  ros::Time t_start = pointcloud_msg->header.stamp +
                      ros::Duration(pointcloud_out.points.begin()->t * 1e-9);
  ros::Time t_end = pointcloud_msg->header.stamp +
                    ros::Duration((--pointcloud_out.points.end())->t * 1e-9);

  try {
    // Wait for all transforms to become available
    if (!tf_buffer_.canTransform(odom_frame, lidar_frame, t_end,
                                 ros::Duration(0.25))) {
      ROS_WARN("Could not lookup transform to correct pointcloud.");
      return;
    }

    // Get the frame that the cloud should be expressed in
    geometry_msgs::TransformStamped msg_T_O_C_original =
        tf_buffer_.lookupTransform(odom_frame, lidar_frame,
                                   pointcloud_msg->header.stamp);
    kindr::minimal::QuatTransformationTemplate<double> T_O_C_original;
    tf::transformMsgToKindr(msg_T_O_C_original.transform, &T_O_C_original);

    // Loop over all points
    // TODO(victorr): Vectorize this into batches (e.g. per column)
    for (ouster_ros::OS1::PointOS1 &point : pointcloud_out.points) {
      // Get the frame that the point was actually in
      geometry_msgs::TransformStamped msg_T_O_C_correct =
          tf_buffer_.lookupTransform(
              odom_frame, lidar_frame,
              pointcloud_msg->header.stamp + ros::Duration(0, point.t));
      kindr::minimal::QuatTransformationTemplate<double> T_O_C_correct;
      tf::transformMsgToKindr(msg_T_O_C_correct.transform, &T_O_C_correct);

      // Correct the pose of the current point
      kindr::minimal::QuatTransformationTemplate<double>::Position point_pose(
          point.x, point.y, point.z);
      point_pose = T_O_C_original.inverse() * T_O_C_correct * point_pose;

      // Store our changes
      point.x = point_pose.x();
      point.y = point_pose.y();
      point.z = point_pose.z();
    }
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  // Publish the corrected pointcloud
  sensor_msgs::PointCloud2 pointcloud_corrected_msg;
  pcl::toROSMsg(pointcloud_out, pointcloud_corrected_msg);
  corrected_pointcloud_pub_.publish(pointcloud_corrected_msg);
}
}  // namespace lidar_undistortion

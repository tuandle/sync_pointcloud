#ifndef SYNC_POINTCLOUD_H
#define SYNC_POINTCLOUD_H

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/pass_through.h>
#include <message_filters/simple_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nodelet/nodelet.h>
#include <pcl_ros/point_cloud.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <Eigen/Eigen>

#include <cmath>
#include <limits>

namespace sync_pointcloud {

constexpr int kQueueSize = 10;
class SyncPointcloud {
 public:
  SyncPointcloud(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  void syncAndPub(const sensor_msgs::ImageConstPtr& mask_image_in,
                  const sensor_msgs::ImageConstPtr& depth_image_in);

 private:
  int getQueueSize() const;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher pub_pointcloud_;
  image_transport::ImageTransport it_;
  /*image_transport::TransportHints hints("raw", ros::TransportHints(),
                                        getPrivateNodeHandle());*/

  // subscribers
  image_transport::SubscriberFilter mask_image_sub_;
  image_transport::SubscriberFilter depth_image_sub_;

  /*typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Imu, sensor_msgs::Image, sensor_msgs::Image>
      ImuCamSyncPolicy;*/
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image>
      DepthMaskSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<DepthMaskSyncPolicy>>
      depth_mask_sync_ptr_;

  int queue_size_;
  int cam_width_, cam_height_;
  double fx_, fy_, cx_, cy_;
};

}  // namespace sync_pointcloud

#endif
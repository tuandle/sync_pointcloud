#include "sync_pointcloud/sync_pointcloud.h"

namespace sync_pointcloud {
int SyncPointcloud::getQueueSize() const {
  int queue_size;
  nh_private_.param("queue_size", queue_size, kQueueSize);
  if (queue_size < 1) {
    ROS_ERROR("Queue size must be >= 1, setting to 1");
    queue_size = 1;
  }
  return queue_size;
}

SyncPointcloud::SyncPointcloud(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      it_(nh),
      queue_size_(getQueueSize()),
      mask_image_sub_(it_, "/rs_sensor/argmax", queue_size_),
      depth_image_sub_(it_, "/aligned_depth_to_color/image_raw", queue_size_),
      cam_width_(640),
      cam_height_(480),
      /*fx_(384.4273986816406),
      fy_(384.4273986816406),
      cx_(322.5639953613281),
      cy_(236.31768798828125)*/
      fx_(612.7745361328125),
      fy_(612.6051635742188),
      cx_(324.9194030761719),
      cy_(238.48410034179688) {
  nh_private_.param("queue_size", queue_size_, kQueueSize);
  if (queue_size_ < 1) {
    ROS_ERROR("Queue size must be at least 1, set to 1");
    queue_size_ = 1;
  }
  pub_pointcloud_ =
      nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("pole_points", 1);
  depth_mask_sync_ptr_ =
      std::make_shared<message_filters::Synchronizer<DepthMaskSyncPolicy>>(
          DepthMaskSyncPolicy(queue_size_), mask_image_sub_, depth_image_sub_);
  depth_mask_sync_ptr_->registerCallback(
      boost::bind(&SyncPointcloud::syncAndPub, this, _1, _2));
}

void SyncPointcloud::syncAndPub(
    const sensor_msgs::ImageConstPtr& mask_img_in,
    const sensor_msgs::ImageConstPtr& depth_img_in) {
  cv_bridge::CvImagePtr mask_ptr =
      cv_bridge::toCvCopy(mask_img_in, mask_img_in->encoding);
  /*if (mask_img_in->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    (mask_ptr->image).convertTo(mask_ptr->image, CV_8UC1);*/
  cv::Mat mask = mask_ptr->image;

  cv_bridge::CvImagePtr depth_ptr;
  depth_ptr = cv_bridge::toCvCopy(depth_img_in, depth_img_in->encoding);
  constexpr double kDepthScalingFactor = 0.001;
  if (depth_img_in->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    (depth_ptr->image)
        .convertTo(depth_ptr->image, CV_32FC1, kDepthScalingFactor);
  cv::Mat depth = depth_ptr->image;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  for (int i = 0; i < cam_width_; ++i)
    for (int j = 0; j < cam_height_; ++j) {
      float depth_value = depth.at<float>(j, i);
      if ((mask.at<ushort>(j, i) == 0) || (depth_value == 0)) continue;
      Eigen::Vector3f cam_point;
      cam_point(0) = (i - cx_) * depth_value / fx_;
      cam_point(1) = (j - cy_) * depth_value / fy_;
      cam_point(2) = depth_value;
      pcl::PointXYZI p;
      p.x = cam_point(0);
      p.y = cam_point(1);
      p.z = cam_point(2);
      p.intensity = mask.at<ushort>(j, i);
      pointcloud->push_back(p);
    }
  /*pointcloud->header.frame_id = depth_img_in->header.frame_id;*/
  pointcloud->header.frame_id = "camera_depth_optical_frame";
  // pointcloud->header.stamp = depth_img_in->header.stamp;
  pub_pointcloud_.publish(pointcloud);
}
}  // namespace sync_pointcloud
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "sync_pointcloud/sync_pointcloud.h"

namespace sync_pointcloud {

class SyncPointcloudNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      sync_pointcloud_ = std::make_shared<SyncPointcloud>(
          getNodeHandle(), getPrivateNodeHandle());
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<SyncPointcloud> sync_pointcloud_;
};
}  // namespace sync_pointcloud

PLUGINLIB_EXPORT_CLASS(sync_pointcloud::SyncPointcloudNodelet,
                       nodelet::Nodelet);

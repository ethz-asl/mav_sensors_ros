#include <memory>

#include <log++.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "mav_sensors_ros/imu.h"

namespace mav_sensors_ros {

class ImuNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      imu_ = std::make_unique<Imu>(getPrivateNodeHandle());
      if (!imu_->init()) imu_.release();
    } catch (std::runtime_error e) {
      LOG(E, "%s", e.what());
    }
  }

  std::unique_ptr<Imu> imu_;
};
}  // namespace mav_sensors_ros

PLUGINLIB_EXPORT_CLASS(mav_sensors_ros::ImuNodelet, nodelet::Nodelet)
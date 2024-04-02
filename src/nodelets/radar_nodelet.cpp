#include <memory>

#include <log++.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "mav_sensors_ros/radar.h"

namespace mav_sensors_ros {

class RadarNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      radar_ = std::make_unique<Radar>(getPrivateNodeHandle());
      if (!radar_->init()) radar_.release();
    } catch (std::runtime_error e) {
      LOG(E, "%s", e.what());
    }
  }

  std::unique_ptr<Radar> radar_;
};
}  // namespace mav_sensors_ros

PLUGINLIB_EXPORT_CLASS(mav_sensors_ros::RadarNodelet, nodelet::Nodelet)
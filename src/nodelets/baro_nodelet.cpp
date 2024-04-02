#include <memory>

#include <log++.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "mav_sensors_ros/baro.h"

namespace mav_sensors_ros {

class BaroNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      baro_ = std::make_unique<Baro>(getPrivateNodeHandle());
      if (!baro_->init()) baro_.release();
    } catch (std::runtime_error e) {
      LOG(E, "%s", e.what());
    }
  }

  std::unique_ptr<Baro> baro_;
};
}  // namespace mav_sensors_ros

PLUGINLIB_EXPORT_CLASS(mav_sensors_ros::BaroNodelet, nodelet::Nodelet)
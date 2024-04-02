#pragma once

#include <mav_sensors_drivers/barometer/bmp390.h>
#include <ros/ros.h>

#include "mav_sensors_ros/base_sensor.h"

namespace mav_sensors_ros {
class Baro : public BaseSensor {
 private:
  void readSensor() override;
  bool openSensor() override;

  mav_sensors::BMP390<mav_sensors::Spi> baro_;
  ros::Publisher baro_pub_;
  ros::Publisher temp_pub_;

 public:
  Baro(const ros::NodeHandle& nh_private);
  ~Baro();
};
}  // namespace mav_sensors_ros
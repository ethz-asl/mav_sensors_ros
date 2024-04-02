/*
BSD 3-Clause License

Copyright (c) 2024 ETH Zurich, Autonomous Systems Lab, Rik Girod

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "mav_sensors_ros/baro.h"

#include <log++.h>
#include <mav_sensors_core/sensor_config.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>

using namespace mav_sensors_ros;

Baro::Baro(const ros::NodeHandle& nh_private) : BaseSensor(nh_private) {}

Baro::~Baro() { baro_.close(); }

bool Baro::openSensor() {
  baro_pub_ = nh_private_.advertise<sensor_msgs::FluidPressure>("pressure", 1);
  temp_pub_ = nh_private_.advertise<sensor_msgs::Temperature>("temperature", 1);

  std::string path;
  if (!nh_private_.getParam("path", path)) {
    LOG(F, "Failed to read barometer path.");
    return false;
  } else {
    LOG(I, "Opening barometer on path: " << path.c_str());
  }

  mav_sensors::SensorConfig cfg;
  cfg.set("path", path);
  baro_.setConfig(cfg);
  if (!baro_.open()) {
    LOG(F, "Failed to open barometer.");
    return false;
  }

  return true;
}

void Baro::readSensor() {
  // Read sensor data.
  auto measurement = baro_.read();

  if (std::get<0>(measurement).has_value() &&
      std::get<2>(measurement).has_value()) {
    // Publish pressure measurements.
    LOG_FIRST(I, 1, "Publishing first pressure measurement.");
    sensor_msgs::FluidPressure msg;
    msg.header.stamp = toRosTime(std::get<2>(measurement).value());
    msg.header.frame_id = frame_id_;
    msg.fluid_pressure = std::get<0>(measurement).value();
    baro_pub_.publish(msg);
  }
  if (std::get<1>(measurement).has_value() &&
      std::get<2>(measurement).has_value()) {
    // Publish temperature measurements.
    LOG_FIRST(I, 1, "Publishing first temperature measurement.");
    sensor_msgs::Temperature msg;
    msg.header.stamp = toRosTime(std::get<2>(measurement).value());
    msg.header.frame_id = frame_id_;
    msg.temperature = std::get<1>(measurement).value();
    temp_pub_.publish(msg);
  }
}
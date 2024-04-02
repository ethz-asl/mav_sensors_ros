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
#pragma once

#include <deque>

#include <mav_sensors_core/common/vec.h>
#include <mav_sensors_drivers/imu/bmi088.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include "mav_sensors_ros/base_sensor.h"

namespace mav_sensors_ros {
class Imu : public BaseSensor {
 private:
  void readSensor() override;
  bool openSensor() override;

  mav_sensors::Bmi088<mav_sensors::Spi> imu_;
  ros::Publisher imu_pub_, bias_pub_;
  ros::ServiceServer calibrate_srv_;
  std::deque<mav_sensors::vec3<double>> omega_;
  mav_sensors::vec3<double> b_g_{0.0, 0.0, 0.0};
  int bias_samples_ = 0;

 public:
  Imu(const ros::NodeHandle& nh_private);
  ~Imu();

  bool calibrate(std_srvs::Trigger::Request& req,
                 std_srvs::Trigger::Response& res);
};
}  // namespace mav_sensors_ros
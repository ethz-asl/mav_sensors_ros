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

#include <Eigen/SVD>
#include <eigen3/Eigen/Dense>
#include <mav_sensors_drivers/radar/xwr18xx_mmw_demo.h>
#include <mav_sensors_drivers/sensor_types/Radar.h>
#include <ros/ros.h>

#include "mav_sensors_ros/base_sensor.h"

namespace mav_sensors_ros {
class Radar : public BaseSensor {
 private:
  void readSensor() override;
  bool openSensor() override;

  mav_sensors::Xwr18XxMmwDemo radar_;
  ros::Publisher ls_vel_pub_;
  ros::Publisher cfar_pub_;

 public:
  Radar(const ros::NodeHandle& nh_private);
  ~Radar();
  /**
   * @brief Least squares fit to estimate linear velocity.
   *
   * @param Radar measurement containing CFAR detections.
   * @param velocity pointer to velocity estimate.
   * @return true if velocity estimate successful.
   */
  static bool leastSquares(const mav_sensors::Radar& measurement,
                           Eigen::Vector3d* velocity);
};
}  // namespace mav_sensors_ros
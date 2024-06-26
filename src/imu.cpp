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
#include "mav_sensors_ros/imu.h"

#include <numeric>

#include <geometry_msgs/Vector3Stamped.h>
#include <log++.h>
#include <mav_sensors_core/sensor_config.h>
#include <sensor_msgs/Imu.h>

using namespace mav_sensors_ros;

Imu::Imu(const ros::NodeHandle& nh_private) : BaseSensor(nh_private) {}

Imu::~Imu() { imu_.close(); }

bool Imu::openSensor() {
  imu_pub_ = nh_private_.advertise<sensor_msgs::Imu>("data_raw", 1);
  bias_pub_ =
      nh_private_.advertise<geometry_msgs::Vector3Stamped>("gyro_offset", 1);

  calibrate_srv_ =
      nh_private_.advertiseService("calibrate", &Imu::calibrate, this);

  std::vector<double> gyro_bias;
  if (!nh_private_.getParam("gyro_bias", gyro_bias)) {
    LOG(W, "Failed to read gyro_bias.");
  } else if (gyro_bias.size() == 3) {
    b_g_ = {gyro_bias[0], gyro_bias[1], gyro_bias[2]};
    LOG(I, "Using gyro_bias: [" << gyro_bias[0] << ", " << gyro_bias[1] << ", "
                                << gyro_bias[2] << "] rad/s.");
  } else {
    LOG(W, "gyro_bias has wrong size.");
  }

  mav_sensors::SensorConfig cfg;

  std::string path_acc;
  if (!nh_private_.getParam("path_acc", path_acc)) {
    LOG(F, "Failed to read IMU path_acc.");
    return false;
  }
  cfg.set("path_acc", path_acc);

  std::string path_gyro;
  if (!nh_private_.getParam("path_gyro", path_gyro)) {
    LOG(F, "Failed to read IMU path_gyro.");
    return false;
  }
  cfg.set("path_gyro", path_gyro);

  std::string acc_range;
  if (nh_private_.getParam("acc_range", acc_range))
    cfg.set("acc_range", acc_range);

  std::string acc_bwp;
  if (nh_private_.getParam("acc_bwp", acc_bwp)) cfg.set("acc_bwp", acc_bwp);

  std::string acc_odr;
  if (nh_private_.getParam("acc_odr", acc_odr)) cfg.set("acc_odr", acc_odr);

  std::string gyro_range;
  if (nh_private_.getParam("gyro_range", gyro_range))
    cfg.set("gyro_range", gyro_range);

  std::string gyro_bw;
  if (nh_private_.getParam("gyro_bw", gyro_bw)) cfg.set("gyro_bw", gyro_bw);

  if (!nh_private_.getParam("bias_samples", bias_samples_)) {
    LOG(F, "Failed to read IMU bias_samples.");
    return false;
  }

  LOG(I, "Opening IMU on path_acc: " << path_acc.c_str()
                                     << " path_gyro: " << path_gyro.c_str());

  imu_.setConfig(cfg);
  if (!imu_.open()) {
    LOG(F, "Failed to open IMU.");
    return false;
  }

  imu_.printImuConfig();

  return true;
}

void Imu::readSensor() {
  // Read sensor data.
  auto measurement = imu_.read();

  if (std::get<0>(measurement).has_value() &&
      std::get<1>(measurement).has_value() &&
      std::get<2>(measurement).has_value()) {
    LOG_FIRST(I, 1, "Publishing first IMU measurement.");
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = toRosTime(std::get<2>(measurement).value());
    imu_msg.header.frame_id = frame_id_;

    imu_msg.angular_velocity.x = std::get<1>(measurement).value().x - b_g_.x;
    imu_msg.angular_velocity.y = std::get<1>(measurement).value().y - b_g_.y;
    imu_msg.angular_velocity.z = std::get<1>(measurement).value().z - b_g_.z;
    imu_msg.linear_acceleration.x = std::get<0>(measurement).value().x;
    imu_msg.linear_acceleration.y = std::get<0>(measurement).value().y;
    imu_msg.linear_acceleration.z = std::get<0>(measurement).value().z;
    imu_msg.orientation_covariance[0] =
        -1;  // orientation not supported by bmi088
    imu_pub_.publish(imu_msg);

    // Update bias.
    omega_.push_back(std::get<1>(measurement).value());
    if (omega_.size() > bias_samples_) omega_.pop_front();
    geometry_msgs::Vector3Stamped bias_msg;
    bias_msg.header = imu_msg.header;
    bias_msg.vector.x = b_g_.x;
    bias_msg.vector.y = b_g_.y;
    bias_msg.vector.z = b_g_.z;
    bias_pub_.publish(bias_msg);
  }
}

bool Imu::calibrate(std_srvs::Trigger::Request& req,
                    std_srvs::Trigger::Response& res) {
  if (omega_.size() < bias_samples_) {
    res.success = false;
    res.message = "Not enough samples.";
    return true;
  }

  b_g_ = std::accumulate(omega_.begin(), omega_.end(),
                         mav_sensors::vec3<double>{0.0, 0.0, 0.0}) /
         double(bias_samples_);

  res.success = true;
  res.message = "Calibrated with " + std::to_string(bias_samples_) +
                " samples. Gyro bias: " + std::to_string(b_g_.x) + " " +
                std::to_string(b_g_.y) + " " + std::to_string(b_g_.z) + ".";

  return true;
}
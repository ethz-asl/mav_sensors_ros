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
#include "mav_sensors_ros/base_sensor.h"

#include <log++.h>
#include <std_msgs/Header.h>

using namespace mav_sensors_ros;

BaseSensor::BaseSensor(const ros::NodeHandle& nh_private)
    : nh_private_(nh_private) {}

bool BaseSensor::init() {
  // Open sensor.
  if (!openSensor()) {
    LOG(F, "Failed to open sensor.");
    return false;
  }

  // Advertise timestamp.
  trigger_pub_ = nh_private_.advertise<std_msgs::Header>("trigger", 1);

  // Get sensor frame_id.
  if (!nh_private_.getParam("frame_id", frame_id_)) {
    LOG(F, "Failed to read frame_id.");
    return false;
  }

  // Initialize timer.
  double poll_rate;
  if (!nh_private_.getParam("poll_rate", poll_rate)) {
    LOG(F, "Failed to read rate.");
    return false;
  }
  timer_ = nh_private_.createTimer(ros::Duration(1.0 / poll_rate),
                                   &BaseSensor::timerCallback, this);

  return true;
}

void BaseSensor::timerCallback(const ros::TimerEvent& event) {
  // Publish timestamp.
  std_msgs::Header msg;
  msg.stamp = ros::Time::now();
  msg.frame_id = frame_id_;
  trigger_pub_.publish(msg);

  // Read sensor data.
  readSensor();
}
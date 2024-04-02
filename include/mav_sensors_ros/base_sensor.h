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

#include <memory>

#include <mav_sensors_core/sensor_config.h>
#include <ros/ros.h>

// This class interfaces mav_sensors to poll sensor data.
// It is configured at a fixed timer rate.
// Additionally, on each callback it publishes a timestamp before reading the
// sensor data. This can be used by the downstream sensor fusion to initialize a
// new state.
namespace mav_sensors_ros {
class BaseSensor {
 public:
  bool init();

  static inline ros::Time toRosTime(uint64_t unix_stamp_ns) {
    // Convert nanoseconds to seconds and remaining nanoseconds.
    uint64_t sec = unix_stamp_ns * 1e-9;
    uint64_t nsec = unix_stamp_ns - sec * 1e9;
    return ros::Time(sec, nsec);
  }

 protected:
  BaseSensor(const ros::NodeHandle& nh_private);
  ros::NodeHandle nh_private_;
  virtual void readSensor() = 0;
  virtual bool openSensor() = 0;
  mav_sensors::SensorConfig sensor_config_;

  std::string frame_id_;

 private:
  ros::Timer timer_;
  ros::Publisher trigger_pub_;

  // This function is called at a fixed rate.
  // It publishes a timestamp and reads the sensor data.
  void timerCallback(const ros::TimerEvent& event);
};
}  // namespace mav_sensors_ros
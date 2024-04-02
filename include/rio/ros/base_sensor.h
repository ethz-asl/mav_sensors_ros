#pragma once

#include <memory>

#include <mav_sensors_core/sensor_config.h>
#include <ros/ros.h>

// This class interfaces mav_sensors to poll sensor data.
// It is configured at a fixed timer rate.
// Additionally, on each callback it publishes a timestamp before reading the sensor data.
// This can be used by the downstream sensor fusion to initialize a new state.
namespace rio
{
  class BaseSensor
  {
  public:
    bool init();

    static inline ros::Time toRosTime(uint64_t unix_stamp_ns)
    {
      // Convert nanoseconds to seconds and remaining nanoseconds.
      uint64_t sec = unix_stamp_ns * 1e-9;
      uint64_t nsec = unix_stamp_ns - sec * 1e9;
      return ros::Time(sec, nsec);
    }

  protected:
    BaseSensor(const ros::NodeHandle &nh_private);
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
    void timerCallback(const ros::TimerEvent &event);
  };
} // namespace rio
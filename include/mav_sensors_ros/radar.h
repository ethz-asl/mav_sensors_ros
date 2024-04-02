#pragma once

#include <mav_sensors_drivers/radar/xwr18xx_mmw_demo.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <mav_sensors_drivers/sensor_types/Radar.h>
#include <Eigen/SVD>

#include "mav_sensors_ros/base_sensor.h"

namespace mav_sensors_ros
{
  class Radar : public BaseSensor
  {
  private:
    void readSensor() override;
    bool openSensor() override;

    mav_sensors::Xwr18XxMmwDemo radar_;
    ros::Publisher ls_vel_pub_;
    ros::Publisher cfar_pub_;

  public:
    Radar(const ros::NodeHandle &nh_private);
    ~Radar();
    /**
     * @brief Least squares fit to estimate linear velocity.
     *
     * @param Radar measurement containing CFAR detections.
     * @param velocity pointer to velocity estimate.
     * @return true if velocity estimate successful.
     */
    static bool leastSquares(const mav_sensors::Radar &measurement,
                             Eigen::Vector3d *velocity);
  };
} // namespace mav_sensors_ros
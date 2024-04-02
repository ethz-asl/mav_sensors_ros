#pragma once

#include <mav_sensors_drivers/radar/xwr18xx_mmw_demo.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <mav_sensors_drivers/sensor_types/Radar.h>
#include <Eigen/SVD>

#include "rio/ros/base_sensor.h"

namespace rio
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
    static inline bool leastSquares(const mav_sensors::Radar &measurement,
                                    Eigen::Vector3d *velocity)
    {
      if (measurement.cfar_detections.size() < 3)
        return false;
      // Get detection direction vector r and doppler velocity vd
      Eigen::MatrixXd r(measurement.cfar_detections.size(), 3);
      Eigen::VectorXd vd(measurement.cfar_detections.size());
      for (size_t i = 0; i < measurement.cfar_detections.size(); i++)
      {
        r.row(i) << measurement.cfar_detections[i].x,
            measurement.cfar_detections[i].y, measurement.cfar_detections[i].z;
        r.rowwise().normalize();
        vd(i) = measurement.cfar_detections[i].velocity;
      }
      // Compute least squares velocity estimate Ax = b, where A = r, b = vd
      auto jacobiSvd = r.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
      if (jacobiSvd.rank() < 3)
        return false;
      else
      {
        *velocity = jacobiSvd.solve(vd);
        return true;
      }
    }
  };
} // namespace rio
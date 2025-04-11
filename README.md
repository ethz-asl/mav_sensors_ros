# mav_sensors_ros
ROS 1 interface to [mav_sensors](https://github.com/ethz-asl/mav_sensors).

**Paper**: https://arxiv.org/pdf/2408.05764
```
@inproceedings{girod2024brio,
author = {Rik Girod and Marco Hauswirth and Patrick Pfreundschuh and Mariano Biasio and Roland Siegwart},
title = {A robust baro-radar-inertial odometry m-estimator for multicopter navigation in cities and forests},
booktitle={IEEE Int. Conf. Multisensor Fusion Integration Intell. Syst.},
year={2024}
}
```

# Related packages

| Package         | Description                       | Link                                                   |
| --------------- | --------------------------------- | ------------------------------------------------------ |
| rio             | radar-inertial odometry estimator | [rio](https://github.com/ethz-asl/rio)                 |
| mav_sensors     | Linux user space sensor drivers   | [mav_sensors](https://github.com/ethz-asl/mav_sensors) |


# TIAWR1843AOP Radar Coordinate Frame
The radar sensor uses by default a left-handed coordinate frame. A new parameter `flip_z` (boolean) is introduced, with which the frame can be made right-handed.

> **Attention:** the original implementation of RIO makes use of the original TIAWR1843AOP LH coordinate frame!

![TIAWR1843AOP Coordinate Frame](TICoordinateFrame.svg)

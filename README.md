# ROS2 Tello Driver

## About The Project
ROS2 driver to control Tello drones based on Tello SDK 2.0. Features include: status and video stream, action server for processing commands (takeoff, land, etc), connect to wifi access points, and swarm control.

## Getting Started
The driver has been tested on Ubuntu 22.04 with ROS2 Humble.

### Prerequisites
* ROS2 Humble
* Numpy
* h264decoder ([link to repo](https://github.com/DaWelter/h264decoder)

### Installation
To install the Tello driver:

```
mkdir tello_ws && cd tello_ws
mkdir src && cd src
git clone https://github.com/MRT-Codebase/ros2_tello_driver.git
colcon build
```

## Usage


## Contact
Kousheek Chakraborty - kousheekc@gmail.com

Project Link: [https://github.com/MRT-Codebase/ros2_tello_driver](https://github.com/MRT-Codebase/ros2_tello_driver)


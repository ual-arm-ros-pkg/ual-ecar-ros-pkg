[![Build Status](https://travis-ci.org/ual-arm-ros-pkg/ual-ecar-ros-pkg.svg?branch=master)](https://travis-ci.org/ual-arm-ros-pkg/ual-ecar-ros-pkg)

# ual-ecar-ros-pkg
ROS packages and config files for University of Almeria autonomous electric car (UAL eCar). Documentation for electrical connections, etc. are kept in a [separate repo](https://github.com/ual-arm/ual-ecar-docs).

See docs for each individual node:
  * [steer_controller](https://github.com/ual-arm-ros-pkg/ual-ecar-ros-pkg/tree/master/steer_controller): ROS node for UAL eCAR's steer driver ("caja azul").
  * [joystick_driving](https://github.com/ual-arm-ros-pkg/ual-ecar-ros-pkg/tree/master/joystick_driving): Manual joystick-based driving node.


## Installation

* [Install ROS](http://wiki.ros.org/indigo/Installation)
* Create a catkin workspace and clone this package:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
git clone https://github.com/ual-arm-ros-pkg/ual-ecar-ros-pkg.git
```

* Install dependencies:
```
sudo apt-get install -y ros-${ROS_DISTRO}-hardware-interface \
  ros-${ROS_DISTRO}-transmission-interface  ros-${ROS_DISTRO}-controller-manager \
  ros-${ROS_DISTRO}-diagnostic-updater xutils-dev
```

* Install [arduino_daq](https://github.com/ual-arm-ros-pkg/arduino-daq-ros-pkg).

* Build:
```
cd ~/catkin_ws
catkin_make
```

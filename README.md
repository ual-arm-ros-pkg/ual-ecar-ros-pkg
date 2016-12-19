[![Build Status](https://travis-ci.org/ual-arm-ros-pkg/ual-ecar-ros-pkg.svg?branch=master)](https://travis-ci.org/ual-arm-ros-pkg/ual-ecar-ros-pkg)

# ual-ecar-ros-pkg
ROS packages and config files for University of Almeria autonomous electric car (UAL eCar).

See docs for each individual node:
  * [steer_controller](https://github.com/ual-arm-ros-pkg/ual-ecar-ros-pkg/tree/master/steer_controller): ROS node for UAL eCAR's steer driver ("caja azul").
  * [arduino_daq](https://github.com/ual-arm-ros-pkg/ual-ecar-ros-pkg/tree/master/arduino_daq): A minimal DAQ system based on an Arduino.


## Installation

* [Install ROS](http://wiki.ros.org/indigo/Installation)
* Create a catkin workspace and clone this package and all submodules:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
git clone --recursive https://github.com/ual-arm-ros-pkg/ual-ecar-ros-pkg.git
```

* Install dependencies:
```
sudo apt-get install -y ros-${ROS_DISTRO}-hardware-interface \
  ros-${ROS_DISTRO}-transmission-interface  ros-${ROS_DISTRO}-controller-manager \
  ros-${ROS_DISTRO}-diagnostic-updater xutils-dev
```

* Build:
```
cd ~/catkin_ws
catkin_make
```

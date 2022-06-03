# md_drive_ros

## Introduction

The ROS 2 node for motor controller of wheeled mobile robots.
More details about the device could be found [here](https://mdrwiega.com/dc-motors-controller-for-mobile-platforms-with-ros-support/).

The node allows to communicate with the controller device.
It publishes the odometry data, digital inputs states and subscribes velocity commands, digital outputs states.

<img src="http://mdrwiega.com/wp_mdrwiega/wp-content/uploads/2016/05/controller1_400.png">
<br>
The MD Drive device.
<br>

<img src="http://mdrwiega.com/wp_mdrwiega/wp-content/uploads/2016/05/drive_controller_node-2.png">
<br>A structure of a ROS node.

## Requirements

- Install the `hidapi` library that is used to handle USB communication.
It could be done for example with APT on the Ubuntu system.
```bash
sudo apt install libhidapi-dev
```

- Install from sources the MD Drive API package [md_drive_api](https://github.com/mdrwiega/md_drive_api).

## Build (ROS 2 Galactic)

- Create ROS 2 workspace.
- Build: `colcon build`

## Run

- ros2 launch md_drive_ros controller_launch.py
- ros2 run md_drive_ros md_drive_ros_exe

## Features

## TODO

- Add full configuration of controller by parameters in yaml
- Add kinematics calculations
- Change watchdog to time period instead of ticks
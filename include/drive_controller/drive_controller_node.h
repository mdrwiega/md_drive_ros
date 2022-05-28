// Copyright (c) 2022 Michal Drwiega
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>

#include <md_drive_api/md_drive_api.h>

//#define DEBUG_NODE

namespace drive_controller {

/**
 *  ROS node that communicates with the MD Drive device.
 */
class DriveControllerNode
{
public:
  DriveControllerNode(ros::NodeHandle& n, ros::NodeHandle& pnh);
  ~DriveControllerNode();

  /**
    * @brief Update the drive controller node.
    * It should be called once per loop.
    *
    * In general, this function takes data from the device
    * send configurations and control data to the device and finally publish received data to topics.
    */
  void Update();

private:
  using MsgID = md_drive::MsgID;

  /**
    * @brief Callback which is called when new velocities command are received.
    *
    * @param cmd_vel Velocities commands
    */
  void CmdVelCallback(const geometry_msgs::Twist & cmd_vel);
  /**
    * @brief Callback which is called when new digital outputs commands are received.
    * @param do_states
    */
  void DigitalOutputsCallback(const std_msgs::UInt8 do_states);

  void StopMotors();

  md_drive::MdDriveAPI mc_api_;

  ros::NodeHandle pnh_;                ///< Private node handler

  // Publishers
  ros::Publisher  pub_status_;         ///< Drive controller status publisher
  ros::Publisher  pub_odom_;           ///< Odometry data publisher
  ros::Publisher  pub_mot_voltage_;    ///< Motors voltage publisher
  ros::Publisher  pub_digital_inputs_; ///< Digital inputs states publisher
  tf2_ros::TransformBroadcaster tf_broadcaster_; ///< Broadcaster for tf publishing

  // Subscribers
  ros::Subscriber sub_cmd_vel_;         ///< Velocity commands subscriber
  ros::Subscriber sub_digital_outputs_; ///< Subscriber for digital outputs ports

  float back_wheels_separation_;                 ///< Back robot wheels separation in meters
  float front_wheels_separation_;                ///< Front robot wheels separation in meters
  std::string base_frame_id_{"base_link"}; ///< Frame for mobile robot base
  std::string odom_frame_id_{"odom"};      ///< Frame for odometry

  float cmd_vel_timeout_; ///< Velocity commands timeout in ms

  // Velocities and acceleration limits
  float linear_velocity_min_;      ///< Minimum linear velocity in m/s
  float linear_velocity_max_;      ///< Maximum linear velocity in m/s
  float angular_velocity_min_;     ///< Minimum angular velocity in rad/s
  float angular_velocity_max_;     ///< Maximum angular velocity in rad/s
  float linear_acceleration_min_;  ///< Minimum linear acceleration in m/s^2
  float linear_acceleration_max_;  ///< Maximum linear acceleration in m/s^2
  float angular_acceleration_min_; ///< Minimum angular acceleration in rad/s^2
  float angular_acceleration_max_; ///< Maximum angular acceleration in rad/s^2

  // Simple watchdog which send zero speeds to microcontroller if no new cmd_vel
  unsigned int watchdog_cnt_{0};
};

} // namespace drive_controller

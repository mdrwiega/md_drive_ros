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

#ifndef DRIVE_CONTROLLER_DRIVE_CONTROLLER_NODE_HPP_
#define DRIVE_CONTROLLER_DRIVE_CONTROLLER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <md_drive_api/md_drive_api.h>
#include <drive_controller/odometry.h>

namespace drive_controller {

/**
 *  ROS node that communicates with the MD Drive device.
 */
class DriveControllerNode : public rclcpp::Node
{
public:
  DriveControllerNode();
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
  void CmdVelCallback(const geometry_msgs::msg::Twist & cmd_vel);
  /**
    * @brief Callback which is called when new digital outputs commands are received.
    * @param do_states Digital outputs control values.
    */
  void DigitalOutputsCallback(const std_msgs::msg::UInt8 do_states);

  void StopMotors();

  void PublishOdometry();

  md_drive::MdDriveAPI mc_api_;

  // Publishers
  /// Drive controller status publisher
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_status_;
   ///< Odometry data publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  ///< Motors voltage publisher
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_mot_voltage_;
  ///< Digital inputs states publisher
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_digital_inputs_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Subscribers
  /// Velocity commands subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  /// Subscriber for digital outputs ports
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_digital_outputs_;

  std::shared_ptr<Odometry> odometry; ///< Calculates odometry

  // Node parameters
  std::string base_frame_id_;
  std::string odom_frame_id_;
  float back_wheels_separation_;
  float front_wheels_separation_;
  float wheel_radius_;

  // Simple watchdog which send zero speeds to microcontroller if no new cmd_vel
  unsigned int watchdog_cnt_ = 0;
};

} // namespace drive_controller

#endif // DRIVE_CONTROLLER_DRIVE_CONTROLLER_NODE_HPP_
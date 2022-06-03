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

#include <drive_controller/drive_controller_node.hpp>

#include <exception>

#include <std_msgs/msg/u_int16.h>
#include <std_msgs/msg/float64.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <md_drive_api/md_drive_api.h>
#include <md_drive_api/board.h>

namespace drive_controller {

using namespace std::chrono_literals;

DriveControllerNode::DriveControllerNode()
: Node("drive_controller")
{
  using std::placeholders::_1;

  // Subscribers
  sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 1, std::bind(&DriveControllerNode::CmdVelCallback, this, _1));

  sub_digital_outputs_ = this->create_subscription<std_msgs::msg::UInt8>(
    "digital_outputs", 1, std::bind(&DriveControllerNode::DigitalOutputsCallback, this, _1));

  // Odometry data publishing
  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);

  // Digital inputs states publisher
  pub_digital_inputs_ = this->create_publisher<std_msgs::msg::UInt8>("digital_inputs", 1);

  // Status publisher
  pub_status_ = this->create_publisher<std_msgs::msg::UInt16>("drive_status", 1);

  // Motors voltage publisher
  pub_mot_voltage_ = this->create_publisher<std_msgs::msg::Float64>("voltage", 1);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  if (mc_api_.ConnectToDevice()) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Connected to drive controller device.");
  }
  else {
    RCLCPP_ERROR_ONCE(this->get_logger(), "HW device connection can't be established.");
    throw std::runtime_error("Unable to connect to the device.");
  }

  // Declare parameters
  this->declare_parameter<std::string>("base_frame_id", "base_link");
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<float>("back_wheels_distance", 1.0f);
  this->declare_parameter<float>("front_wheels_distance", 1.0f);
  this->declare_parameter<float>("wheel_radius", 1.0f);

  // Get parameters
  this->get_parameter("base_frame_id", base_frame_id_);
  this->get_parameter("odom_frame_id", odom_frame_id_);
  this->get_parameter("back_wheels_distance", back_wheels_separation_);
  this->get_parameter("front_wheels_distance", front_wheels_separation_);
  this->get_parameter("wheel_radius", wheel_radius_);

  // TODO: Configure motors controller
  md_drive::GeneralConfig general_config;
  general_config.controllerMode = md_drive::ControllerMode::Speed;
  for (auto & motor : general_config.enabledMotors)
  {
    motor = true;
  }

  RCLCPP_INFO_STREAM(this->get_logger(),general_config.ToString() << "\n");
  mc_api_.SendRawMsgToDevice(SerializeMsg(general_config));

  // Initialize odometry
  odometry.reset(
    new Odometry(wheel_radius_, (back_wheels_separation_ + front_wheels_separation_) / 2.0));
}

DriveControllerNode::~DriveControllerNode()
{
  mc_api_.DisconnectFromDevice();
}

void DriveControllerNode::Update()
{
  // Get and publish status
  auto serialized_msg = mc_api_.GetRawMsgFromDevice(MsgID::GeneralState);
  const auto state_msg = md_drive::DeserializeMsg<md_drive::GeneralState>(serialized_msg);
  std_msgs::msg::UInt16 status;
  status.data = static_cast<uint16_t>(state_msg.state_reg0);
  pub_status_->publish(status);

  std_msgs::msg::Float64 voltage;
  voltage.data = state_msg.motors_voltage;
  pub_mot_voltage_->publish(voltage);

  // Update and publish odometry
  using md_drive::Motor;
  const auto left_pos = (state_msg.distance[0] + state_msg.distance[2]) / 2.0;
  const auto right_pos = (state_msg.distance[1] + state_msg.distance[3]) / 2.0;
  const auto left_speed = (state_msg.velocity[0] + state_msg.velocity[2]) / 2.0;
  const auto right_speed = (state_msg.velocity[1] + state_msg.velocity[3]) / 2.0;
  odometry->Update(left_pos, right_pos, left_speed, right_speed);
  PublishOdometry();

  // Check if watchdog maximum value exceeded
  if (++watchdog_cnt_ > 50) {
    StopMotors();
  }
}

void DriveControllerNode::CmdVelCallback(const geometry_msgs::msg::Twist & cmd_vel)
{
    // Inverse kinematics for differential drive
    const double v = cmd_vel.linear.x;
    const double w = cmd_vel.angular.z;
    const double d = (back_wheels_separation_ + front_wheels_separation_) / 2.0;

    const double left_wheel_speed = (v - w * d / 2) / wheel_radius_;
    const double right_wheel_speed = (v + w * d / 2) / wheel_radius_;

    md_drive::SpeedControl ctrl;
    ctrl.setpoints[static_cast<unsigned>(md_drive::Motor::BL)] = left_wheel_speed;
    ctrl.setpoints[static_cast<unsigned>(md_drive::Motor::BR)] = right_wheel_speed;
    ctrl.setpoints[static_cast<unsigned>(md_drive::Motor::FL)] = left_wheel_speed;
    ctrl.setpoints[static_cast<unsigned>(md_drive::Motor::FR)] = right_wheel_speed;
    mc_api_.SendRawMsgToDevice(SerializeMsg(ctrl));

    // Reset watchdog counter
    watchdog_cnt_ = 0;
}

void DriveControllerNode::DigitalOutputsCallback(const std_msgs::msg::UInt8 do_states)
{
  md_drive::DigitalOutputsControl ctrl;
  for (int i = 0; i < DIGITAL_OUTPUTS_NR; ++i) {
    ctrl.ctrl_DO[i] = do_states.data & (1 << i);
    ctrl.mask_DO[i] = true;
  }
  mc_api_.SendRawMsgToDevice(SerializeMsg(ctrl));
}

void DriveControllerNode::StopMotors()
{
  md_drive::SpeedControl ctrl;
  for (auto & setpoint : ctrl.setpoints) {
    setpoint = 0.0;
  }

  mc_api_.SendRawMsgToDevice(SerializeMsg(ctrl));
}

void DriveControllerNode::PublishOdometry()
{
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->get_clock()->now();
    odom.header.frame_id = odom_frame_id_;
    // Set the odom position and orientation

    odom.pose.pose.position.x = odometry->x;
    odom.pose.pose.position.y = odometry->y;
    odom.pose.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, odometry->theta);
    odom.pose.pose.orientation = tf2::toMsg(q);

    // Set the velocities
    odom.child_frame_id = base_frame_id_;
    odom.twist.twist.linear.x = odometry->lin_vel;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = odometry->ang_vel;

    odom.pose.covariance[0] = 0.00001;
    odom.pose.covariance[7] = 0.00001;
    odom.pose.covariance[14] = 1000000000000.0;
    odom.pose.covariance[21] = 1000000000000.0;
    odom.pose.covariance[28] = 1000000000000.0;
    odom.pose.covariance[35] = 0.001;

    pub_odom_->publish(odom);

    // Publish odometry transformations (tf)
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = odom.header.stamp;
    transform.header.frame_id = odom_frame_id_;
    transform.child_frame_id = base_frame_id_;
    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.translation.z = odom.pose.pose.position.z;
    transform.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
}

} // namespace drive_controller
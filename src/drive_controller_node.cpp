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

#include <drive_controller/drive_controller_node.h>

#include <exception>

#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>

#include <md_drive_api/md_drive_api.h>
#include <md_drive_api/board.h>

namespace drive_controller {

DriveControllerNode::DriveControllerNode(ros::NodeHandle& n, ros::NodeHandle& pnh)
  : pnh_(pnh)
{
  // Velocity commands subscription (geometry_msgs/Twist)
  sub_cmd_vel_ = n.subscribe("cmd_vel", 1, &DriveControllerNode::CmdVelCallback, this);

  sub_digital_outputs_ = n.subscribe(
    "digital_outputs", 1, &DriveControllerNode::DigitalOutputsCallback, this);

  // Odometry data publishing (sensor_msgs/Odom)
  pub_odom_ = n.advertise<nav_msgs::Odometry>("odom", 1);

  // Digital inputs states publisher
  pub_digital_inputs_ = n.advertise<std_msgs::UInt8>("digital_inputs", 1);

  // Status publisher
  pub_status_ = n.advertise<std_msgs::UInt16 >("drive_status", 1);

  // Motors voltage publisher
  pub_mot_voltage_ = n.advertise<std_msgs::Float64>("drive_controller/motors_voltage",1);

  if (mc_api_.ConnectToDevice()) {
    ROS_INFO("Connected to drive controller device.");
  }
  else {
    ROS_ERROR("HW device connection can't be established.");
    throw std::runtime_error("Unable to connect to the device.");
  }

  // Get parameters
  n.param<std::string>("base_frame_id", base_frame_id_, "base_link");
  n.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
  n.param<float>("back_wheels_distance", back_wheels_separation_, 1.0f);
  n.param<float>("front_wheels_distance", front_wheels_separation_, 1.0f);
  n.param<float>("wheel_radius", wheel_radius_, 1.0f);

  // TODO: Configure motors controller
  md_drive::GeneralConfig general_config;
  general_config.controllerMode = md_drive::ControllerMode::Speed;
  for (auto & motor : general_config.enabledMotors)
  {
    motor = true;
  }

  ROS_INFO_STREAM(general_config.ToString() << "\n");
  mc_api_.SendRawMsgToDevice(SerializeMsg(general_config));
}

DriveControllerNode::~DriveControllerNode()
{
  mc_api_.DisconnectFromDevice();

  sub_cmd_vel_.shutdown();
  sub_digital_outputs_.shutdown();
  pub_status_.shutdown();
  pub_mot_voltage_.shutdown();
  pub_odom_.shutdown();
  pub_digital_inputs_.shutdown();
}

void DriveControllerNode::Update()
{
  // Get and publish status
  auto serialized_msg = mc_api_.GetRawMsgFromDevice(MsgID::GeneralState);
  const auto state_msg = md_drive::DeserializeMsg<md_drive::GeneralState>(serialized_msg);
  std_msgs::UInt16 status;
  status.data = static_cast<uint16_t>(state_msg.state_reg0);
  pub_status_.publish(status);

  std_msgs::Float64 voltage;
  voltage.data = state_msg.motors_voltage;
  pub_mot_voltage_.publish(voltage);

  // Check if watchdog maximum value exceeded
  if (++watchdog_cnt_ > 50) {
    StopMotors();
  }
}

void DriveControllerNode::CmdVelCallback(const geometry_msgs::Twist & cmd_vel)
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

void DriveControllerNode::DigitalOutputsCallback(const std_msgs::UInt8 do_states)
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

// void DriveControllerNode::publishOdometry(...)
// {
    // Publish odometry data to topic
    // nav_msgs::Odometry odom;
    // odom.header.stamp = ros::Time::now();
    // odom.header.frame_id = odom_frame_id_;
    // Set the odom position and orientation

    // odom.pose.pose.position.x = mc_data_.x_;
    // odom.pose.pose.position.y = mc_data_.y_;
    // odom.pose.pose.position.z = 0;

    // geometry_msgs::Quaternion orient = tf::createQuaternionMsgFromYaw(mc_data_.theta_);
    // odom.pose.pose.orientation = orient;

    // // Set the velocities
    // odom.child_frame_id = base_frame_id_;
    // odom.twist.twist.linear.x = mc_data_.lin_vel_;
    // odom.twist.twist.linear.y = 0;
    // odom.twist.twist.linear.z = 0;
    // odom.twist.twist.angular.x = 0;
    // odom.twist.twist.angular.y = 0;
    // odom.twist.twist.angular.z = mc_data_.ang_vel_;

    // odom.pose.covariance[0] = 0.00001;
    // odom.pose.covariance[7] = 0.00001;
    // odom.pose.covariance[14] = 1000000000000.0;
    // odom.pose.covariance[21] = 1000000000000.0;
    // odom.pose.covariance[28] = 1000000000000.0;
    // odom.pose.covariance[35] = 0.001;

    // pub_odom_.publish(odom);

    // Publish odometry transformations (tf)
    // geometry_msgs::TransformStamped transform;
    // transform.header.stamp = odom.header.stamp;
    // transform.header.frame_id = odom_frame_id_;
    // transform.child_frame_id = base_frame_id_;
    // transform.transform.translation.x = odom.pose.pose.position.x;
    // transform.transform.translation.y = odom.pose.pose.position.y;
    // transform.transform.translation.z = odom.pose.pose.position.z;
    // transform.transform.rotation = orient;
    // tf_broadcaster_.sendTransform(transform);
// }

} // namespace drive_controller
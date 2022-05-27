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

#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>

#include <iostream>

namespace drive_controller {

//==============================================================================
DriveControllerNode::DriveControllerNode(ros::NodeHandle& n, ros::NodeHandle& pnh):
    pnh_(pnh), dyn_srv_(pnh)
{
    std::lock_guard<std::mutex> lock(connection_mutex_);

    // Set callback for dynamic reconfigure server
    dyn_srv_.setCallback(boost::bind(&DriveControllerNode::reconfigureCb, this, _1, _2));

    // Velocity commands subscription (geometry_msgs/Twist)
    sub_cmd_vel_ = n.subscribe("cmd_vel", 10,
                               &DriveControllerNode::velocitiesCb, this);

    // Velocity commands subscription (std_msgs/UInt8)
    /*sub_cmd_vel_ = n.subscribe("drive_controller/digital_outputs", 10,
                             &DriveControllerNode::digitalOutputsCb, this);*/

    // Odometry data publishing (sensor_msgs/Odom)
    pub_odom_ = n.advertise<nav_msgs::Odometry>("odom", 1);

    // Digital inputs states publisher
    pub_digital_inputs_ = n.advertise<std_msgs::UInt8>("digital_inputs", 1);

    // Status publisher
    pub_status_ = n.advertise<std_msgs::UInt16 >("drive_status", 1);

    // Motors voltage publisher
    pub_mot_voltage_ = n.advertise<std_msgs::Float64>("drive_controller/motors_voltage",1);

    mc_ctrl_.ang_z_vel = 0.0;
    mc_ctrl_.lin_x_vel = 0.0;

    // Connect to HW device
    if (mc_api_.connectToDevice(DEVICE_VID, DEVICE_PID))
    {
        ROS_INFO("Connected to drive controller device.");
    }
    else
    {
        ROS_ERROR("HW device connection can't be established.");
    }

    // Get constant ros params values from server params
    //  n.param<float>("back_wheels_separation", back_wheels_separation_, 1.0);
    //  n.param<float>("front_wheels_separation", front_wheels_separation_, 1.0);
    base_frame_id_ = "base_link";
    odom_frame_id_ = "odom";
    //  n.param<std::string>("base_frame_id", base_frame_id_, "base_link");
    //  n.param<std::string>("base_frame_id", odom_frame_id_, "odom");
}

//==============================================================================
DriveControllerNode::~DriveControllerNode()
{
    if (mc_api_.isConnected())
        mc_api_.disconnectDevice();

    sub_cmd_vel_.shutdown();
    sub_digital_outputs_.shutdown();
    pub_status_.shutdown();
    pub_mot_voltage_.shutdown();
    pub_odom_.shutdown();
    pub_digital_inputs_.shutdown();
}

//==============================================================================
void DriveControllerNode::writeDataToDevice()
{
    // Check if watchdog maximum value exceeded
    if (++watchdog_cnt_ > 50)
    {
        mc_ctrl_.setReferenceSpeed(BL, 0);
        mc_ctrl_.setReferenceSpeed(BR, 0);
        mc_ctrl_.setReferenceSpeed(FL, 0);
        mc_ctrl_.setReferenceSpeed(FR, 0);

        mc_ctrl_.lin_x_vel = 0;
        mc_ctrl_.ang_z_vel = 0;
    }

    mc_api_.sendCtrlDataToDevice(this->mc_ctrl_);

    std::cout << "\nSend data: lin_vel = " << mc_ctrl_.lin_x_vel << "  ang_vel = "
              << mc_ctrl_.ang_z_vel << "  ";
}

//==============================================================================
void DriveControllerNode::readDataFromDevice()
{
    mc_api_.getProcessDataFromDevice(mc_data_);

    std::cout << "\nRead data: x = " << mc_data_.x_ << "  y = " << mc_data_.y_
              << "  theta = " << mc_data_.theta_ << "  lin_vel = " << mc_data_.lin_vel_
              << "  ang_vel = " << mc_data_.ang_vel_;
}

//==============================================================================
void DriveControllerNode::publishDataToTopics()
{
    // Publish status
    std_msgs::UInt16 status;
    status.data = (uint16_t) mc_data_.state_reg0;
    pub_status_.publish(status);

    // Publish motors voltage
    std_msgs::Float64 voltage;
    voltage.data = mc_data_.getMotorsVoltage_mV() / 1000.0;
    pub_mot_voltage_.publish(voltage);

    // Publish odometry data to topic
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = odom_frame_id_;
    // Set the odom position and orientation

    odom.pose.pose.position.x = mc_data_.x_;
    odom.pose.pose.position.y = mc_data_.y_;
    odom.pose.pose.position.z = 0;

    geometry_msgs::Quaternion orient = tf::createQuaternionMsgFromYaw(mc_data_.theta_);
    odom.pose.pose.orientation = orient;

    // Set the velocities
    odom.child_frame_id = base_frame_id_;
    odom.twist.twist.linear.x = mc_data_.lin_vel_;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = mc_data_.ang_vel_;

    odom.pose.covariance[0] = 0.00001;
    odom.pose.covariance[7] = 0.00001;
    odom.pose.covariance[14] = 1000000000000.0;
    odom.pose.covariance[21] = 1000000000000.0;
    odom.pose.covariance[28] = 1000000000000.0;
    odom.pose.covariance[35] = 0.001;

    pub_odom_.publish(odom);

    // Publish odometry transformations (tf)
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = odom.header.stamp;
    transform.header.frame_id = odom_frame_id_;
    transform.child_frame_id = base_frame_id_;
    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.translation.z = odom.pose.pose.position.z;
    transform.transform.rotation = orient;
    tf_broadcaster_.sendTransform(transform);
}

//==============================================================================
void DriveControllerNode::velocitiesCb(const geometry_msgs::Twist cmd_vel)
{
    // Calculate references wheels speeds based on inverse kinematics
    std::vector<float> ref_speeds(4);

    mc_ctrl_.lin_x_vel = cmd_vel.linear.x;
    mc_ctrl_.ang_z_vel = cmd_vel.angular.z;

    // Set ref speeds in control structure
    /*mc_ctrl_.setReferenceSpeed(BL, ref_speeds[BL]);
  mc_ctrl_.setReferenceSpeed(BR, ref_speeds[BR]);
  mc_ctrl_.setReferenceSpeed(FL, ref_speeds[FL]);
  mc_ctrl_.setReferenceSpeed(FR, ref_speeds[FR]);*/

    // Reset watchdog counter
    watchdog_cnt_ = 0;
}

//==============================================================================
void digitalOutputsCb(const std_msgs::UInt8 do_states)
{
}

//==============================================================================
void DriveControllerNode::reconfigureCb(
        drive_controller::DriveControllerConfig& config, uint32_t level)
{
}

}

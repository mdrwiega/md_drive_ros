/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the copyright holder nor the names of its
 *        contributors may be used to endorse or promote products derived
 *        from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
/**
 * @file   drive_controller_node.h
 * @author Michal Drwiega (drwiega.michal@gmail.com)
 * @date   2016
 * @brief  drive_controller package
 */

#pragma once

#include<mutex>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>

#include <drive_controller/DriveControllerConfig.h>
#include <drive_controller/motor_controller_api.h>

//#define DEBUG_NODE

namespace drive_controller {

class DriveControllerNode
{
public:
    DriveControllerNode(ros::NodeHandle& n, ros::NodeHandle& pnh);
    ~DriveControllerNode();

    /// writeDataToDevice
    void writeDataToDevice();
    /// readDataFromDevice
    void readDataFromDevice();
    /// publishDataToTopics
    void publishDataToTopics();
private:
    /**
   * @brief velocitiesCb is callback which is called when new velocities command received
   *
   * @param cmd_vel Velocities commands
   */
    void velocitiesCb(const geometry_msgs::Twist cmd_vel);
    /**
   * @brief digitalOutputsCb
   * @param do_states
   */
    void digitalOutputsCb(const std_msgs::UInt8 do_states);
    /**
   * @brief reconfigureCb is dynamic reconfigure callback
   *
   * Callback is necessary to set ROS parameters with dynamic reconfigure server.
   *
   * @param config Dynamic Reconfigure object.
   * @param level Dynamic Reconfigure level.
   */
    void reconfigureCb(drive_controller::DriveControllerConfig& config, uint32_t level);

private: // Private fields

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

    /// Dynamic reconfigure server
    dynamic_reconfigure::Server<drive_controller::DriveControllerConfig> dyn_srv_;

    MotorControllerAPI     mc_api_;    ///< HW controller API object
    MotorControllerConfig  mc_config_; ///< HW controller configuration
    MotorControllerDataSet mc_data_;   ///< Data received from HW module
    ControlDataSet         mc_ctrl_;   ///< HW controller control data

    //-----------------------------------------------------------------------------------------------
    // ROS parameters setted when node starts
    //-----------------------------------------------------------------------------------------------
    float back_wheels_separation_;                 ///< Back robot wheels separation in meters
    float front_wheels_separation_;                ///< Front robot wheels separation in meters
    std::string base_frame_id_{"wheel_base_link"}; ///< Frame for mobile robot base
    std::string odom_frame_id_{"wheel_odom"};      ///< Frame for odometry

    //-----------------------------------------------------------------------------------------------
    // ROS parameters changed by dynamic reconfigure
    //-----------------------------------------------------------------------------------------------
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
    //-----------------------------------------------------------------------------------------------

    // Simple watchdog which send zero speeds to microcontroller if no new cmd_vel
    unsigned int watchdog_cnt_{0};
    // time_out need to be calculated as max cnt value

    ///< Prevents the connectCb and disconnectCb from being called until everything is initialized.
    std::mutex connection_mutex_;
};
}

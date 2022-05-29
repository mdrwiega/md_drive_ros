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

namespace drive_controller {

class Odometry
{
public:

  /*
  * wheel_radius [m]
  * base [m]
  */
  Odometry(double wheel_radius, double base);

  void Update(double left_pos, double right_pos, double left_speed, double right_speed);

  void Reset();

  double x = 0.0;        ///< Position in x axis [m]
  double y = 0.0;        ///< Position in y axis [m]
  double theta = 0.0;    ///< Heading [rad]

  double lin_vel = 0.0;  ///< Linear velocity [m/s]
  double ang_vel = 0.0;  ///< Angular velocity [rad/s]

private:
  void IntegrateRungeKutta2(double linear, double angular);

  // Previous wheel position/state [rad]:
  double left_pos_old_ = 0;
  double right_pos_old_ = 0;

  // Wheel kinematic parameters [m]:
  double base_ = 0.0;
  double wheel_radius_ = 0.0;
};

} // namespace drive_controller

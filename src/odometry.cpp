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

#include <drive_controller/odometry.h>

#include <cmath>

namespace drive_controller {

Odometry::Odometry(double wheel_radius, double base)
{
  wheel_radius_ = wheel_radius;
  base_ = base;
}

void Odometry::Reset()
{
  x = 0;
  y = 0;
  theta = 0;

  lin_vel = 0;
  ang_vel = 0;

  left_pos_old_ = 0;
  right_pos_old_ = 0;
}

void Odometry::IntegrateRungeKutta2(double linear, double angular)
{
  double direction = theta + angular * 0.5;

  // Runge-Kutta 2nd order integration:
  x     += linear * std::cos(direction);
  y     += linear * std::sin(direction);
  theta += angular;
}

void Odometry::Update(
  double left_pos, double right_pos, double left_speed, double right_speed)
{
  // Change of left and right wheel position (differential-drive robot)
  double left_diff  = left_pos - left_pos_old_;
  double right_diff  = right_pos - right_pos_old_;

  // Save old position values
  left_pos_old_  = left_pos;
  right_pos_old_ = right_pos;

  // Calculate linear and angular differences
  double lin_diff  = (left_diff + right_diff) * wheel_radius_ * 0.5 ;
  double ang_diff = (right_diff - left_diff) * wheel_radius_ / base_;

  // Integrate odometry with Runge-Kutta method
  IntegrateRungeKutta2(lin_diff, ang_diff);

  lin_vel = (left_speed  + right_speed) * wheel_radius_ * 0.5;
  ang_vel = (right_speed  - left_speed) * wheel_radius_ / base_;
}

} // namespace drive_controller

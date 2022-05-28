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

#include <ros/ros.h>
#include <drive_controller/drive_controller_node.h>

constexpr int loopFrequency = 100;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive_controller");

  ros::NodeHandle n;
  ros::NodeHandle pnh("~");
  drive_controller::DriveControllerNode controller(n, pnh);

  ros::Rate rate(loopFrequency);

  while (ros::ok())
  {
    ros::spinOnce();
    controller.update();
    rate.sleep();
  }

  return 0;
}

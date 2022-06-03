# Copyright 2016-2021 Michał Drwięga (drwiega.michal@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = launch.LaunchDescription()

    config = os.path.join(
        get_package_share_directory('md_drive_ros'),
        'config',
        'params.yaml'
    )

    md_drive_ros = Node(
        package='md_drive_ros',
        node_executable='md_drive_ros_exe',
        parameters=[config]
    )
    ld.add_action(md_drive_ros)

    return ld

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='md_drive_ros',
            namespace='md_drive_ros',
            executable='md_drive_ros_exe',
            name='md_drive_ros'
        )
    ])

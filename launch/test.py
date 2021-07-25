# Copyright (c) 2021 OUXT Polaris
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(get_package_share_directory(
        'ouxt_behavior_descriptor_v1'), 'test', 'example/test.yaml')

    start_descriptor_cmd = Node(
        package='ouxt_behavior_descriptor_v1', executable='ouxt_behavior_descriptor_v1_node',
        output='screen',
        parameters=[{'config_path': config_path}]
    )

    ld = LaunchDescription()

    ld.add_action(start_descriptor_cmd)

    return ld

#!/usr/bin/env python3
#
# Copyright 2025 Reazon Holdings, Inc.
# Copyright 2024 Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#
# This file is based on the templates in the
# [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace)
# repository.
#
# Author: Dr. Denis
#

from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='openarm_bilateral',
            executable='bilateral_openarm_main',
            output='screen',
            arguments=['right_arm', 'unilate'],
            parameters=[]
        )
    ])




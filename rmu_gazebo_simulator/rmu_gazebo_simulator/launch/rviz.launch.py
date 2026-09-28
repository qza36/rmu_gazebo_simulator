# Copyright 2025 Lihan Chen
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    declare_rviz_config_file = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(
            get_package_share_directory("rmu_gazebo_simulator"),
            "rviz",
            "visualize.rviz",
        ),
        description="Path to the RViz2 configuration file",
    )

    # 不加命名空间：话题/坐标系都是全局的（/livox/lidar、/rplidar_a2/scan、odom、baselink ...）
    start_rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(declare_rviz_config_file)
    ld.add_action(start_rviz2)

    return ld

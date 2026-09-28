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

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")
    pkg_pb2025_robot_description = get_package_share_directory(
        "pb2025_robot_description"
    )

    robot_xmacro = LaunchConfiguration("robot_xmacro")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    declare_robot_xmacro = DeclareLaunchArgument(
        "robot_xmacro",
        default_value=os.path.join(
            pkg_pb2025_robot_description,
            "resource",
            "xmacro",
            "simple_chassis_robot.sdf.xmacro",
        ),
        description=(
            "Path to the robot SDF xmacro file. Default: simple_chassis_robot "
            "(580x580mm square chassis, mid360 at 210mm). Use "
            "simulation_robot.sdf.xmacro for the full RM robot."
        ),
    )

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Whether to start RViz2 together with the simulation",
    )

    declare_rviz_config_file = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(pkg_simulator, "rviz", "visualize.rviz"),
        description="Path to the RViz2 configuration file",
    )

    gz_world_path = os.path.join(pkg_simulator, "config", "gz_world.yaml")
    with open(gz_world_path) as file:
        config = yaml.safe_load(file)
        selected_world = config.get("world")

    world_sdf_path = os.path.join(
        pkg_simulator, "resource", "worlds", f"{selected_world}_world.sdf"
    )
    gz_config_path = os.path.join(pkg_simulator, "resource", "ign", "gui.config")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world_sdf_path": world_sdf_path,
            "gz_config_path": gz_config_path,
        }.items(),
    )

    spawn_robots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "spawn_robots.launch.py")
        ),
        launch_arguments={
            "gz_world_path": gz_world_path,
            "world": selected_world,
            "robot_xmacro": robot_xmacro,
        }.items(),
    )

    referee_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "referee_system.launch.py")
        )
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "rviz.launch.py")
        ),
        launch_arguments={
            "rviz_config_file": rviz_config_file,
        }.items(),
        condition=IfCondition(use_rviz),
    )

    ld = LaunchDescription()

    ld.add_action(declare_robot_xmacro)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_rviz_config_file)
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_robots_launch)
    ld.add_action(referee_system_launch)
    ld.add_action(rviz_launch)

    return ld

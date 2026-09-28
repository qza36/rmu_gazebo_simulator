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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString
from sdformat_tools.urdf_generator import UrdfGenerator
from xmacro.xmacro4sdf import XMLMacro4sdf


def launch_setup(context, *args, **kwargs):
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")
    pkg_pb2025_robot_description = get_package_share_directory(
        "pb2025_robot_description"
    )

    robot_xmacro_path = LaunchConfiguration("robot_xmacro").perform(context)
    bridge_config = os.path.join(pkg_simulator, "config", "ros_gz_bridge.yaml")
    robot_config = os.path.join(pkg_simulator, "config", "base_params.yaml")

    gz_world_path = os.path.join(pkg_simulator, "config", "gz_world.yaml")
    with open(gz_world_path) as file:
        config = yaml.safe_load(file)
        selected_world = config.get("world")
        robots = config["robots"].get(selected_world)

    xmacro = XMLMacro4sdf()
    xmacro.set_xml_file(robot_xmacro_path)

    ld = []

    for robot in robots:
        xmacro.generate({"global_initial_color": robot["color"]})
        # 模型里的 @ROBOT_NAME@（速度控制插件的 gz 话题名）替换成实际机器人名
        robot_xml = xmacro.to_string().replace("@ROBOT_NAME@", robot["name"])

        urdf_generator = UrdfGenerator()
        urdf_generator.parse_from_sdf_string(robot_xml)
        robot_urdf_xml = urdf_generator.to_string()

        aft_replace_ros_bridge_params = ReplaceString(
            source_file=bridge_config,
            replacements={"<robot_name>": robot["name"]},
        )

        spawn_robot = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-string",
                robot_xml,
                "-name",
                robot["name"],
                "-allow_renaming",
                "true",
                "-x",
                robot["x_pose"],
                "-y",
                robot["y_pose"],
                "-z",
                robot["z_pose"],
                "-Y",
                robot["yaw"],
            ],
        )

        # 说明：以下节点都不加命名空间（单机器人调试用），
        # 话题是 /cmd_vel、/chassis_odometry_gt、/livox/...、/rplidar_a2/... 这样不带机器人名前缀的形式。
        # 机器人名只保留在参数里，用于拼接 gazebo 侧的话题。
        robot_base = Node(
            package="rmoss_gz_base",
            executable="rmua19_robot_base",
            parameters=[robot_config, {"robot_name": robot["name"]}],
        )

        # 注意：robot_state_publisher 不加 namespace，
        # 这样 URDF 里的坐标系（chassis / front_mid360 / ...）就是全局的，
        # 与 odom_to_tf、传感器话题改写后的 frame_id 一致，rviz2 里才能连成 TF 树。
        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            remappings=[("joint_states", f"/{robot['name']}/joint_states")],
            parameters=[
                {
                    "use_sim_time": True,
                    "robot_description": robot_urdf_xml,
                }
            ],
        )

        robot_ign_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[{"config_file": aft_replace_ros_bridge_params}],
        )

        mid360_frame_normalizer = Node(
            package="rmu_gazebo_simulator",
            executable="normalize_mid360_frames.py",
            parameters=[
                {
                    "frame_id": "mid360",
                    "pointcloud_input_topic": "livox/lidar_raw",
                    "pointcloud_output_topic": "livox/lidar",
                    "imu_input_topic": "livox/imu_raw",
                    "imu_output_topic": "livox/imu",
                    "scan_input_topic": "rplidar_a2/scan_raw",
                    "scan_output_topic": "rplidar_a2/scan",
                    "scan_frame_id": "rplidar_a2",
                }
            ],
        )

        odom_to_tf = Node(
            package="rmu_gazebo_simulator",
            executable="odom_to_tf.py",
            parameters=[
                {"use_sim_time": True, "child_frame_id": "baselink"}
            ],
            remappings=[],
        )

        set_performer_service = ExecuteProcess(
            cmd=[
                "gz",
                "service",
                "-s",
                "/world/default/level/set_performer",
                "--reqtype",
                "gz.msgs.StringMsg",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "2000",
                "--req",
                f'data: "{robot["name"]}"',
            ],
            output="screen",
        )

        ld.append(spawn_robot)
        ld.append(robot_base)
        ld.append(robot_state_publisher)
        ld.append(robot_ign_bridge)
        ld.append(mid360_frame_normalizer)
        ld.append(odom_to_tf)
        ld.append(set_performer_service)

    return ld


def generate_launch_description():
    pkg_pb2025_robot_description = get_package_share_directory(
        "pb2025_robot_description"
    )

    declare_robot_xmacro_cmd = DeclareLaunchArgument(
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

    ld = LaunchDescription()

    ld.add_action(declare_robot_xmacro_cmd)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld

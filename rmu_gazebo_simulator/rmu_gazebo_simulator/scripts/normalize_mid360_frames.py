#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, PointCloud2


class NormalizeMid360Frames(Node):
    """把 Gazebo 传感器话题里的 frame_id 换成不含命名空间的坐标系名字。

    Gazebo 发布的传感器 frame_id 形如 `<model>/<link>/<sensor>`，
    与 robot_state_publisher 发布的 URDF 坐标系（不带机器人名前缀）对不上，
    所以在这里统一改写（2D 雷达 / mid360 点云 / mid360 IMU）。
    """

    def __init__(self):
        super().__init__("normalize_mid360_frames")

        self.declare_parameter("frame_id", "front_mid360")
        self.declare_parameter("pointcloud_input_topic", "livox/lidar_raw")
        self.declare_parameter("pointcloud_output_topic", "livox/lidar")
        self.declare_parameter("imu_input_topic", "livox/imu_raw")
        self.declare_parameter("imu_output_topic", "livox/imu")
        # 2D 雷达（可留空表示不处理）
        self.declare_parameter("scan_input_topic", "rplidar_a2/scan_raw")
        self.declare_parameter("scan_output_topic", "rplidar_a2/scan")
        self.declare_parameter("scan_frame_id", "front_rplidar_a2")

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        pointcloud_input_topic = (
            self.get_parameter("pointcloud_input_topic").get_parameter_value().string_value
        )
        pointcloud_output_topic = (
            self.get_parameter("pointcloud_output_topic").get_parameter_value().string_value
        )
        imu_input_topic = self.get_parameter("imu_input_topic").get_parameter_value().string_value
        imu_output_topic = self.get_parameter("imu_output_topic").get_parameter_value().string_value
        scan_input_topic = (
            self.get_parameter("scan_input_topic").get_parameter_value().string_value
        )
        scan_output_topic = (
            self.get_parameter("scan_output_topic").get_parameter_value().string_value
        )
        self.scan_frame_id = (
            self.get_parameter("scan_frame_id").get_parameter_value().string_value
        )

        # 发布用 Reliable：和 ros_gz_bridge / rviz2 的默认 QoS 一致，
        # 订阅端无论 Reliable 还是 Best-Effort 都能收到（SensorDataQoS 的 Best-Effort 发布
        # 会和 rviz2 的 Reliable 订阅匹配不上，表现为“话题有发布者但没有数据”）。
        pub_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            pointcloud_output_topic,
            pub_qos,
        )
        self.imu_pub = self.create_publisher(
            Imu,
            imu_output_topic,
            pub_qos,
        )
        self.scan_pub = self.create_publisher(
            LaserScan,
            scan_output_topic,
            pub_qos,
        )

        self.create_subscription(
            PointCloud2,
            pointcloud_input_topic,
            self.pointcloud_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            imu_input_topic,
            self.imu_cb,
            qos_profile_sensor_data,
        )
        if scan_input_topic:
            self.create_subscription(
                LaserScan,
                scan_input_topic,
                self.scan_cb,
                qos_profile_sensor_data,
            )

    def pointcloud_cb(self, msg: PointCloud2):
        msg.header.frame_id = self.frame_id
        self.pointcloud_pub.publish(msg)

    def imu_cb(self, msg: Imu):
        msg.header.frame_id = self.frame_id
        self.imu_pub.publish(msg)

    def scan_cb(self, msg: LaserScan):
        msg.header.frame_id = self.scan_frame_id
        self.scan_pub.publish(msg)


def main():
    rclpy.init()
    node = NormalizeMid360Frames()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

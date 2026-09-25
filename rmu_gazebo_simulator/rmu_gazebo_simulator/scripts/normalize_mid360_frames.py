#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2


class NormalizeMid360Frames(Node):
    def __init__(self):
        super().__init__("normalize_mid360_frames")

        self.declare_parameter("frame_id", "front_mid360")
        self.declare_parameter("pointcloud_input_topic", "livox/lidar_raw")
        self.declare_parameter("pointcloud_output_topic", "livox/lidar")
        self.declare_parameter("imu_input_topic", "livox/imu_raw")
        self.declare_parameter("imu_output_topic", "livox/imu")

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        pointcloud_input_topic = (
            self.get_parameter("pointcloud_input_topic").get_parameter_value().string_value
        )
        pointcloud_output_topic = (
            self.get_parameter("pointcloud_output_topic").get_parameter_value().string_value
        )
        imu_input_topic = self.get_parameter("imu_input_topic").get_parameter_value().string_value
        imu_output_topic = self.get_parameter("imu_output_topic").get_parameter_value().string_value

        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            pointcloud_output_topic,
            qos_profile_sensor_data,
        )
        self.imu_pub = self.create_publisher(
            Imu,
            imu_output_topic,
            qos_profile_sensor_data,
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

    def pointcloud_cb(self, msg: PointCloud2):
        msg.header.frame_id = self.frame_id
        self.pointcloud_pub.publish(msg)

    def imu_cb(self, msg: Imu):
        msg.header.frame_id = self.frame_id
        self.imu_pub.publish(msg)


def main():
    rclpy.init()
    node = NormalizeMid360Frames()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, 'chassis_odometry_gt', self.cb, 10)

    def cb(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        # 去掉命名空间前缀
        frame_id = msg.header.frame_id
        child_frame_id = msg.child_frame_id if msg.child_frame_id else 'base_link'
        if '/' in frame_id:
            frame_id = frame_id.split('/')[-1]
        if '/' in child_frame_id:
            child_frame_id = child_frame_id.split('/')[-1]
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(OdomToTF())

if __name__ == '__main__':
    main()

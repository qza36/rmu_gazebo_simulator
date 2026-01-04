#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rmoss_interfaces.msg import ChassisCmd


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')

        # Declare parameters
        self.declare_parameter('input_topic', 'cmd_vel')
        self.declare_parameter('output_topic', '/robot/robot_base/chassis_cmd')
        self.declare_parameter('chassis_type', 1)  # 1: velocity mode, 2: follow gimbal mode


        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.chassis_type = self.get_parameter('chassis_type').value

        self.subscription = self.create_subscription(
            Twist,
            input_topic,
            self.cmd_vel_callback,
            10
        )

        self.publisher = self.create_publisher(
            ChassisCmd,
            output_topic,
            10
        )

        self.get_logger().info(f'Relay node started: {input_topic} -> {output_topic}')
        self.get_logger().info(f'Chassis type: {self.chassis_type}')

    def cmd_vel_callback(self, msg):
        chassis_cmd = ChassisCmd()
        chassis_cmd.type = self.chassis_type
        chassis_cmd.twist.linear.x = msg.linear.x
        chassis_cmd.twist.linear.y = msg.linear.y
        chassis_cmd.twist.linear.z = msg.linear.z
        chassis_cmd.twist.angular.x = msg.angular.x
        chassis_cmd.twist.angular.y = msg.angular.y
        chassis_cmd.twist.angular.z = 4.5

        self.publisher.publish(chassis_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
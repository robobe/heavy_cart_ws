#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3

class TwistStamper(Node):

    def __init__(self):
        super().__init__('twist_stamper')

        self.declare_parameter("frame_id", "")
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.create_timer(1.0, self.timer_handler)
        self.publisher_ = self.create_publisher(TwistStamped, '/i2c_controller/cmd_vel', 10)

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel_in',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def timer_handler(self):
        msg = Twist()
        msg.linear = Vector3(x=10.0, y=0.0, z=0.0)
        msg.angular= Vector3(x=0.0, y=0.0, z=0.0)

        self.listener_callback(msg)

    def listener_callback(self, inMsg):
        outMsg = TwistStamped()
        outMsg.header = Header()
        outMsg.header.stamp = self.get_clock().now().to_msg()
        outMsg.header.frame_id = self.frame_id
        outMsg.twist = inMsg

        self.publisher_.publish(outMsg)
        # self.get_logger().info('X: "%f"' % outMsg.twist.linear.x)


def main(args=None):
    rclpy.init(args=args)

    twist_stamper = TwistStamper()
    rclpy.spin(twist_stamper)

    twist_stamper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
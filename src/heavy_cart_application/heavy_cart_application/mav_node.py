#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from heavy_cart_application.mavlink import Sender , Receiver


class MyNode(Node):
    def __init__(self):
        node_name="mav_node"
        super().__init__(node_name)
        self.__mav_sender = Sender()
        self.__mav_receiver = Receiver()
        self.__mav_sender.run()
        self.__mav_receiver.run()
        self.__mav_receiver.register_handler("RC_CHANNELS_OVERRIDE", self.__joy_handler)
        self.__timer = self.create_timer(1.0, self.__timer_handler)
        self.get_logger().info("Hello ROS2")

    def __timer_handler(self):
        self.get_logger().info("Timer hamdl;er")

    def __joy_handler(self, mav_msg):
        self.get_logger().info(f"{mav_msg.chan1_raw}, {mav_msg.chan2_raw}")
        
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
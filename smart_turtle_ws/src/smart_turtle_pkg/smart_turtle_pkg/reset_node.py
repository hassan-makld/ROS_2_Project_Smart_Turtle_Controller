#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import time


class NodeReset(Node):
    def __init__(self):
        super().__init__('Node_reset')
        
        self.reset_turtlesim_client = self.create_client(Empty, '/clear')
        self.clear_turtlesim_client = self.create_client(Empty, '/reset')

        while not self.reset_turtlesim_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for reset service")
        while not self.clear_turtlesim_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for clear service")

        self.timer_ = self.create_timer(10.0,self.service_callback)


    def service_callback(self):
        request = Empty.Request()

        self.reset_turtlesim_client.call_async(request)
        self.clear_turtlesim_client.call_async(request)
            


def main(args=None):
    rclpy.init(args=args)
    node = NodeReset()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
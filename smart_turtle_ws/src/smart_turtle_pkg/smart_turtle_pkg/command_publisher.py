#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from smart_turtle_interface.msg import TurtleCommand

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')

        self.declare_parameter('pattern','circle')
        self.declare_parameter('speed',2.0)

        self.pattern = self.get_parameter('pattern').value
        self.speed = self.get_parameter('speed').value

        self.publisher_ = self.create_publisher(TurtleCommand,'/customcommand',10)
        self.timer_ = self.create_timer(1.0, self.publish_custom)

    def publish_custom(self):
        msg = TurtleCommand()
        msg.command = self.pattern
        msg.speed = self.speed
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()  
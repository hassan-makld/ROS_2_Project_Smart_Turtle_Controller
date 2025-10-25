#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from smart_turtle_interface.msg import TurtleCommand
import time
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__('Turtle_Controller')

        self.publisher_ = self.create_publisher(Twist,'/turtle1/cmd_vel',10)

        self.subscriber_ = self.create_subscription(TurtleCommand,'/customcommand',self.callback_receive,10)

    def callback_receive(self,msg: TurtleCommand):
        if msg.command == 'circle':
            self.draw_circle(msg.speed)
        elif msg.command == 'square':
            self.draw_square(msg.speed)
        elif msg.command == 'spiral':
            self.draw_spiral(msg.speed)
        else:
            self.get_logger().info("provide another pattern")

    

    def draw_circle(self,speed):
        msgtwist = Twist()
        msgtwist.linear.x = speed
        msgtwist.angular.z = speed / 2

        self.get_logger().info("circle drawing...")

        for i in range(200):
            self.publisher_.publish(msgtwist)
            time.sleep(0.05)

        msgtwist.linear.x = 0.0
        msgtwist.angular.z = 0.0
        self.publisher_.publish(msgtwist)
        self.get_logger().info("circle is now done")

    def draw_square(self,speed):
        msgtwist = Twist()
        for i in range(4):
            msgtwist.linear.x = speed
            msgtwist.angular.z = 0.0
            self.publisher_.publish(msgtwist)
            time.sleep(1.0)
            msgtwist.linear.x = 0.0
            msgtwist.angular.z = speed
            self.publisher_.publish(msgtwist)
            time.sleep(math.pi / (2 * speed))
        
        msgtwist.linear.x = 0.0
        msgtwist.angular.z = 0.0
        self.publisher_.publish(msgtwist)
        self.get_logger().info("square is now done")

    def draw_spiral(self,speed):
        msgtwist = Twist()
        msgtwist.angular.z = 1.0
        radius = 0.1
        self.get_logger().info("spiral drawing...")
        for i in range(100):
            msgtwist.linear.x = radius
            self.publisher_.publish(msgtwist)
            radius += 0.01
            time.sleep(0.1)

        msgtwist.linear.x = 0.0
        msgtwist.angular.z = 0.0
        self.publisher_.publish(msgtwist)
        self.get_logger().info("spiral is now done")




def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys
import select
import tty
import termios
import time
import math
# import matplotlib.pyplot as plt
# import p1pygame

class OdomPublsiher(Node):
    def __init__(self):
        super().__init__('OdomPublisher')

        # self.sub_node = rclpy.create_node('OdomSubscriber')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        # self.robot_x = 0
        # self.robot_y = 0
        # self.robot_theta = 0
        self.pub_zeros()

    def pub_zeros(self):
        # Create an Odometry message
        odom_msg = Odometry()
        # Populate the message fields with your odometry data
        # For example:
        while True:
            odom_msg.pose.pose.position.x = 0.0
            odom_msg.pose.pose.position.y = 0.0
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.x = 0.0
            odom_msg.pose.pose.orientation.y = 0.0
            odom_msg.pose.pose.orientation.z = 0.0
            odom_msg.pose.pose.orientation.w = 0.0

            odom_msg.twist.twist.linear.x = 0.0
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.linear.z = 0.0

            # Publish the message
            print("publishing....\n")
            self.odom_pub.publish(odom_msg)
            # rate.sleep()


    # def quaternion_to_euler(self, x, y, z, w):
    #     # Conversion to Euler angles (roll, pitch, yaw)
    #     roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * 2 + y * 2))
    #     pitch = math.asin(2 * (w * y - z * x))
    #     yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * 2 + z * 2))
    #     return roll, pitch, yaw




# main
def main(args=None):
    rclpy.init(args=args)
    node = OdomPublsiher()
    # rclpy.spin(node)

    # node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '_main_':
    main()
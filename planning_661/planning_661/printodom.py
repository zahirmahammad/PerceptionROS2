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
import numpy as np
# import p1pygame

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('OdomSubscriber')

        # self.sub_node = rclpy.create_node('OdomSubscriber')
        self.odom_subs = self.create_subscription(Odometry, '/odom',  self.odom_callback, 10)
        # self.robot_x = 0
        # self.robot_y = 0
        # self.robot_theta = 0

    def odom_callback(self, msg):
        self.robot_x =  msg.pose.pose.position.x
        self.robot_y =  msg.pose.pose.position.y
        self.robot_z =  msg.pose.pose.position.z

        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        _, _, self.robot_theta = self.quaternion_to_euler(x, y, z, w)

        print('Position - X: %f, Y: %f, Z: %f' % (self.robot_x, self.robot_y, self.robot_z))
        print('Orientation Angle: %f degrees' % self.robot_theta)
        print('Orientation Angle: %f degrees' % np.rad2deg(self.robot_theta))


    def quaternion_to_euler(self, x, y, z, w):
        # Conversion to Euler angles (roll, pitch, yaw)
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x**2 + y**2)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if math.fabs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use +-90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw




# main
def main(args=None):
    rclpy.init(args=args)
    node = OdomSubscriber()
    rclpy.spin(node)

    # node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '_main_':
    main()
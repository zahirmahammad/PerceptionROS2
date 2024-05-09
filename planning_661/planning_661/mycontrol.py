#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile

import sys
import select
import tty
import termios
import time
import math
import p1pygame

wheel_radius = 3.3 # r
track_width = 28.7 # L 
timestep = 1


# action_list = p1pygame.action_list
points_list = p1pygame.points_list
# print(len(action_list))
print(len(points_list))
print(points_list)
points_list= [list(i) for i in points_list]

for i in range(len(points_list)-1):
    points_list[i+1][0] -= points_list[0][0]    
    points_list[i+1][1] -= points_list[0][1]    

points_list[0][0] -= points_list[0][0]    
points_list[0][1] -= points_list[0][1] 

print(points_list)




class MyrobotController(Node):
    def __init__(self):
        super().__init__('MyrobotController')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sub_node = rclpy.create_node('ScanSubscriber')
        # self.odom_subs = self.sub_node.create_subscription(Odometry, '/odom',  self.odom_callback, 10)

        # self.robot_x = 0
        # self.robot_y = 0
        # self.robot_theta = 0

        qos_profile = QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,history=rclpy.qos.HistoryPolicy.KEEP_LAST,depth=5)
        self.subscription = self.sub_node.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )


    def scan_callback(self, msg):
        # Get the range measurements
        ranges = msg.ranges
        # Assuming the lidar only has 3 directions: left, front, and right
        # Get distance measurements in each direction
        ranges_list = []
        for i in ranges:
            ranges_list.append(i)

        self.num_range = len(ranges_list)
        # self.left_distance = ranges[2]  # 90 degrees left
        # self.front_distance = ranges[1]  # Front
        # self.right_distance = ranges[0]  # 90 degrees right

        # Print the distances
        # print(f"Left Distance: {self.left_distance}")
        # print(f"Front Distance: {self.front_distance}")
        # print(f"Right Distance: {self.right_distance}")
        print(f"Total Distance: {self.num_range}")


    def run(self):
        velocity_message = Twist()
        linear_vel=0.0
        angular_vel=0.0
        
        min_thetadiff = 100
        max_thetadiff = -100

        for i,point in enumerate(points_list[:-1]):
            print("new point ", points_list[i+1])



            while True: 
                rclpy.spin_once(self.sub_node)
                print(self.num_range)
                
            #     theta_diff = self.robot_theta - math.atan2((points_list[i+1][1]-(self.robot_y*100)), (points_list[i+1][0]-(self.robot_x*100)))
            #     if abs(theta_diff) >180:
            #          theta_diff = abs(theta_diff) - 360

            #     print(theta_diff)
            #     linear_vel = 0.25*(1.00 - 0.5*abs(theta_diff))
            #     angular_vel = 2*-theta_diff

            #     if theta_diff > max_thetadiff:
            #         max_thetadiff = theta_diff
            #     if theta_diff < min_thetadiff:
            #         min_thetadiff = theta_diff

            #     #find difference between those 2 points
            #     dist_diff = ((points_list[i+1][1]-(self.robot_y*100))**2+(points_list[i+1][0]-(self.robot_x*100))**2)**0.5
            #     print(dist_diff)
            #     if dist_diff < 5:
            #         print("breaking...")
            #         break
                
            #     # Publish the twist message
            #     velocity_message.linear.x = linear_vel
            #     velocity_message.angular.z = angular_vel
                
            #     # publish velocities
            #     self.cmd_vel_pub.publish(velocity_message)



                # time.sleep(timestep)
        velocity_message.linear.x = 0.0
        velocity_message.angular.z = 0.0
        print("min angular vel ", min_thetadiff )
        print("max angular vel ", max_thetadiff )
        # publish velocities
        self.cmd_vel_pub.publish(velocity_message)

# main
def main(args=None):
    rclpy.init(args=args)
    node = MyrobotController()
    # rclpy.spin(node)

    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
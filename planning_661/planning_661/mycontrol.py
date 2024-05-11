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
import matplotlib.pyplot as plt
# import p1pygame

wheel_radius = 3.3 # r
track_width = 28.7 # L 
timestep = 1


# action_list = p1pygame.action_list
# points_list = p1pygame.points_list
points_list = [[50, 100, 0], (76, 100, 0), (102, 100, 0), (128, 100, 0), (154, 100, 0), (172, 105, 1), (187, 117, 2), (203, 138, 2), (218, 158, 2), (234, 179, 2), (249, 191, 1), (273, 202, 1), (291, 206, 0), (310, 201, 11), (325, 189, 10), (340, 168, 10), (356, 148, 10), (371, 136, 11), (394, 124, 11), (418, 113, 11), (441, 102, 11), (460, 98, 0), (478, 102, 1), (502, 114, 1), (516, 126, 1), (531, 138, 1), (546, 151, 2)]
# points_list = [[50, 100, 0], (76, 100, 0), (94, 95, 11), (118, 84, 11), (141, 72, 11), (160, 68, 0), (178, 73, 1), (193, 86, 2)]# print(len(action_list))
print(len(points_list))
print(points_list)
points_list= [list(i) for i in points_list]

for i in range(len(points_list)-1):
    points_list[i+1][0] -= points_list[0][0]    
    points_list[i+1][1] -= points_list[0][1]    

points_list[0][0] -= points_list[0][0]    
points_list[0][1] -= points_list[0][1] 

odom_cx = 0
odom_cy = 0
for i in range(len(points_list)):
    points_list[i][0]-=odom_cx
    points_list[i][1]-=odom_cy

start = time.time()

print(points_list)

l_angular_vel = []
l_linear_vel=[]

class MyrobotController(Node):
    def __init__(self):
        super().__init__('MyrobotController')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sub_node = rclpy.create_node('OdomSubscriber')
        self.odom_subs = self.sub_node.create_subscription(Odometry, '/odom',  self.odom_callback, 10)
        # self.robot_x = 0
        # self.robot_y = 0
        # self.robot_theta = 0

    def odom_callback(self, msg):
        self.robot_x =  msg.pose.pose.position.x
        self.robot_y =  msg.pose.pose.position.y

        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        _, _, self.robot_theta = self.quaternion_to_euler(x, y, z, w)

        # self.get_logger().info('Position - X: %f, Y: %f' % (self.robot_x, self.robot_y))
        # self.get_logger().info('Orientation Angle: %f degrees' % math.degrees(self.robot_theta))

    def quaternion_to_euler(self, x, y, z, w):
        # Conversion to Euler angles (roll, pitch, yaw)
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * 2 + y * 2))
        pitch = math.asin(2 * (w * y - z * x))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * 2 + z * 2))
        return roll, pitch, yaw


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


                # linear_vel_cm = (wheel_radius/2)*(action[0]+action[1])
                # angular_vel_cm = (wheel_radius/track_width)*(action[1]-action[0])
                
                theta_diff = self.robot_theta - math.atan2((points_list[i+1][1]-(self.robot_y*100)), (points_list[i+1][0]-(self.robot_x*100)))
                if abs(theta_diff) >180:
                     theta_diff = abs(theta_diff) - 360

                linear_vel = 0.2*(1.00 - 0.6*abs(theta_diff))
                angular_vel = 2*-theta_diff

                # print("theta_diff", theta_diff)
                # print("angular_vel", angular_vel)

                # for storing angular vel and theta diff
                l_linear_vel.append(linear_vel)
                l_angular_vel.append(angular_vel)

                if angular_vel > max_thetadiff:
                    max_thetadiff = angular_vel
                if angular_vel < min_thetadiff:
                    min_thetadiff = angular_vel

                #find difference between those 2 points
                dist_diff = ((points_list[i+1][1]-(self.robot_y*100))**2+(points_list[i+1][0]-(self.robot_x*100))**2)**0.5
                print(dist_diff)    
                if dist_diff < 4:
                    print("breaking...")
                    break
                
                # need to be corrected accoridng to gazebo's real time factor
                # p_linear = 1
                # p_angular = 1.05

                # linear_vel = (linear_vel_cm/100)*p_linear
                # angular_vel = (angular_vel_cm/1)*p_angular

                #printing actions and publishing velocities
                # print("\nAction ",i)
                # print("Steer Angle",angular_vel)
                # print("Linear Velocity",linear_vel)
                
                # Publish the twist message
                velocity_message.linear.x = linear_vel
                velocity_message.angular.z = angular_vel
                
                # publish velocities
                self.cmd_vel_pub.publish(velocity_message)

            #  position:
            #       x: 0.17101175549201383
            #       y: -2.7618773738964766
            #       z: 0.0
            #     orientation:
            #       x: 0.0
            #       y: 0.0
            #       z: -0.6848864256227182
            #       w: 0.7286498363396076

                # time.sleep(timestep)
        velocity_message.linear.x = 0.0
        velocity_message.angular.z = 0.0
        print("min theta ", min_thetadiff )
        print("max theta ", max_thetadiff )
        # publish velocities
        self.cmd_vel_pub.publish(velocity_message)
        # print("linear_vel ", l_linear_vel)      
        # print("angular vel ",l_angular_vel)
        # record end time
        end = time.time()
        
        # print the difference between start 
        # and end time in milli. secs
        print("The time of execution of above program is :",
            (end-start) * 10**3, "ms")  

# main
def main(args=None):
    rclpy.init(args=args)
    node = MyrobotController()
    # rclpy.spin(node)

    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '_main_':
    main()
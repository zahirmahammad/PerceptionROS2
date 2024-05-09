import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
from copy import deepcopy
import numpy as np
from geometry_msgs.msg import Twist
import math


def stop_fn(image):
    stop = False
    # Stop Sign Cascade Classifier xml
    stop_sign = cv2.CascadeClassifier('/home/zahir/turtlebot3_ws/src/perception/perception/cascade_stop_sign.xml')
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    stop_sign_scaled = stop_sign.detectMultiScale(gray, 2.0, 5)
    if len(stop_sign_scaled)!=0:
        stop = True
        # Detect the stop sign, x,y = origin points, w = width, h = height
        for (x, y, w, h) in stop_sign_scaled:
            # Draw rectangle around the stop sign
            stop_sign_rectangle = cv2.rectangle(image, (x,y), (x+w, y+h), (0, 255, 0), 3)
            # Write "Stop sign" on the bottom of the rectangle
            stop_sign_text = cv2.putText(img=stop_sign_rectangle, text="STOP", org=(x, y+h+30), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 255), thickness=2, lineType=cv2.LINE_4)

    return stop

def horizon():
    pass

def DynamicObstacle():
    pass

def get_corners(cb_l, cb_w, map_image):
  grey_img = cv2.cvtColor(map_image, cv2.COLOR_BGR2GRAY)
  grey_img_8bit = cv2.convertScaleAbs(grey_img)
  # cb_l=10
  # cb_w=7
  retval, corners = cv2.findChessboardCorners(grey_img_8bit ,(cb_l, cb_w), None)
    # checking if the specific sized corners set is detected in this image
  if retval==True:
      corners = np.squeeze(corners)
  img_copy = deepcopy(map_image)
  corner_points = [corners[0], corners[cb_l-1], corners[(cb_l*cb_w)-cb_l], corners[-1]]
  for corner in corner_points:
      # print(corner)
      coord = (int(corner[0]), int(corner[1]))
      # approximating corner value to nearest pixel coordinate and circle with mentioned radius and thickness is drawn around it
      cv2.circle(img=img_copy, center=coord, radius=4, color=(255, 0, 0), thickness=2)

  # corner_ points format to list
  # corner_points = corner_points

  return retval, corner_points, img_copy


def way_points(corner_pts, img):
    # get mid point of 1 points in corner_pts
    far_point = (int((corner_pts[0][0] + corner_pts[2][0]) / 2), int((corner_pts[0][1] + corner_pts[2][1]) / 2))
    near_point = (int((corner_pts[1][0] + corner_pts[3][0]) / 2), int((corner_pts[1][1] + corner_pts[3][1]) / 2)) 
    
    cv2.circle(img=img, center=far_point, radius=4, color=(0, 0, 255), thickness=2)
    cv2.circle(img=img, center=near_point, radius=4, color=(0, 0, 255), thickness=2)
    
    return far_point, near_point, img









class Percept(Node):    
    def __init__(self):
        super().__init__('camera_viewer')
        
        self.sub_node = rclpy.create_node('ImageSubscriber')

        # Set the QoS profile to match the camera node's QoS settings
        qos_profile = QoSProfile(
            depth=10,  # Adjust the depth as needed
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Set the reliability policy
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self.subscription = self.sub_node.create_subscription(Image, '/camera/image_raw',  self.image_callback, qos_profile )

        self.cv_bridge = CvBridge()

        self.stop = False
        self.iw = 0
        self.ih = 0
        self.robot_ref_pixel = ()


    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to a CV2 image
            self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Stop Sign 
            self.stop = stop_fn(self.cv_image)
            print(self.stop)

            self.iw = self.cv_image.shape[1]
            self.ih = self.cv_image.shape[0]
            self.robot_ref_pixel = (self.iw/2, self.ih)

            # Display the image using OpenCV
            cv2.imshow('Camera Feed', self.cv_image)
            cv2.waitKey(1)  # Adjust the delay time as needed
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def temprun(self):
        while True:
            rclpy.spin_once(self.sub_node)


    # def run(self):
    #     rclpy.spin_once(self.sub_node)

    #     velocity_message = Twist()
    #     linear_vel=0.0
    #     angular_vel=0.0
        
    #     min_thetadiff = 100
    #     max_thetadiff = -100

    #     points_list=[]
    #     _, corner_pts, imgc = get_corners(7,5, self.cv_image)
    #     farp, nearp, img_path = way_points(corner_pts, imgc)
    #     points_list.append(nearp)
    #     points_list.append(farp)


    #     for i,point in enumerate(points_list):
    #         print("new point ", points_list[i])

    #         while True: 
    #             rclpy.spin_once(self.sub_node)


    #             # linear_vel_cm = (wheel_radius/2)*(action[0]+action[1])
    #             # angular_vel_cm = (wheel_radius/track_width)*(action[1]-action[0])
                
    #             theta_diff = 90 - math.atan2((points_list[i][1]-(self.robot_ref_pixel[1])), (points_list[i][0]-(self.robot_ref_pixel[0])))
    #             if abs(theta_diff) >180:
    #                  theta_diff = abs(theta_diff) - 360

    #             print(theta_diff)
    #             # linear_vel = 0.25*(1-(theta_diff))
    #             linear_vel = 0.05*(1.00 - 0.5*abs(theta_diff))
    #             angular_vel = 2*-theta_diff
    #             # linear_vel = 0.3
    #             # angular_vel = 5*-theta_diff

    #             if theta_diff > max_thetadiff:
    #                 max_thetadiff = theta_diff
    #             if theta_diff < min_thetadiff:
    #                 min_thetadiff = theta_diff

    #             #find difference between those 2 points
    #             dist_diff = ((points_list[i][1]-(self.robot_ref_pixel[1]))**2+(points_list[i][0]-(self.robot_ref_pixel[0]))**2)**0.5
    #             print(dist_diff)
    #             if dist_diff < 10:
    #                 print("breaking...")
    #                 break
                
    #             # need to be corrected accoridng to gazebo's real time factor
    #             # p_linear = 1
    #             # p_angular = 1.05

    #             # linear_vel = (linear_vel_cm/100)*p_linear
    #             # angular_vel = (angular_vel_cm/1)*p_angular

    #             #printing actions and publishing velocities
    #             # print("\nAction ",i)
    #             # print("Steer Angle",angular_vel)
    #             # print("Linear Velocity",linear_vel)
                
    #             # Publish the twist message
    #             velocity_message.linear.x = linear_vel
    #             velocity_message.angular.z = angular_vel
                
    #             # publish velocities
    #             self.cmd_vel_pub.publish(velocity_message)



    #             # time.sleep(timestep)
    #     velocity_message.linear.x = 0.0
    #     velocity_message.angular.z = 0.0
    #     print("min theta ", min_thetadiff )
    #     print("max theta ", max_thetadiff )
    #     # publish velocities
    #     self.cmd_vel_pub.publish(velocity_message)






def main(args=None):
    rclpy.init(args=args)

    percept = Percept()

    percept.temprun()

    # percept.run()
    percept.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
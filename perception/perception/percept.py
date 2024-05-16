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
import torch
import time

model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/zahir/turtlebot3_ws/src/perception/perception/best_100.pt', force_reload=True)
start_time = time.time()

### ------------------ Stop Sign -------------------------------- #####
def stop_fn_classifier(image):
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

def stop_fn(image):
    stop = False

    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # model = ()
    results = model(image)

    output_list = results.pandas().xyxy[0].to_dict('records')

    # Draw bounding boxes on the image
    if output_list:
        for detection in output_list:
            if "stop" in detection['name']:
                if detection['confidence'] > 0.89: 
                    label = detection['name']
                    confidence = detection['confidence']
                    xmin, ymin, xmax, ymax = int(detection['xmin']), int(detection['ymin']), int(detection['xmax']), int(detection['ymax'])
                    # Draw bounding box
                    cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                    # Add label and confidence
                    label_text = f"{label}: {confidence:.2f}"
                    cv2.putText(image, label_text, (xmin, ymin - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    stop = True
    return stop

### ------------------ End Stop Sign -------------------------------- #####


## -------------------- Horizon ---------------------------- ##

def horizon_line(image):
    
    # convert gray
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #  thresholding
    thresh = cv2.threshold(gray, 245, 255, cv2.THRESH_BINARY)[1]
    # edge detection
    edges = cv2.Canny(thresh, 50, 150, apertureSize=3)
    # Line Detection using Hough Transform
    lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=70, minLineLength=90, maxLineGap= 20)

    # # plot all detected edges(lines)
    # image1 = image.copy()
    # for line in lines:
    #     x1 , y1, x2 , y2 = line[0]
    #     cv2.line(image1, (x1,y1), (x2,y2), (250,0,0),2)

    # Filter out horizontal lines
    filtered_lines = []
    try:
        for i in range(len(lines)-1):
            x1, y1, x2, y2 = lines[i][0]
            angle1 = np.rad2deg(np.arctan2(y2 - y1, x2 - x1))
            # print(angle1)
            parallel_count = 0
            if -5<=angle1<=5 or 85<=angle1<=95 or -90<=angle1<=-85:
                continue

            filtered_lines.append(lines[i])
    except:
        return None
    # image2 = image.copy()
    for line in filtered_lines:
        x1 , y1, x2 , y2 = line[0]
        # cv2.line(image, (x1,y1), (x2,y2), (250,0,0),2)
    print("No of filtered lines: ", len(filtered_lines))


    def plot_and_find_intersection(image, line1, line2):

        # Extract line coordinates
        line1 = tuple(int(x) for x in line1)
        line2 = tuple(int(x) for x in line2)
        x1_1, y1_1, x1_2, y1_2  = line1  # changed order of points
        x2_1, y2_1, x2_2, y2_2 = line2

        pt1_l1 = (int(x1_1),int(y1_1)) 
        pt1_l2 =  (int(x2_1),int(y2_1)) 
        pt2_l1 = (int(x1_2),int(y1_2)) 
        pt2_l2= (int(x2_2),int(y2_2))


        # # Draw lines on the image 
        cv2.line(image, pt1_l1,pt2_l1, (255, 0, 0), 4)  # Blue line
        cv2.line(image, pt1_l2,pt2_l2,  (0, 0, 255), 4) # red line 

        # slope of line1
        start_point= list(pt2_l1)
        end_point = list(pt1_l1)
        l1_slope = (end_point[1] - start_point[1]) / (end_point[0] - start_point[0])
        print("l1_slope:",l1_slope)

        l2_start_point = list(pt2_l2)
        l2_end_point = list(pt1_l2)
        l2_slope = (l2_end_point[1] - l2_start_point[1]) / (l2_end_point[0] - l2_start_point[0])
        print("l2_slope:",l2_slope)

        # Check for parallel lines (avoid division by zero)
        if l1_slope == l2_slope:
            return None

        # Calculate the x-coordinate of the intersection point
        x_intersect = (l2_start_point[1] - start_point[1] + l1_slope * start_point[0] - l2_slope * l2_start_point[0]) / (l1_slope - l2_slope)

        # Calculate the y-coordinate of the intersection point using the equation of either line
        y_intersect = l1_slope * (x_intersect - start_point[0]) + start_point[1]
        print("intersection_points(x,y):", x_intersect , y_intersect)
        x_intersect = int(x_intersect)
        y_intersect = int(y_intersect)
        hl_start_point = (0, y_intersect)
        hl_end_point = (image.shape[1], y_intersect)
        # Draw the line parallel to x-axis
        cv2.line(image, hl_start_point, hl_end_point, (255, 155, 0), 2)  # Blue line with thickness 2  # horizon  line 

        cv2.circle(image, (x_intersect,y_intersect), 5, (255, 0, 0), -1)  # vanishing point # Green circle at intersection #random pt test

        return (x_intersect, y_intersect) , image

    # ipt_img = image.copy()
    # print("img_shape", ipt_img.shape)
    try:
        intersection_point ,image = plot_and_find_intersection(image, filtered_lines[0][0], filtered_lines[1][0])
    except:
        intersection_point = None
    if intersection_point:
        print("Intersection point:", intersection_point)
        # cv2.imshow("Image with Lines and Intersection", image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # plt.imshow(image)
        # plt.show()
        return intersection_point[1]
    else:
        print("Lines don't intersect!")

        return None

## -------------------- End Horizon ---------------------------- ##


## ---------------------- Dynamic Obstacle -------------------- ##
def motion(flow):
     
    #height and width
    h, w = flow.shape[:2]

    #velocity vectors in x and y direction
    fx, fy = flow[:,:,0], flow[:,:,1]

    # angle of motion vector (0 to 2pi)
    angle = np.arctan2(fy, fx) + np.pi

    # magnitude of motion vector
    mag =  np.sqrt(fx**2+fy**2)

    # Initailizing empty mask
    mask = np.zeros((h, w, 1), np.uint8)
    
    #Velocity Threshold
    threshold_value = 5
    mask[:,:,0] = np.where(mag > threshold_value, 255, 0)

    #reducing the noise
    kernel = np.ones((3,3),np.uint8)

    #closing operation
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,kernel, iterations=1)
    # dilate the image 
    # mask = cv2.dilate(mask, kernel, iterations=1) 

    return mask

## ---------------------- End Dynamic Obstacle -------------------- ##


## -------------- Planning -------------------------##
def get_center(image):

    try: 
        # 2. Preprocess the image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # 3. Clear background
        _, thresh = cv2.threshold(blurred, 240, 255, cv2.THRESH_BINARY)
        thresh[:275,:] = 0    
        
        # 4. Find Cont
        contours,hierarchy = cv2.findContours(thresh, 1, 2)
        # contours = sorted(contours, key = cv2.contourArea, reverse=True)
        # centers=[]
        # for cnt in contours:
        #     # print(area)
        #     M = cv2.moments(cnt)
        #     # print(M)
        #     cx = int(M['m10']/M['m00'])
        #     cy = int(M['m01']/M['m00'])
        #     centers.append([cx, cy])  
           
        # if centers[0][1] > image.shape[0] - 5:
        #     goto = centers[1]
        # else:
        #     goto = centers[0] 
        cnt = contours[0]
        area = cv2.contourArea(contours[0])
        if len(contours) > 1:
            if area < 12000:
                cnt = contours[1]
        # # print(area)
        M = cv2.moments(cnt)
        # print(M)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        goto = [cx, cy]

            # goto = [cx, cy-200]
        cv2.circle(image, (goto[0], goto[1]), 5, (0, 0, 255), -1)

        return goto

    except:
        # pass
        return [image.shape[0]//2, image.shape[1]//2]
## ----------------- End Planning ------------------##


counter = []



class Percept(Node):    
    def __init__(self):
        super().__init__('camera_viewer')
        
        self.velpub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sub_node = rclpy.create_node('ImageSubscriber')

        # Set the QoS profile to match the camera node's QoS settings
        qos_profile = QoSProfile(
            depth=10,  # Adjust the depth as needed
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Set the reliability policy
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # self.subscription = self.sub_node.create_subscription(Image, '/image_raw',  self.image_callback, qos_profile )
        self.subscription = self.sub_node.create_subscription(Image, '/camera/image_raw',  self.image_callback, qos_profile )
        self.cv_bridge = CvBridge()
        self.stop = False
        self.obstacle = False
        self.iw = 0
        self.ih = 0
        self.y_point = 0
        self.robot_ref_pixel = ()
        self.counter = 0
        self.y_point = None


    def image_callback(self, msg):
        global start_time
        try:
            # Convert the ROS Image message to a CV2 image
            self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            self.iw = self.cv_image.shape[1]
            self.ih = self.cv_image.shape[0]
            self.robot_ref_pixel = (self.iw/2, self.ih)

            # Detect Horizon Line
            if self.y_point is None:
                self.y_point = horizon_line(self.cv_image)
                cv2.imshow('Horizon Line', self.cv_image)
                time.sleep(2)

            # Display the horizon line
            if self.y_point is not None:
                cv2.line(self.cv_image, (0, self.y_point), (self.iw, self.y_point), (255, 155, 0), 2)
                cv2.putText(self.cv_image, "Horizon", (self.iw//2, self.y_point-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 155, 0), 2)


            # Stop Sign 
            self.stop = stop_fn(self.cv_image)
            if self.stop:
                cv2.putText(self.cv_image, "Stop Sign Detected", (self.iw//2 - 150, self.ih-150), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 200), 2)

            # print(self.stop)

            # Planner
            self.center = get_center(self.cv_image)

            # Dynamic Obstacle
            # current_time = time.time()
            # if current_time - start_time >= 0.2:
                # print('im in dynamics')
            
            if self.obstacle == True:
                print("Dynamic Obstacle Ahead")
                check = self.DynamicObstacle(self.cv_image)
                counter.append(check)
                if len(counter) > 5:
                    # find which is more in number true or false
                    if counter.count(True) > 2:
                        self.obstacle = True
                    else:
                        self.obstacle = False
                    counter.clear()
            else:
                # print(self.DynamicObstacle(self.cv_image))
                self.obstacle = self.DynamicObstacle(self.cv_image)


                # start_time = current_time
            # if self.obstacle:
                # cv2.putText(self.cv_image, "Dynamic Obstacle Ahead", (self.iw//2, 0+50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 100, 230), 2)

            # horizon line
            # y_point = horizon_line(self.cv_image)




            # Display the image using OpenCV
            cv2.imshow('Camera Feed', self.cv_image)
            cv2.waitKey(1)  # Adjust the delay time as needed
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")    




    def run(self, x, y):
        goalx, goaly = (self.iw//2, self.ih)
        goal_angle = 90

        m = -math.degrees(math.atan2(y - goaly, x - goalx))
        if m < 0:
            m+=360
        turn_angle = m - (goal_angle - 5) 

        print("Turn:  ", turn_angle)

        ang_vel = np.radians(turn_angle)*0.12

        velocity = Twist()
        # velocity.linear.x = 0.0
        # velocity.angular.z = ang_vel

        velocity.linear.x = 0.08

        if y > self.ih - 70 or x < 0 + 30 or x > self.iw - 30:
            velocity.angular.z = ang_vel
            velocity.linear.x = 0.0

        if -5<turn_angle<5:
            velocity.linear.x = 0.08

        if self.obstacle or self.stop:
            velocity.linear.x = 0.0
            velocity.angular.z = 0.0 
            # time.sleep(2)

        self.velpub.publish(velocity)




    def temprun(self):
        # Resize the frame (640x480)
        ## Dynamic Obstacle
        rclpy.spin_once(self.sub_node)
        print("Image size: ", (self.iw,self.ih))

        # width = 640
        # height = 480
        # self.prev_img = cv2.resize(self.cv_image, (width,height))
        self.prev_gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        counter = 0
        while True:
            rclpy.spin_once(self.sub_node)
            if self.center == [self.cv_image.shape[0]//2, self.cv_image.shape[1]//2]:
                counter+=1
                if counter > 50:
                    break
            else:
                self.run(self.center[0], self.center[1])


            



    def DynamicObstacle(self, image):
        obstacle = False
        #resize frame
        # curr_img = cv2.resize(image, (640, 480))

        #Grayscale
        curr_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Calcualting Optical flow
        flow = cv2.calcOpticalFlowFarneback(self.prev_gray, curr_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        
        # Updating previous gray frame
        self.prev_gray = curr_gray

        #creating motion mask
        motion_mask = motion(flow)

        #make an image copy
        # img_copy = curr_img.copy()

        #create a bounding box
        # final_img,flag = bounding_box(curr_img,motion_mask)

        contours, _ = cv2.findContours(motion_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        #choose the largest contour
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]

        flag =False

        if len(contours)!=0:
            for contour in contours:
                area =cv2.contourArea(contour)
                if area >10000 and area<60000:
                # if area >9000:
                    # print(f"area:{area}")
                    flag = True
                    # x, y, w, h = cv2.boundingRect(contour)
                    # cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        else:
            flag = False


        #motion detected
        if flag == True:
            self.counter+=1

            if self.counter>=2:
                print("Motion detected")
                for contour in contours:
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    if y > self.y_point and y+h > self.y_point:
                        cv2.putText(self.cv_image, "Above Horizon", (x+w, y+h), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 0), 2)
                    else:
                        obstacle = True
                        cv2.putText(self.cv_image, "Below Horizon", (x+w, y+h), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 0), 2)
                self.counter=0
        else:
            self.counter = 0
            

        #Display image
        # cv2.imshow('original',curr_img)

        #drawing bounding box
        # cv2.imshow('bounding box',final_img)

        # motion_img = motion(flow)
        # cv2.imshow('motion',motion_mask)
        return obstacle
            
        

def main(args=None):
    rclpy.init(args=args)

    percept = Percept()

    percept.temprun()

    # percept.run()
    percept.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
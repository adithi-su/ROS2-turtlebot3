#!/usr/bin/env python3

import cv2
import time
import rclpy
import random 
import pickle
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from skimage.feature import hog
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import CompressedImage

def thresh_vel(cmd_vel):
    if (cmd_vel.linear.x > 0.2):
        cmd_vel.linear.x = 0.2

    if (cmd_vel.angular.z > 1.2):
        cmd_vel.angular.z = 1.0
    elif (cmd_vel.angular.z < -1.2):
        cmd_vel.angular.z = -1.0

    return cmd_vel

class maze_run(Node):
    def __init__(self):
        super().__init__('maze_run')
        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0
        self.globalPos = Point()
        self.globalAng = 0.0 

        self.obstacle_dist = 0.0
        self.obstacle_left_dist = 0.0 
        self.obstacle_angular = 0.0
        self.obstacle_detected = False

        self.current_angle = 0.0
        self.current_position = 0.0

        with open('/home/burger/ros2_ws/src/lab6/lab6/no_split.pkl', 'rb') as f:
             self.model = pickle.load(f)
        #joblib.load('/home/burger/ros2_ws/src/lab6/lab6/75_25_split.joblib') 

        self.image_subscriber = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            1)

        self.obstacle_range_sub = self.create_subscription(
            Twist,
            'obstacle_data', 
            self.obstacle_callback,
            1)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            1)
        self.odom_sub

        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)     

    def obstacle_callback(self, msg):
        self.obstacle_dist = msg.linear.x
        self.obstacle_left_dist = msg.linear.y
        self.obstacle_angular = msg.angular.z

        if (self.obstacle_dist <= 0.7):
             self.start_predicting = True

        if (self.obstacle_dist <= 0.56): #0.575 last working
            self.obstacle_detected = True
            self.start_predicting = False
            self.stop_robot()
        else:
            self.obstacle_detected = False
            self.start_predicting = False
        
        if (self.obstacle_detected):
            print(f"Obstacle detected at a distance of {self.obstacle_dist}")

        if (self.obstacle_detected == False):
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.1
            cmd_vel = thresh_vel(cmd_vel)
            self.velocity_publisher.publish(cmd_vel)
         
    def odom_callback(self, data):
        self.update_Odometry(data)
        self.current_angle = np.arctan2(np.sin(self.globalAng), np.cos(self.globalAng))
        self.current_position = np.array([self.globalPos.x, self.globalPos.y])

    def image_callback (self, msg):
        self.cv_bridge = CvBridge()
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image_copy = image.copy() 
        resized_image = cv2.resize(image_copy, (240, 320))
             
        if (self.obstacle_detected):
            X = self.process_images(resized_image)
            X = X.reshape(1, -1)
            y_pred = self.model.predict(X)
            print("Predicted Label:", y_pred)
            
            if y_pred == 0:
                 print("Empty Wall\n")
                #  print(f"Turning to self.globalAng + 0.6 = {self.globalAng + 0.6}")
                #  rotate_angle = self.globalAng + 0.6
                #  self.rotate90(rotate_angle, 'A') #prev 0.75
                #  self.obstacle_detected = False
                 rand_no = random.random()
                 print(f"randomnumber generated = {rand_no}")
                 if rand_no < 0.5:
                    rotate_angle = self.globalAng + np.pi/2
                    self.rotate90(rotate_angle, 'A')
                    self.obstacle_detected = False  
                 else:
                    rotate_angle = self.globalAng + np.pi/2
                    self.rotate90(rotate_angle, 'C')
                    self.obstacle_detected = False  
            
            elif y_pred == 1:
                 print("Go Left\n")
                 print(f"Turning to self.globalAng + np.pi/2 = {self.globalAng + np.pi/2}")
                 rotate_angle = self.globalAng + np.pi/2 
                 self.rotate90(rotate_angle, 'A')
                 self.obstacle_detected = False
            
            elif y_pred == 2:
                 print("Go Right\n")
                 print(f"Turning to self.globalAng + np.pi/2 = {self.globalAng + np.pi/2}")
                 rotate_angle = self.globalAng + np.pi/2
                 self.rotate90(rotate_angle, 'C')
                 self.obstacle_detected = False
             
            elif y_pred == 3:
                 print("Do Not Enter\n")
                 print(f"Turning to self.globalAng + np.pi = {self.globalAng + np.pi}")
                 rotate_angle = self.globalAng + np.pi 
                 self.rotate90(rotate_angle, 'C')
                 self.obstacle_detected = False
            
            elif y_pred == 4:
                 print("Stop\n")
                 self.stop_robot()
                 print(f"Turning to self.globalAng + np.pi = {self.globalAng + np.pi}")
                 rotate_angle = self.globalAng + np.pi
                 self.rotate90(rotate_angle, 'C')
                 self.obstacle_detected = False
            
            elif y_pred == 5:
                 print("Goal Reached\n")
                 self.stop_robot()

    def stop_robot(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(cmd_vel_msg)

    def rotate90(self, target_angle, direction):
        current_angle = np.arctan2(np.sin(self.globalAng), np.cos(self.globalAng))
        angle_to_be_turned = target_angle - current_angle

        if(angle_to_be_turned > np.pi):
             angle_to_be_turned -= 2*np.pi 
        if(angle_to_be_turned < -np.pi):
             angle_to_be_turned += 2*np.pi 

        rotate_vel = Twist()

        if (direction == 'A'):
            rotate_vel.angular.z = (angle_to_be_turned / 3)
        else:
            rotate_vel.angular.z = (-(angle_to_be_turned / 3))
        
        self.velocity_publisher.publish(rotate_vel)
        time.sleep(3.1)
        rotate_vel.angular.z = 0.0
        self.velocity_publisher.publish(rotate_vel)
        time.sleep(1)         

    def update_Odometry(self,Odom):
        position = Odom.pose.pose.position
        
        #Orientation uses the quaternion aprametrization.
        #To get the angular position along the z-axis, the following equation is required.
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z
        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        

        #We subtract the initial values
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang
    
        #self.get_logger().info('Transformed global pose is x:{}, y:{}, a:{}'.format(self.globalPos.x,self.globalPos.y,self.globalAng))

    def process_images(self, X):
        image = X
        contour_count = 0
        
        # Blurring 
        image = cv2.GaussianBlur(image, (7, 7), 0)
        image = cv2.GaussianBlur(image, (3, 3), 0)
        
        # Shifting to HSV frame
        hsvFrame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Defining hsv ranges for red, green, blue
        red_lower_1 = np.array([0, 87, 111], np.uint8)      # Lower red range (0-10 degrees)
        red_upper_1 = np.array([10, 255, 255], np.uint8)    # Upper red range (0-10 degrees)
        red_lower_2 = np.array([170, 87, 111], np.uint8)    # Lower red range (340-360 degrees)
        red_upper_2 = np.array([180, 255, 255], np.uint8)   # Upper red range (340-360 degrees)

        # Create masks for both red ranges
        red_mask_1 = cv2.inRange(hsvFrame, red_lower_1, red_upper_1)
        red_mask_2 = cv2.inRange(hsvFrame, red_lower_2, red_upper_2)

        # Combine the masks using bitwise OR
        red_mask = red_mask_1 | red_mask_2

        # Green
        green_lower = np.array([25, 20, 20], np.uint8)
        green_upper = np.array([102, 255, 255], np.uint8)
        green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
        
        # Blue 
        blue_lower = np.array([94, 80, 2], np.uint8)
        blue_upper = np.array([120, 255, 255], np.uint8)
        blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

        kernel = np.ones((5, 5), "uint8")

        # For red color
        red_mask = cv2.dilate(red_mask, kernel)
        res_red = cv2.bitwise_and(image, image, mask=red_mask)

        # For green color
        green_mask = cv2.dilate(green_mask, kernel)
        res_green = cv2.bitwise_and(image, image, mask=green_mask)

        # For blue color
        blue_mask = cv2.dilate(blue_mask, kernel)
        res_blue = cv2.bitwise_and(image, image, mask=blue_mask)

        # Creating contour to track red color
        contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
                contour_count += 1
                cnts = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(cnts)
                
                image_width, image_height = image.shape[1], image.shape[0]
                min_x_threshold = int(0.1 * image_width)    # 10% from left edge
                max_x_threshold = int(0.9 * image_width)    # 10% from right edge
                min_y_threshold = int(0 * image_height)     # 10% from top edge
                max_y_threshold = int(1 * image_height)     # 10% from bottom edge
                
                if (x >= min_x_threshold and (x + w) <= max_x_threshold 
                    and y >= min_y_threshold and (y + h) <= max_y_threshold):
                        # Increase bounding box size and draw rectangle
                        w = int(w * 1.5)  # Increase width by 25%
                        h = int(h * 1.5)  # Increase height by 25%
                        x = max(0, x - int(0.15 * w))
                        y = max(0, y - int(0.15 * h))
                        cv2.rectangle(image, (x, y), (x + w, y + h), color=(0,0,0))
                        image = image[y:y+h, x:x+w]
                        
                        # self.center_x = (2*x + w) / 2
                        # self.object_angle_x = self.center_x * 62.2 / 320
                        # print("Red sign. X coordinate of centre of rectangle =", self.center_x)
                        # print("Angle to centre of the sign =", self.object_angle_x)
                        # self.object_angle_x = np.deg2rad(self.object_angle_x)

        # Creating contour to track green color
        contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
                contour_count += 1
                cnts = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(cnts)
                
                image_width, image_height = image.shape[1], image.shape[0]
                min_x_threshold = int(0.1 * image_width)    # 10% from left edge
                max_x_threshold = int(0.9 * image_width)    # 10% from right edge
                min_y_threshold = int(0 * image_height)     # 10% from top edge
                max_y_threshold = int(1 * image_height)     # 10% from bottom edge
                
                if (x >= min_x_threshold and (x + w) <= max_x_threshold 
                    and y >= min_y_threshold and (y + h) <= max_y_threshold):
                        # Increase bounding box size and draw rectangle
                        w = int(w * 1.5)  # Increase width by 25%
                        h = int(h * 1.5)  # Increase height by 25%
                        x = max(0, x - int(0.15 * w))
                        y = max(0, y - int(0.15 * h))
                        cv2.rectangle(image, (x, y), (x + w, y + h), color=(0,0,0))
                        image = image[y:y+h, x:x+w]
                        
                        # self.center_x = (2*x + w) / 2
                        # self.object_angle_x = self.center_x * 62.2 / 320
                        # print("Green sign. X coordinate of centre of rectangle =", self.center_x)
                        # print("Angle to centre of the sign =", self.object_angle_x)
                        # self.object_angle_x = np.deg2rad(self.object_angle_x)

        # Creating contour to track blue color
        contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
                contour_count += 1
                cnts = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(cnts)

                image_width, image_height = image.shape[1], image.shape[0]
                min_x_threshold = int(0.1 * image_width)    # 10% from left edge
                max_x_threshold = int(0.9 * image_width)    # 10% from right edge
                min_y_threshold = int(0 * image_height)     # 10% from top edge
                max_y_threshold = int(1 * image_height)     # 10% from bottom edge
                
                if (x >= min_x_threshold and (x + w) <= max_x_threshold 
                    and y >= min_y_threshold and (y + h) <= max_y_threshold):
                        # Increase bounding box size and draw rectangle
                        w = int(w * 1.5)  # Increase width by 25%
                        h = int(h * 1.5)  # Increase height by 25%
                        x = max(0, x - int(0.15 * w))
                        y = max(0, y - int(0.15 * h))
                        cv2.rectangle(image, (x, y), (x + w, y + h), color=(0,0,0))
                        image = image[y:y+h, x:x+w]	
                        
                        # self.center_x = (2*x + w) / 2
                        # self.object_angle_x = self.center_x * 62.2 / 320
                        # print("Blue sign. X coordinate of centre of rectangle =", self.center_x)
                        # print("Angle to centre of the sign =", self.object_angle_x)	
                        # self.object_angle_x = np.deg2rad(self.object_angle_x)	

        resized_image = cv2.resize(image, (320, 240))

        grayscale_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)

        # Compute HOG features
        hog_features, hog_image = hog(grayscale_image, orientations=9, pixels_per_cell=(8, 8), cells_per_block=(2, 2), visualize=True)
        
        # Rescale HOG image back to 0-255 range
        hog_image_rescaled = hog_image * 255
        
        # Convert float32 to uint8
        hog_image_rescaled = hog_image_rescaled.astype(np.uint8)

        return hog_image_rescaled
    
def main(args=None):
    rclpy.init(args=args)
    go = maze_run()
    rclpy.spin(go)
    go.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

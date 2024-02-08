#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Point, Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import imutils

class FindObject(Node):
    def __init__(self):
        super().__init__('find_object')
        self.image_publisher = self.create_publisher(CompressedImage, 'processed_image', 10)
        self.image_subscriber = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )
        self.point_publisher = self.create_publisher(Point, 'object_location', 10)
        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        # Convert compressed image message to OpenCV image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Preprocessing
        frame_processed = frame.copy()  # Create a copy of the frame for processing
        frame_processed = imutils.resize(frame_processed, width=500)
        blurred = cv2.GaussianBlur(frame_processed, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # yellow spherical object HSV ranges
        Lowerbound = (29, 86, 6)
        Upperbound = (64, 255, 255)

        # Thresholding
        mask = cv2.inRange(hsv, Lowerbound, Upperbound)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Find contours
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        
        centroid = None

        # If object detected
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)

            # Calculate centroid
            centroid = (M["m10"] / M["m00"], M["m01"] / M["m00"])

            # Publish centroid
            point_msg = Point()
            point_msg.x = centroid[0]
            point_msg.y = centroid[1]
            point_msg.z = 0.0
            self.point_publisher.publish(point_msg)            
            self.get_logger().info("Centroid coordinates - X: {:.2f}, Y: {:.2f}".format(point_msg.x, point_msg.y))

            # Draw circle and centroid on the frame
            #cv2.circle(frame_processed, (int(x), int(y)), int(radius), (0, 0, 255), 2)
            #cv2.circle(frame_processed, centroid, 5, (0, 255, 255), -1)


def main(args=None):
    rclpy.init(args=args)
    find_object_node = FindObject()
    rclpy.spin(find_object_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



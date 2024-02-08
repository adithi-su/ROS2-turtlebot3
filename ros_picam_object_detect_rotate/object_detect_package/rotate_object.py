import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Point, Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import imutils


class RotateRobot(Node):
    def __init__(self):
        super().__init__('rotate_robot')
        self.point_subscription = self.create_subscription(
            Point,
            'object_location',
            self.point_callback,
            10
        )
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 5)

    def point_callback(self, msg):
        # Determine the direction to turn based on object coordinates
        if msg.x < 90:
            # Turn left if the object is on the left side
            vel_msg = Twist()
            vel_msg.angular.z = 0.35
            self.vel_publisher.publish(vel_msg)
        elif msg.x > 230:
            # Turn right if the object is on the right side
            vel_msg = Twist()
            vel_msg.angular.z = -0.35  
            self.vel_publisher.publish(vel_msg)
        else:
            # Stand still if the object is in the center
            vel_msg = Twist()
            vel_msg.angular.z = 0.0
            self.vel_publisher.publish(vel_msg)

        self.get_logger().info('Received cmd_vel message: Angular - {:.2f}'.format(vel_msg.angular.z))


def main(args=None):
    rclpy.init(args=args)
    rotate_robot_node = RotateRobot()
    rclpy.spin(rotate_robot_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

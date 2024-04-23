#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from math import cos, sin, atan2, isnan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        lidar_qos_profile = QoSProfile(
		    reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1
		)

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            lidar_qos_profile)
        
        self.obstacle_data_publisher = self.create_publisher(Twist, 'obstacle_data', 10)

        self.obstacle_twist = Twist()
        self.obstacle_twist.linear.x = 0.0
        self.obstacle_twist.linear.y = 0.0
        self.obstacle_twist.linear.z = 0.0
        self.obstacle_twist.angular.x = 0.0
        self.obstacle_twist.angular.y = 0.0
        self.obstacle_twist.angular.z = 0.0

    def scan_callback(self, msg):
        ranges = msg.ranges 
        distances_left = np.array(ranges[0:20])
        distances_right = np.array(ranges[-20:])
        distances_combined = np.concatenate((distances_left, distances_right))

        valid_indices = np.where(~np.isnan(distances_combined) & (distances_combined >= 0.05) & (distances_combined < 1.5))

        if (np.size(valid_indices) > 0):
                distance_min_index = valid_indices[0][np.argmin(distances_combined[valid_indices])]
                distance_min = distances_combined[distance_min_index]
                if (distance_min_index <= 20):
                        angle = msg.angle_min + (distance_min_index*msg.angle_increment)
                else:
                        angle = msg.angle_max - (distance_min_index*msg.angle_increment)

                print(f"Index = {distance_min_index:.3f}, Distance = {distance_min:.3f}, Angle = {angle:.3f}, index*angle_inc = {distance_min_index*msg.angle_increment:.3f}")

        else:
            distance_min = 5.0
            angle = 0.0
            print(f"Distance = {distance_min:.3f}, Angle = {angle:.3f}")

        size = len(ranges)
        obstacle_data_to_the_left_index = int(size/4)
        sum = []
        obstacle_data_to_the_left = ranges[obstacle_data_to_the_left_index - 3 : obstacle_data_to_the_left_index + 3]
        for i in obstacle_data_to_the_left:
            if (~np.isnan(i)):
                sum.append(i)

        if (len(sum) == 0):
            output_mean = 5.0
        else:
            output_mean = np.mean(sum)
        
        print(f"Distance to the left = {output_mean:.3f}\n")

        self.obstacle_twist.linear.x = float(distance_min)
        self.obstacle_twist.linear.y = float(output_mean)
        self.obstacle_twist.angular.z = float(angle)

        self.obstacle_data_publisher.publish(self.obstacle_twist)

def main(args=None):
    rclpy.init(args=args)
    obstacle_detector_node = ObstacleDetectorNode()
    rclpy.spin(obstacle_detector_node)
    obstacle_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

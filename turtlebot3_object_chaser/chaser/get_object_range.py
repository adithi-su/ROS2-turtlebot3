#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import LaserScan
import numpy as np
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class GetObjectRange(Node):
    def __init__(self):
        super().__init__('get_object_range')

        self.camera_fov_x = 62.2 
        self.image_width = 320    

        self.lidar_range_resolution = 0.1  # Angular resolution of LIDAR scan in degrees

        lidar_qos_profile = QoSProfile(
		    reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1
		)

        self.point_subscriber = self.create_subscription(
            Point,
            'object_location',
            self.point_callback,
            10
        )
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            lidar_qos_profile
        )
        self.range_publisher = self.create_publisher(PointStamped, 'object_range', 10)
        self.object_angle = None

    def point_callback(self, msg):
        print(msg.x, msg.y)
        self.object_angle = (msg.x, msg.y)  

    def lidar_callback(self, msg):
        #print("angle min:", msg.angle_min)
        #print("angle max:", msg.angle_max)
        #print("angle increment: ", msg.angle_increment)

        object_distance = 999.0
        object_angle_x = 999.0
        dist = []

        if(self.object_angle is not None and self.object_angle[0]!=999 and self.object_angle[1] != 999):
            object_angle_x = self.object_angle[0] *  62.2 / 320
            #numElements = (msg.angle_max - msg.angle_min) / msg.angle_increment
            
            if (object_angle_x < 31.1):
                object_angle_x = np.deg2rad(object_angle_x - 31.1)
                lidar_index =  int((object_angle_x - msg.angle_min)/msg.angle_increment)              

                leftIndex = max(0, lidar_index - 5)  
                # angle_ranges = [lidar_index-5, lidar_index-4, lidar_index-3, lidar_index-2, lidar_index-1, lidar_index, lidar_index+1, lidar_index+2, lidar_index+3, lidar_index+4, lidar_index+5]
                for i in range(leftIndex, lidar_index + 5): 
                    if (msg.ranges[i] != float('nan')):
                        dist.append(msg.ranges[i])
                        #print(msg.ranges[-i])
                    else:
                        dist.append(object_distance)
                
                object_distance = np.mean(dist)
            
            else:
                object_angle_x = np.deg2rad(object_angle_x - 31.1)  
                lidar_index =  int((object_angle_x - msg.angle_min)/msg.angle_increment)
                rightIndex = max(lidar_index + 5, len(msg.ranges))

                #angle_ranges = [lidar_index-5, lidar_index-4, lidar_index-3, lidar_index-2, lidar_index-1, lidar_index, lidar_index+1, lidar_index+2, lidar_index+3, lidar_index+4, lidar_index+5]
                for i in range(lidar_index - 5, rightIndex): 
                    if(msg.ranges[-i] != float('nan')):
                        dist.append(msg.ranges[-i])
                        #print(msg.ranges[-i])
                    else:
                        dist.append(object_distance)
                object_distance = np.mean(dist)
                
        range_msg = PointStamped()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.point.x = object_distance
        range_msg.point.y = object_angle_x
        range_msg.point.z = msg.time_increment
        self.range_publisher.publish(range_msg)
        self.get_logger().info("Object distance: {:.2f} and angle: {:.2f} ".format(range_msg.point.x, range_msg.point.y))
            
def main(args=None):
    rclpy.init(args=args)
    get_object_range_node = GetObjectRange()
    rclpy.spin(get_object_range_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

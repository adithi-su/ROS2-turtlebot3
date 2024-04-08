#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
import math
import numpy as np 

class ChaseObject(Node):
    def __init__(self):
        super().__init__('chase_object')
        self.object_range_subscriber = self.create_subscription(
            PointStamped,
            'object_range',
            self.chase_object_callback,
            10
        )
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.angular_kp = 1.5
        self.angular_kd = 0.05

        self.linear_kp = 1.3 
        self.linear_kd = 0.5 
        
        self.prev_angular_error = 0.0
        self.prev_linear_error = 0.0

        self.prev_angular_ctrl = 0
        self.prev_linear_ctrl = 0.04

    def chase_object_callback(self, msg):
        angular_error = msg.point.y 
        linear_error = msg.point.x  

        if linear_error != float('nan'):
            if linear_error < 0.4: 
                linear_ctrl = -0.1 
            else:
                linear_ctrl = self.linear_pid(linear_error)
            angular_ctrl = self.angular_pid(angular_error)
            self.prev_linear_error = linear_ctrl
            self.prev_angular_ctrl = angular_ctrl
        
        else:
            linear_ctrl = self.prev_linear_error 
            angular_ctrl = self.prev_angular_ctrl
        
        velocity_msg = Twist()

        if(msg.point.y == 999.0): #no object detected
            velocity_msg.linear.x = 0.0
            angular_ctrl = 0.0
        elif (linear_error > 0.4 and linear_error < 0.6): #deadzone 
            velocity_msg.linear.x = 0.0
            angular_ctrl = 0.0
        else:
            velocity_msg.linear.x = self.prev_linear_ctrl
        

        velocity_msg.angular.z = angular_ctrl
        self.velocity_publisher.publish(velocity_msg)

        self.get_logger().info("Angular Velocity: {:.2f}, Linear Velocity: {:.2f}".format(angular_ctrl, linear_ctrl))

    def angular_pid(self, error):
        p = self.angular_kp * error
        d = self.angular_kd * (error - self.prev_angular_error) * 9.89
        self.prev_angular_error = error
        ctrl_signal =  - (p + d)
        return ctrl_signal 
    
    def linear_pid(self, error):
        p = self.linear_kp * error
        d = self.linear_kd * (error - 0.4 - self.prev_linear_error) * 9.89
        self.prev_linear_error = error
        ctrl_signal = p + d
        return ctrl_signal

def main(args=None):
    rclpy.init(args=args)
    chase_object_node = ChaseObject()
    rclpy.spin(chase_object_node)
    rclpy.shutdown()

if __name__ == '_main_':
    main()
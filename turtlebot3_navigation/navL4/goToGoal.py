#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

# etc
import numpy as np
import math

def thresh_vel(cmd_vel):

    if (cmd_vel.linear.x > 0.2):
        cmd_vel.linear.x = 0.2

    if (cmd_vel.angular.z > 1.5):
        cmd_vel.angular.z = 1.5
    elif (cmd_vel.angular.z < -1.5):
        cmd_vel.angular.z = -1.5

    return cmd_vel

class goToGoal(Node):
    def __init__(self):
        super().__init__('goToGoal')
        # State (for the update_Odometry code)
        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0
        self.globalPos = Point()
        
        self.rotateCompleted = False
        
        self.startAngle = 0.0
        self.targetAngle = 0.0
        self.obstacle_dist = 0.0
        self.obstacle_left_dist = 0.0 

        self.target_angle = 0.0
        self.prev_angle = 0.0
        self.sleep_done = False
        self.sleep_done_1 = False
        self.sleep_done_2 = False

        self.obstacle_detected = False
        self.obstacle_avoidance_rotation = False
        self.prev_angle_rotation = 0.0

        self.updateWaitCounter = 0

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
        self.odom_sub  # prevent unused variable warning

        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.waypoints = [(1.5, 0.0), (1.5, 1.4), (0.0, 1.4)]  # Read goal locations from file
        self.current_goal_index = 0

    def obstacle_callback(self, msg):
        self.obstacle_dist = msg.linear.x
        self.obstacle_left_dist = msg.linear.y
        self.obstacle_angular = msg.angular.z

    def avoid_obstacle(self):
        avoid_obstacle_vel = Twist()
        
        print(f"Obstacle is towards the left at a distance of {self.obstacle_left_dist}")

        if (self.obstacle_left_dist < 0.5):
            avoid_obstacle_vel.linear.x = 0.2
            avoid_obstacle_vel.angular.z = 0.0
            time.sleep(2.5)
        
        elif (self.obstacle_left_dist > 0.6 and self.obstacle_dist > 0.6 and self.rotateCompleted == False):
            self.rotate90(self.globalAng + np.pi/2, 'A')
            self.rotateCompleted = True
            self.obstacle_detected = False

        return avoid_obstacle_vel
    
    def odom_callback(self, data):
        self.update_Odometry(data)
        # current_position = np.array([self.globalPos.x, self.globalPos.y])
        # goal_position = self.waypoints[self.current_goal_index]
        # distance_to_goal = np.linalg.norm(current_position - goal_position)

        current_angle = np.arctan2(np.sin(self.globalAng), np.cos(self.globalAng))
    
        if (self.current_goal_index == 0):
            current_position = np.array([self.globalPos.x, self.globalPos.y])
            goal_position = self.waypoints[self.current_goal_index]
            distance_to_goal = np.linalg.norm(current_position - goal_position)
            
            if (distance_to_goal <= 0.02):
                print(f"Goal {self.current_goal_index} reached, stop moving forward")
                self.stop_robot()

                if(self.sleep_done == False):
                    time.sleep(2)
                    self.sleep_done = True
                    
                    self.rotate90(np.pi/2, 'A')
                    self.current_goal_index += 1 
                    print(f"Goal: {self.waypoints[self.current_goal_index]}, Current position = {self.globalPos}")
                
            else:
                print(f"Moving to goal #{self.current_goal_index}")
                cmd_vel = Twist()

                current_position = np.array([self.globalPos.x, self.globalPos.y])
                
                desired_angle = np.arctan2((self.waypoints[self.current_goal_index][1] - current_position[1]), 
                                           (self.waypoints[self.current_goal_index][0] - current_position[0]))
                
                angular_error = desired_angle - self.globalAng

                while(angular_error > np.pi):
                    angular_error -= 2*np.pi
                
                while(angular_error < -np.pi):
                    angular_error += 2*np.pi

                cmd_vel.linear.x = 0.15
                cmd_vel.angular.z = 1.2 * angular_error
                cmd_vel = thresh_vel(cmd_vel)
                self.velocity_publisher.publish(cmd_vel)

                print(f"Desired Angle = {desired_angle}, Current Angle = {current_angle}, Current Angular Error = {angular_error}")
        
            print(f"X = {self.globalPos.x:.3f}, Y = {self.globalPos.y:.3f}, Distance to Goal {self.current_goal_index} = {distance_to_goal:.3f}")
            print(f"Current Angle = {current_angle:.3f}, Error = {self.target_angle - self.prev_angle}\n")

        
        if (self.current_goal_index == 1):
            current_position = np.array([self.globalPos.x, self.globalPos.y])
            goal_position = self.waypoints[self.current_goal_index]
            distance_to_goal = np.linalg.norm(current_position - goal_position)
            
            if (distance_to_goal <= 0.075):
                print(f"Goal {self.current_goal_index} reached, stop moving forward")
                self.stop_robot()

                if(self.sleep_done_1 == False):
                    time.sleep(2)
                    self.sleep_done_1 = True
                
                    self.rotate90(np.pi, 'A')
                    self.current_goal_index += 1 
                    self.obstacle_detected = False
                    self.rotateCompleted = False
                    print(f"goal index = {self.current_goal_index}")

                
            else:
                if (self.obstacle_dist <= 0.27):
                    if (self.obstacle_detected == False):
                        print(f"Obstacle detected at a distance of {self.obstacle_dist}. Turning 90 degrees")
                        self.obstacle_detected = True
                        self.rotate90(self.globalAng + np.pi/2, 'C')

                if (self.obstacle_detected == True):
                    print("Moving around obstacle")
                    avoid_obstacle_vel = self.avoid_obstacle()
                    print(f"Obstacle avoidance velocity - Linear = {avoid_obstacle_vel.linear.x}, Angular = {avoid_obstacle_vel.angular.z}")
                    self.velocity_publisher.publish(avoid_obstacle_vel)

                if (self.obstacle_detected == False):
                    print(f"Moving to goal #{self.current_goal_index}")
                    cmd_vel = Twist()

                    current_position = np.array([self.globalPos.x, self.globalPos.y])
                    
                    desired_angle = np.arctan2((self.waypoints[self.current_goal_index][1] - current_position[1]), 
                                            (self.waypoints[self.current_goal_index][0] - current_position[0]))
                    
                    angular_error = desired_angle - self.globalAng

                    while(angular_error > np.pi):
                        angular_error -= 2*np.pi
                    
                    while(angular_error < -np.pi):
                        angular_error += 2*np.pi

                    cmd_vel.linear.x = 0.12
                    cmd_vel.angular.z = 0.6 * angular_error
                    cmd_vel = thresh_vel(cmd_vel)
                    self.velocity_publisher.publish(cmd_vel)

                    print(f"Desired Angle = {desired_angle}, Current Angle = {current_angle}, Current Angular Error = {angular_error}")
        
            print(f"X = {self.globalPos.x:.3f}, Y = {self.globalPos.y:.3f}, Distance to Goal {self.current_goal_index} = {distance_to_goal:.3f}\n")


        if (self.current_goal_index == 2): 
            current_position = np.array([self.globalPos.x, self.globalPos.y])
            goal_position = self.waypoints[self.current_goal_index]
            distance_to_goal = np.linalg.norm(current_position - goal_position)
            print("Enter - goal 2")
            if (distance_to_goal <= 0.2):
                print(f"Goal {self.current_goal_index} reached, stop moving forward")
                self.stop_robot() 
                if(self.sleep_done_2 == False):
                    time.sleep(5)
                    self.sleep_done_2 = True
               
            else:
                if (self.obstacle_dist <= 0.27):
                    if (self.obstacle_detected == False):
                        print(f"Obstacle detected at a distance of {self.obstacle_dist}. Turning 90 degrees")
                        self.obstacle_detected = True
                        self.rotate90(self.globalAng + np.pi/2, 'C')

                if (self.obstacle_detected == True):
                    print("Moving around obstacle")
                    avoid_obstacle_vel = self.avoid_obstacle()
                    print(f"Obstacle avoidance velocity - Linear = {avoid_obstacle_vel.linear.x}, Angular = {avoid_obstacle_vel.angular.z}")
                    self.velocity_publisher.publish(avoid_obstacle_vel)

                if (self.obstacle_detected == False):
                    print(f"Moving to goal #{self.current_goal_index}")
                    cmd_vel = Twist()

                    current_position = np.array([self.globalPos.x, self.globalPos.y])
                    
                    desired_angle = np.arctan2((self.waypoints[self.current_goal_index][1] - current_position[1]), 
                                            (self.waypoints[self.current_goal_index][0] - current_position[0]))
                    
                    angular_error = desired_angle - self.globalAng

                    while(angular_error > np.pi):
                        angular_error -= 2*np.pi
                    
                    while(angular_error < -np.pi):
                        angular_error += 2*np.pi

                    cmd_vel.linear.x = 0.15
                    cmd_vel.angular.z = 1.2 * angular_error
                    cmd_vel = thresh_vel(cmd_vel)
                    self.velocity_publisher.publish(cmd_vel)

                    print(f"Desired Angle = {desired_angle}, Current Angle = {current_angle}, Current Angular Error = {angular_error}")
        
            print(f"X = {self.globalPos.x:.3f}, Y = {self.globalPos.y:.3f}, Distance to Goal {self.current_goal_index} = {distance_to_goal:.3f}\n")


    def stop_robot(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(cmd_vel_msg)

    def rotate90(self, target_angle, direction):
        current_angle = np.arctan2(np.sin(self.globalAng), np.cos(self.globalAng))
        angle_to_be_turned = target_angle - current_angle

        rotate_vel = Twist()

        if (direction == 'A'):
            rotate_vel.angular.z = angle_to_be_turned / 3
        else:
            rotate_vel.angular.z = -(angle_to_be_turned / 3)
        
        self.velocity_publisher.publish(rotate_vel)
        time.sleep(3)
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

    
def main(args=None):
    rclpy.init(args=args)
    go = goToGoal()
    rclpy.spin(go)
    go.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
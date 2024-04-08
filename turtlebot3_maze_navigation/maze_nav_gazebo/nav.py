#! /usr/bin/env python3
import time
import rclpy
import numpy as np
from rclpy.node import Node
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
from geometry_msgs.msg import PoseStamped

class FeedbackSubscriber(Node):
  def __init__(self):
    super().__init__('feedback_subscriber')

    self.current_goal_index = 0

    self.subscription = self.create_subscription(
        NavigateToPose_FeedbackMessage,
        '/navigate_to_pose/_action/feedback',
        self.feedback_callback,
        10)
    
    self.publisher = self.create_publisher(
      PoseStamped, 
      '/goal_pose', 
      10)

    goal_msg = PoseStamped()
    goal_msg.header.frame_id = 'map'
    goal_msg.pose.position.x = 0.1
    goal_msg.pose.position.y = 0.0
    goal_msg.pose.position.z = 0.0
    goal_msg.pose.orientation.w = 1.0

    for i in range(10):
      self.publisher.publish(goal_msg)
      time.sleep(0.2)

  def feedback_callback(self, msg):
    #self.get_logger().info('Navigation Feedback: %s' %msg)
    
    goal_msg0 = PoseStamped()
    goal_msg0.header.frame_id = 'map'
    goal_msg0.pose.position.x = 0.69 
    goal_msg0.pose.position.y = -1.2
    goal_msg0.pose.position.z = 0.0
    goal_msg0.pose.orientation.w = 1.0

    goal_msg1 = PoseStamped()
    goal_msg1.header.frame_id = 'map'
    goal_msg1.pose.position.x = -0.18 
    goal_msg1.pose.position.y = -1.8 
    goal_msg1.pose.position.z = 0.0
    goal_msg1.pose.orientation.w = 1.0

    goal_msg2 = PoseStamped()
    goal_msg2.header.frame_id = 'map'
    goal_msg2.pose.position.x = 0.0 
    goal_msg2.pose.position.y = 0.0 
    goal_msg2.pose.position.z = 0.0
    goal_msg2.pose.orientation.w = 1.0

    currentPosition = np.array([msg.feedback.current_pose.pose.position.x, msg.feedback.current_pose.pose.position.y])
    goal0 = np.array([0.69, -1.2])
    goal1 = np.array([-0.18, -1.8])
    goal2 = np.array([0.0, 0.0])

    if(self.current_goal_index == 0):
      print(f"Going to goal 0. Current Position = {currentPosition}")
      for i in range(10):
        self.publisher.publish(goal_msg0)
        time.sleep(0.01)
      
      if (np.linalg.norm(currentPosition - goal0) < 0.25):
        time.sleep(3)
        self.current_goal_index = self.current_goal_index + 1
        print(f"Goal 0 reached, Current goal index = {self.current_goal_index}")
  
    if(self.current_goal_index == 1):
      print(f"Going to goal 1. Current Position = {currentPosition}")
      for i in range(10):
        self.publisher.publish(goal_msg1)
        time.sleep(0.01)

      if (np.linalg.norm(currentPosition - goal1) < 0.25):
        time.sleep(3)
        self.current_goal_index += 1
        print(f"Goal 0 reached, Current goal index = {self.current_goal_index}")
    
    if(self.current_goal_index == 2):
      print(f"Going to goal 2. Current Position = {currentPosition}")
      for i in range(10):
        self.publisher.publish(goal_msg2)
        time.sleep(0.01)
        
      if (np.linalg.norm(currentPosition - goal2) < 0.25):
        time.sleep(3)
        self.current_goal_index += 1
        print(f"Goal 2 reached, Current goal index = {self.current_goal_index}")

    if(self.current_goal_index >= 3):
      time.sleep(10)

def main(args=None):
    rclpy.init(args=args)
    feedback_subscriber = FeedbackSubscriber()
    rclpy.spin(feedback_subscriber)
    feedback_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
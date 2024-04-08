# turtlebot3_navigation
#### Goal:
To design a set of controllers to make the robot drive through a set of waypoints in the presence of unknown obstacles. 

![image](https://github.com/adithi-su/ROS2/assets/63908022/8e1da87a-9806-4783-98f5-790d2e71e02e)

The blue box is fixed in place while the purple faded box will be placed along the robot's path dynamically.

#### TurtleBot3 Robot Nodes:
- camera_robot_bringup - bringup for LIDAR /scan 
- getObjectRange: Filters LIDAR data to detect ranges and orientation of objects
- goToGoal: Reads goal loactions from a file, implements controllers to navigate without collision 

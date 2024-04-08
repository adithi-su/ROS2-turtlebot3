# turtlebot3_navigation
#### Goal:
To design a set of controllers to make the robot drive through a set of waypoints in the presence of unknown obstacles. 

![image](https://github.com/adithi-su/turtlebot3_navigation/assets/63908022/72f84948-2480-4888-b602-50c2e4958f5a)

The blue box is fixed in place while the purple faded box will be placed along the robot's path dynamically.

#### TurtleBot3 Robot Nodes:
- camera_robot_bringup - bringup for LIDAR /scan 
- getObjectRange: Filters LIDAR data to detect ranges and orientation of objects
- goToGoal: Reads goal loactions from a file, implements controllers to navigate without collision 

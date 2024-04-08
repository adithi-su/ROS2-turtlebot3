#### Goal: 
To make the robot chase a desired object observed in its local coordinate frame. The robot must face the object and maintain a specific distance so as to avoid collisions. 

#### TurtleBot3 Robot Nodes:
- camera_robot_bringup: Initializes the camera and publishes compressed images.
- detect_object: Detects the desired object in the camera image and publishes its location.
- get_object_range: Determines the object's range and angular position using camera and LIDAR data, then publishes this information.
- chase_object: Controls the robot to chase the desired object by subscribing to object range data and publishing velocity commands.


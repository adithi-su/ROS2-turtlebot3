# ROS2
source /opt/ros/humble/setup.bash

(before using ros2 commands)

cd ~/ros2_ws

source install/local_setup.bash

### CLI TOOLS: 

- **Running turtlesim**

terminal tab1: ros2 run turtlesim turtlesim_node

terminal tab2: ros2 run turtlesim turtle_teleop_key

- teleop node controls movements

note: each ROS2 node should be responsible for a single, modular purpose. Nodes can send/receive data from other nodes via topics/services/actions/parameters

node : service client, subscriber/publisher (see image: https://docs.ros.org/en/humble/_images/Nodes-TopicandService.gif)

- use rqt - spawn to create multiple turtles
- use rqt - set pen to change the properties of the line drawn
- to move turtle2, in a new terminal window run: ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
- remapping ex: ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
- node info: ros2 node info <node_name>

**Topics:**

- bus for nodes to exchange messages
- publisher/subscriber model
- not necessarily point-to-point communication; can be one-to-many or many-to-one or many-to-many
- ros2 topic list

add -t to get topic type

- **to see data being published on a topic**: ros2 topic echo <topic_name>
- **to publish data using cli:** ros2 topic pub <topic_name> <msg_type> ‘<args>’

ex: ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}”

- the removal of the `--once` option and the addition of the `--rate 1` option, which tells `ros2 topic pub` to publish the command in a steady stream at 1 Hz.
  
![image](https://github.com/adithi-su/ROS2/assets/63908022/b6edcdd0-93d6-4dca-b81f-0c759851809d)

The graph is depicting how the `/turtlesim` node and the `/teleop_turtle` node are communicating with each other over a topic. The `/teleop_turtle` node is publishing data (the keystrokes you enter to move the turtle around) to the `/turtle1/cmd_vel` topic, and the `/turtlesim` node is subscribed to that topic to receive the data.

**Services**

- call-and-response model
- While topics allow nodes to subscribe to data streams and get continual updates, services only provide data when they are specifically called by a client.
- allowed: multiple service clients using the same service; but only 1 service server for a service
- ros2 service list
    - ex: /clear; /kill; /spawn; /turtle1/set_pen
- ros2 service type <service_name>
- ros2 service call <service_name> <service_type> <arguments>

(optional: <arguments>)

![image](https://github.com/adithi-su/ROS2/assets/63908022/cb8effae-a5da-4e98-aaf5-7131c02bbfb0)

**Parameters:**

- ros2 param list
- Every node has the parameter `use_sim_time`; it’s not unique to turtlesim.

![image](https://github.com/adithi-su/ROS2/assets/63908022/ed0c1a4f-1358-41d6-86c2-8da88894ffd8)

`/turtlesim`’s parameters determine the background color of the turtlesim window using RGB color values.

- ros2 param get <node_name> <parameter_name>
- **to change param value at runtime:** ros2 param set <node_name> <parameter_name> <value>
- **view all of a node’s current param values:** ros2 param dump <node_name>
- **load param from a file to current node:** ros2 param load <node_name> <parameter_file(generally .yaml)>

**Actions**

- for long running tasks
- consist of: goal, feedback, result
- can be cancelled
- client-server model: An “action client” node sends a goal to an “action server” node that acknowledges the goal and returns a stream of feedback and a result.
- ros2 action list
- ros2 action info turtle1/rotate_absolute
- **send action goal from cli**:

`ros2 action send_goal <action_name> <action_type> <values>`

`<values>` need to be in YAML format.

![image](https://github.com/adithi-su/ROS2/assets/63908022/83dc3432-0809-4a8e-ae9d-88a4c39cbc5e)

**Using CLI to launch multiple nodes at once**

ros2 launch turtlesim [multisim.launch.py](http://multisim.launch.py/)

(will run 2 turtlesim nodes)

**Recording and playing back data**

- in a new directory, run: ros2 bag record <topic_name>
- to record multiple topics: ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose
- Before replaying the bag file, enter `Ctrl+C` in the terminal where the teleop is running. Then make sure your turtlesim window is visible so you can see the bag file in action.  run: ros2 bag play subset

CLIENT LIBRARIES:

install colcon: sudo apt install python3-colcon-common-extensions

structure: 

-src 

-build

-install

-log

Deprecated packages and their replacements:

![image](https://github.com/adithi-su/ROS2/assets/63908022/a7d1aff1-a0da-437b-9b91-a7436a14897f)

colcon build --symlink-install

colcon test

source install/setup.bash

**Fixing colcon build failed error:** 

re-post from: https://www.reddit.com/r/ROS/comments/wxkfes/colcon_build_failed_in_example_failed_examples/

1. check setuptools version by running `pip3 list | grep setuptools` if output is `setuptools 58.2.0` then go to 3rd step otherwise follow through 2nd step
2. we need to downgrade the setuptools to 58.2.0 by running this `pip3 install setuptools==58.2.0`
3. from https://blog.csdn.net/tanmx219/article/details/126211384:

go to- ros2_ws/src/examples/rclcpp/topics/minimal_subscriber/CMakeLists.txt

change SHARED to STATIC in this line:

add_library(wait_set_subscriber_library) SHARED

**Packages:**

- create a package (go to ros2_ws/src): ros2 pkg create --build-type ament_cmake --license Apache-2.0 <package_name>
- build (go to ros2_ws): colcon build

To build only the `my_package` package next time, you can run:

`colcon build --packages-select my_package`

- to use the new package, source the setup file: `source install/local_setup.bash`
- use the package: ros2 run my_package my_node

# Assessment 1

## TurtleSim Simulation

**Student:** William Kojumian  
**Professor:** Dr. Sameer Kishore  
**Course:** PDE4430 - Mobile Robotics  
**Academic Year:** 2025-26  
**Submission Date:** November 7, 2025  

### Task 1

- **What is the name of the publisher node?**  
The name of the publisher node is **/teleop_turtle**. This was determined using the command `ros2 node list` while both the simulator and controller were running. The controller node is responsible for _publishing_ movement instructions, and its internal name is **/teleop_turtle**.

- **What is the message type of `/cmd_vel`?**  
The message type of the movement topic (`/turtle1/cmd_vel`) is **`geometry_msgs/msg/Twist`**. First, `ros2 topic list` was used to identify the specific topic for commands. Then, `ros2 topic info /turtle1/cmd_vel` was run to inspect the communication format required for sending linear and angular velocity instructions.

- **What is the frequency of publication of the node publishing on `/cmd_vel`?**  
The publication frequency is the default rate of **10 Hz**. This was verified using `ros2 topic hz /turtle1/cmd_vel` while the turtle was moving. Although the live measurement can fluctuate slightly, the control loop in `turtle_teleop_key` that publishes these commands is hardcoded to **10 Hz**.

- **What is the message type of `/turtle1/pose`?**  
The message type is **`turtlesim/msg/Pose`**. This was confirmed using `ros2 topic info /turtle1/pose`. This topic is published by the simulator (`/turtlesim`) to report the turtle’s state, including its X and Y location and orientation, verifying the specific message structure within the `turtlesim` package.

- **Give an example of a message being published on `/turtle1/color_sensor`.**  
The message type is **`turtlesim/msg/Color`**, with an example output being: **r: 179, g: 184, b: 255**. This was obtained using the command `ros2 topic echo /turtle1/color_sensor`, which shows the live data stream. Each message contains the RGB values (Red, Green, Blue) of the background pixel currently under the turtle, confirming the message structure.

# Package Configuration and `setup.py` Structure

This ROS 2 package contains all nodes required for **Tasks 2 and 3** and is properly configured, with all node entry points defined in [`setup.py`](setup.py), allowing each node to be executed directly using `ros2 run`. The [`setup.py`](setup.py) file specifies the package metadata and registers all executable nodes, so it is important to ensure that this file is correctly set up and the package is built before running the simulator or launching any nodes.

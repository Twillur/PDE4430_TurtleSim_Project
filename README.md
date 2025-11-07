**Assessment 1**

**TurtleSim Simulation**

Student: William Kojumian

Professor: Dr. Sameer Kishore

Course: PDE4430 - Mobile Robotics

Academic Year: 2025-26

Submission Date: November 7, 2025

Task 1

- What is the name of the publisher node?

The name of the publisher node is **/teleop_turtle**. I used the command **ros2 node list** while both the simulator and the controller were running. The controller node is the one _publishing_ (sending) the movement instructions, and its internal name is /**teleop_turtle.**

- What is the message type of /cmd_vel?

The message type of the movement topic (/turtle1/cmd_vel) is **geometry_msgs/msg/Twist**. I first used ros2 topic list to find the specific topic name used for commands: /turtle1/cmd_vel. Then, I ran the command ros2 topic info /turtle1/cmd_vel to directly inspect the communication language (message type) required for sending linear and angular velocity instructions.

- What is the frequency of publication of the node publishing on /cmd_vel?

The publication frequency is the default rate of **10 Hz** (Hertz). I ran the command ros2 topic hz /turtle1/cmd_vel while moving the turtle. This tool measures the rate. Although the live measurement can vary, the hardcoded control loop frequency for the turtle_teleop_key node that publishes these commands is fixed at **10 Hz**.

- What is the message type of /turtle1/pose?

The message type is **turtlesim/msg/Pose**. I used the command ros2 topic info /turtle1/pose. This topic is published by the simulator (/turtlesim) to report the robot's state (its X, Y location, and orientation ). The command confirmed the specific message structure defined within the turtlesim package.

- Give an example of a message being published on /turtle1/color_sensor.

The message is an turtlesim/msg/Color type, with an example output being: **r: 179, g: 184, b: 255**. I used the command ros2 topic echo /turtle1/color_sensor. This command shows the live data stream. I copied one message block, which contains the RGB values (Red, Green, Blue) of the background pixel currently under the turtle, confirming the message type is turtlesim/msg/Color.

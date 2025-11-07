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

This ROS 2 package contains all nodes required for **Tasks 2 and 3** and is properly configured, with all node entry points defined in the [`Setup`](setup.py), allowing each node to be executed directly using `ros2 run`. The [`Setup`](setup.py) file specifies the package metadata and registers all executable nodes, so it is important to ensure that this file is correctly set up and the package is built before running the simulator or launching any nodes.

# Task 2

## Straight Line Node Demonstration

Ensure the [`Straight Line Node`](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/turtlesim_pde4430/straight_line_node.py) file is present with all its program contents.

### Simulation Setup

Open two terminals. In the first terminal, start and maintain the simulation environment using the [`Simulation Runner`](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/Simulation%20Runner).

### Running the Straight Line Node

With the simulation running, execute the [`Straight Line`](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/Straight%20Line%20Runner) node in the second terminal.

### Expected Output

The turtle will move linearly across the simulation window:

![Straight Line Demonstration](https://raw.githubusercontent.com/Twillur/PDE4430_TurtleSim_Project/main/TurtleSim%20Simulation%20Visual/TurtleSim_StraightLine.png)

> **Note:** Keep the simulation terminal active while the node is running.

## Figure '8' Node Demonstration

Ensure the [`Figure '8' Node`](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/turtlesim_pde4430/figure_eight_node.py) file is present.

### Simulation Setup

Use the same simulation terminal as before to maintain the environment.

### Running the [Figure 8 Demonstration](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/Figure%20'8'%20Runner) in the second terminal

Start the [`Figure '8'`]([https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/Figure%208%20Runner](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/Figure%20'8'%20Runner)) node in a new terminal.

### Expected Output

The turtle will follow a figure-eight trajectory:

[![Figure 8 Demonstration](https://raw.githubusercontent.com/Twillur/PDE4430_TurtleSim_Project/main/TurtleSim%20Simulation%20Visual/Turtlesim_NumberEight.png)](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/Figure%20'8'%20Runner)

> **Note:** Ensure the simulation remains active while executing this node.

## Roomba Node Demonstration

Ensure the [`Roomba Node`](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/turtlesim_pde4430/roomba_node.py) file is available.

### Simulation Setup

Continue using the running simulation terminal.

### Running the Roomba Node

Execute the [Roomba](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/Roomba%20Runner) node in a separate terminal.
### Expected Output

The turtle will autonomously navigate the environment in a random cleaning pattern:

![Roomba Demonstration](https://raw.githubusercontent.com/Twillur/PDE4430_TurtleSim_Project/main/TurtleSim%20Simulation%20Visual/TurtleSim_Roomba.png)

> **Note:** Keep the simulation terminal active to observe the full behavior.

## 4 Roombas Node Demonstration

Ensure the [`4 Roombas Node`](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/turtlesim_pde4430/four_roombas_node.py) file is available.

### Simulation Setup

Maintain the simulation running in the first terminal.

### Running the 4 Roombas Node

Execute the [`4 Roombas`](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/Four%20Roombas%20Runner) node in a new terminal.

### Expected Output

Four turtles will navigate the environment simultaneously, each following independent cleaning paths:

![4 Roombas Demonstration](https://raw.githubusercontent.com/Twillur/PDE4430_TurtleSim_Project/main/TurtleSim%20Simulation%20Visual/TurtleSim_4Roombas.png)

> **Note:** Ensure the simulation terminal remains active to observe all turtles.


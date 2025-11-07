# Assessment 1: TurtleSim Simulation

**Student:** William Kojumian  
**Professor:** Dr. Sameer Kishore  
**Course:** PDE4430 - Mobile Robotics  
**Academic Year:** 2025-26  
**Submission Date:** November 7, 2025  

---

## Task 1: Publisher and Message Inspection

- **Publisher Node Name:**  
The primary publisher node is **`/teleop_turtle`**, determined via `ros2 node list` while both the simulator and controller nodes were active. This node is responsible for sending movement instructions to the turtle.

- **Message Type of `/cmd_vel`:**  
The topic `/turtle1/cmd_vel` uses **`geometry_msgs/msg/Twist`**. The topic type was confirmed using `ros2 topic info /turtle1/cmd_vel` after identifying it with `ros2 topic list`. This message format allows the specification of both linear and angular velocities.

- **Publication Frequency of `/cmd_vel`:**  
The default publishing frequency is **10 Hz**, verified with `ros2 topic hz /turtle1/cmd_vel`. Minor fluctuations may occur, but the control loop in `turtle_teleop_key` is configured for 10 Hz.

- **Message Type of `/turtle1/pose`:**  
The `/turtle1/pose` topic uses **`turtlesim/msg/Pose`**, confirmed using `ros2 topic info /turtle1/pose`. This topic provides real-time feedback of the turtle’s X, Y position and orientation.

- **Example Message from `/turtle1/color_sensor`:**  
The `/turtle1/color_sensor` topic is of type **`turtlesim/msg/Color`**, with sample output: **r: 179, g: 184, b: 255**. Messages are obtained using `ros2 topic echo /turtle1/color_sensor` and represent the RGB color of the pixel beneath the turtle.

---

## Package Configuration and `setup.py`

This ROS 2 package includes all nodes required for **Tasks 2 and 3**. The [`setup.py`](setup.py) file registers all executable nodes and ensures they can be run directly with `ros2 run`. Proper configuration of this file is essential before building the package and executing the simulator or nodes.

---

## Task 2: Turtle Movement Demonstrations

### Straight Line Node Demonstration

Ensure the [`Straight Line Node`](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/turtlesim_pde4430/straight_line_node.py) file is present with its complete code.  

#### Simulation Setup

Open two terminals. In the first terminal, start the simulation using the [`Simulation Runner`](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/Simulation%20Runner) to maintain the environment.

#### Running the Node

With the simulation running, execute the [`Straight Line`](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/Straight%20Line%20Runner) node in the second terminal.

#### Expected Output

The turtle moves in a straight linear path:

![Straight Line Demonstration](https://raw.githubusercontent.com/Twillur/PDE4430_TurtleSim_Project/main/TurtleSim%20Simulation%20Visual/TurtleSim_StraightLine.png)

> **Note:** Keep the simulation terminal active while executing the node.

---

### Circle Node Demonstration

Ensure the simulation is active and the [Circle Node](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/turtlesim_pde4430/circle_node.py) code is ready.  

Run the [Circle command](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/Circle%20Runner) in a second terminal.  

The turtle will follow a circular trajectory:

![Circle Node Demonstration](https://raw.githubusercontent.com/Twillur/PDE4430_TurtleSim_Project/main/TurtleSim%20Simulation%20Visual/Turtlesim_ClassCircle.png)

---

### Figure '8' Node Demonstration

Ensure the [`Figure '8' Node`](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/turtlesim_pde4430/figure_eight_node.py) is available.  

Run the [`Figure '8'`](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/Figure%20'8'%20Runner) node in a second terminal while the simulation remains active.

The turtle follows a figure-eight trajectory:

![Figure 8 Demonstration](https://raw.githubusercontent.com/Twillur/PDE4430_TurtleSim_Project/main/TurtleSim%20Simulation%20Visual/Turtlesim_NumberEight.png)

> **Note:** Keep the simulation terminal running to observe the complete behavior.

---

### Roomba Node Demonstration

Ensure the [`Roomba Node`](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/turtlesim_pde4430/roomba_node.py) file is present.  

Run the [Roomba](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/Roomba%20Runner) node in a second terminal.  

The turtle will autonomously navigate in a random cleaning pattern:

![Roomba Demonstration](https://raw.githubusercontent.com/Twillur/PDE4430_TurtleSim_Project/main/TurtleSim%20Simulation%20Visual/Turtlesim_1Roomba.png)

> **Note:** Maintain the simulation terminal active for full observation.

---

### 4 Roombas Node Demonstration

Ensure the [4 Roombas Node](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/turtlesim_pde4430/multi_turtle_node.py) is available.  

Run the [4 Roombas](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/4RoombasRunner) node in a new terminal.  

Four turtles navigate independently, simulating multiple agents cleaning simultaneously:

![4 Roombas Demonstration](https://raw.githubusercontent.com/Twillur/PDE4430_TurtleSim_Project/main/TurtleSim%20Simulation%20Visual/Turtlesim_4Roombas.png)

> **Note:** Keep the simulation terminal active to observe all turtles.

---

## Task 3: User Input for Turtle Control

### User Inputting Linear Speed and Angular Values

#### Simulation Setup

Ensure the simulation is running using the [Simulator](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/Simulation%20Runner) in the first terminal.

#### Running the Drive User Input Node

Open a second terminal and execute the [Drive User Input Node](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/turtlesim_pde4430/drive_user_input.py).  

After launching, input the **linear speed**, press Enter, then input the **angular velocity** and press Enter. The turtle will move according to these values.

Use the [Drive User Input Runner](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/Linear%26Angular%20Input) to initiate the node.

#### Example Outputs

- **Linear Speed = 1, Angular = 1**

![TurtleSim 1 Linear 1 Angular](https://raw.githubusercontent.com/Twillur/PDE4430_TurtleSim_Project/main/TurtleSim%20Simulation%20Visual/Turtlesim_1Linear1Angular.png)

- **Linear Speed = 1, Angular = 0**

![TurtleSim 1 Linear 0 Angular](https://raw.githubusercontent.com/Twillur/PDE4430_TurtleSim_Project/main/TurtleSim%20Simulation%20Visual/Turtlesim_1Linear0Angular.png)

---

### User Inputting X and Y Coordinates

#### Simulation Setup

Ensure the simulation is active using the [Simulator](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/Simulation%20Runner).  

#### Running the Coordinate Navigation Node

Have the [Coordinate Navigation Node](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/turtlesim_pde4430/coordinate_navigation.py) ready. Run the [Coordinate Navigator](https://github.com/Twillur/PDE4430_TurtleSim_Project/blob/main/XYCoordinateInput) in a second terminal.  

Enter **X** and **Y** values between 0 and 11 to specify the target location. For example, `X = 10`, `Y = 10`.

#### Expected Outcome

The turtle navigates to the specified coordinates:

![Coordinate Navigation Demonstration](https://raw.githubusercontent.com/Twillur/PDE4430_TurtleSim_Project/main/TurtleSim%20Simulation%20Visual/Turtlesim_Coordinate%20Navigation.png)

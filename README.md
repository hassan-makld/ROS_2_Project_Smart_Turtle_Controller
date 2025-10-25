# Smart Turtle Controller – ROS 2 Project

This project is the **final hands-on exercise** from the *ROS 2 Humble Foundations*.  
It demonstrates how to design a modular, scalable, and fully integrated ROS 2 system using **Turtlesim**.

---

## Overview

The **Smart Turtle Controller** system controls the turtle’s movement based on configurable parameters and custom messages.  
It integrates **topics, services, parameters, and launch files** to simulate a real-world ROS 2 application.

You will be able to:
- Control the turtle’s movement pattern (`circle`, `square`, `spiral`, etc.)
- Adjust its speed through parameters
- Automatically reset and clear the simulation environment every 10 seconds
- Launch all nodes in one command

---

## System Architecture

The project is divided into **three ROS 2 packages**, each representing a specific layer of the system.

### `smart_turtle_interface` – Message Definition Layer
Defines the **custom message** used for communication between nodes.

**File:** `msg/TurtleCommand.msg`
```plaintext
string command
float32 speed
```

### smart_turtle_py – Logic and Control Layer

Contains the core nodes that control the turtle.

- command_publisher.py

Reads parameters from the YAML file (turtle_param.yaml)

Creates and publishes a TurtleCommand message every second on topic /smart_turtle/command

- turtle_controller.py

Subscribes to `/smart_turtle/` command

Converts the received message into a Twist velocity command

Publishes it to `/turtle1/cmd_vel` to control turtle motion

- reset_node.py

Every 10 seconds, calls `/clear` and `/reset` services from Turtlesim

Clears the screen and resets the turtle’s position
---

### smart_turtle_bringup – Integration and Launch Layer

Responsible for integrating all components and launching the full system.

### Files:

launch/smart_turtle.launch.py → launches all nodes and turtlesim

config/turtle_param.yaml → contains pattern and speed parameters

Example YAML:
```
command_publisher:
  ros__parameters:
    pattern: square
    speed: 2.0
```
go back and build
```
colcon build
```

```
source install/setup.bash
```

```
ros2 launch smart_turtle_bringup smart_turtle.launch.py
```


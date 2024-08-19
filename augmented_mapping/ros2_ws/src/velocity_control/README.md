# UAV Velocity Control Nodes


This README provides an overview of the UAV velocity control nodes available in this package. Each node serves a specific purpose and can be run according to your needs for UAV operation and simulation in Gazebo. 


## Nodes Overview


### 1. `velocity_manual_node`


- **Purpose**: This node receives position velocity commands from FUEL and publishes the corresponding velocity. It maintains the current position with a velocity of (0, 0, 0) before the FUEL simulation starts.
- **Use Case**: This node is ideal for manually flying the UAV to a desired location in Gazebo using QGroundControl before starting the FUEL simulation.
- **How to Use**: 
  - Run this node when you need manual control over the UAV's position before initiating the FUEL simulation.


### 2. `velocity_fuel_node`


- **Purpose**: This node commands the UAV to lift off to a height of 4.0 meters and hover until further input is received from the FUEL system.
- **Use Case**: Use this node to prepare the UAV for automated control via FUEL after it reaches a stable hovering position.
- **How to Use**: 
  - Start this node to automatically lift off the UAV and await commands from FUEL.


### 3. `velocity_control_node`


- **Purpose**: This node manages the UAV's takeoff to a specified height, moves it to a desired position, and then waits for input from the FUEL command.
- **Use Case**: Utilize this node when a sequence of actions is needed before handing over control to FUEL, such as taking off and reaching a predefined location.
- **How to Use**: 
  - Execute this node to perform a controlled takeoff and positioning before engaging FUEL for further operations.


## Running the Nodes


To run any of these nodes, use the following command in your terminal:


```bash
ros2 run velocity_control <node_name>

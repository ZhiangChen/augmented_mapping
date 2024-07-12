# augmented_mapping
# Docker Setup and Usage for [Repository Name]

This repository contains Dockerfiles for creating and running Docker containers for various packages including [FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL), [Fast Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner), and [RTAB-Map](https://github.com/introlab/rtabmap_ros/tree/ros2#rtabmap_ros).
++++

## Prerequisites

Before you begin, ensure you have the following installed on your system:

- Git
- Docker (see installation instructions below)

## Installing Docker

Follow these steps to install Docker on your Ubuntu system:

1. Remove any old versions of Docker:

    ```bash
    sudo apt-get remove docker docker-engine docker.io containerd runc
    ```

2. Update your package list:

    ```bash
    sudo apt-get update
    ```

3. Install required packages:

    ```bash
    sudo apt-get install \
        ca-certificates \
        curl \
        gnupg \
        lsb-release
    ```

4. Add Docker’s official GPG key:

    ```bash
    sudo mkdir -p /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    ```

5. Set up the Docker repository:

    ```bash
    echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
      $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    ```

6. Update your package list again:

    ```bash
    sudo apt-get update
    ```

7. Install Docker:

    ```bash
    sudo apt-get install docker-ce docker-ce-cli containerd.io docker-compose-plugin
    ```

8. Verify Docker installation and version (make sure the version is 27.0.3):

    ```bash
    sudo docker --version
    ```

## Building the Docker Image

To build the Docker image for this project, follow these steps:

1. Clone the Git repository:

    ```bash
    git clone https://github.com/ZhiangChen/augmented_mapping
    ```

2. Navigate to the cloned repository:

    ```bash
    cd docker/fuel_docker
    ```

    ```bash
    cd docker/fast_planner_docker
    ```

    ```bash
    cd docker/rtabmap_docker
    ```

3. Build the Docker image:

    ```bash
    sudo docker build -t your_image_name .
    ```

## Running the Docker Container

To run the Docker container with X11 forwarding, follow these steps:

1. **Install and Configure X11 on Host**:
   Ensure you have an X11 server running on your host. Most Linux distributions come with X11 pre-installed. You might need to install `xauth`.

    ```bash
    sudo apt-get install xauth
    ```

2. **Allow Docker to Use Host's X Server**:
   Run the Docker container with X11 forwarding. You need to share the X11 socket and set the DISPLAY environment variable.

    ```bash
    xhost +local:docker
    docker run -it --rm \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        your_image_name
    ```

3. **Running the Same Docker Image in a Different Terminal**:
   Open another terminal and execute the following commands:

    ```bash
    docker ps  # Get the container name
    docker exec -it <container_name> bash
    ```

## Running FUEL in Docker Container Refer to [FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL) for more detailed information. 

1. **Run rviz**: 
```bash 
source devel/setup.bash && roslaunch exploration_manager rviz.launch 
``` 
2. **Run simulation**: 
```bash 
source devel/setup.bash && roslaunch exploration_manager exploration.launch 
``` 

## Running Fast Planner in Docker Container Refer to [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) for more detailed information. 

1. **Start visualization**: 
```bash 
source devel/setup.bash && roslaunch plan_manage rviz.launch 
``` 

2. **Start simulation**: 
```bash 
source devel/setup.bash && roslaunch plan_manage kino_replan.launch 
``` 

## Running RTAB-Map (Example with TurtleBot3) Refer to [RTAB-Map ROS](https://github.com/introlab/rtabmap_ros/tree/ros2#rtabmap_ros) for running RTAB-Map in ROS 2. 

1. **Launch TurtleBot3 Simulator**: 
```bash 
export TURTLEBOT3_MODEL=waffle 
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 
export TURTLEBOT3_MODEL=waffle 
ros2 run turtlebot3_teleop teleop_keyboard 
``` 
2. **Launch RTAB-Map**: 
```bash 
ros2 launch rtabmap_demos turtlebot3_scan.launch.py 
# OR with rtabmap.launch.py ros2 launch rtabmap_launch rtabmap.launch.py \ visual_odometry:=false \ 
frame_id:=base_footprint \ 
subscribe_scan:=true depth:=false \ 
approx_sync:=true \ 
odom_topic:=/odom \ 
scan_topic:=/scan \ 
qos:=2 \ 
args:="-d --RGBD/NeighborLinkRefining true --Reg/Strategy 1" \ 
use_sim_time:=true \ 
rviz:=true 
``` 

3. **Launch Navigation** (nav2_bringup package should be installed): 
```bash 
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True 
ros2 launch nav2_bringup rviz_launch.py
```


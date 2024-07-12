# UAV System for Mapping Precariously Balanced Rocks (PBRs)


## Package Overview


This project aims to develop a UAV system capable of approaching, detecting, identifying, and mapping Precariously Balanced Rocks (PBRs) using advanced exploration algorithms. The mapping process is divided into three main stages:


1. **Approach Stage**: The UAV plans and follows a path to the PBR using the Fast Planner algorithm.
2. **Detection and Identification Stage**: The UAV detects rocks using YOLOv8 and identifies the target PBR with a recognition algorithm.
3. **Mapping Stage**: The UAV maps the identified PBR using the FUEL exploration algorithm and RTAB-Map.


## Dependencies

### Packages
- **PX4_msgs**: installation instructions below

### Docker Containers


- **Fast Planner**: Path planning algorithm.
- **YOLOv8**: Object detection algorithm.
- **FUEL**: Fast UAV Exploration Algorithm.
- **RTAB-Map**: Simultaneous localization and mapping library.


#### Docker Containers


- **ROS Noetic**: Used for running ROS1 packages.
- **ROS2 Humble**: Used for running ROS2 packages.


## Installation


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
    git clone repository_url
    ```


2. Navigate to the cloned repository:


    ```bash
    cd repository_name
    ```


3. Build the Docker image:


    ```bash
    sudo docker build -t your_image_name .
    ```


## Running the FUEL Example


To run the Docker container, use the following commands:


1. Run the Docker container:


    ```bash
    xhost +local:docker
    docker run -it --rm \
        -e DISPLAY=$DISPLAY \
        -e LIBGL_ALWAYS_SOFTWARE=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        your_image_name
    ```


2. Inside the Docker container, start `roscore`:


    ```bash
    roscore
    ```


3. Open a new terminal and run the ROS1-ROS2 bridge:


    ```bash
    source /opt/ros/humble/setup.bash
    source ~/ros-humble-ros1-bridge/install/local_setup.bash
    ros2 run ros1_bridge dynamic_bridge
    ```


4. Get the Docker container name:


    ```bash
    docker ps
    ```


5. Execute the exploration manager commands in the Docker container:


    ```bash
    docker exec -it <container_name> bash
    source devel/setup.bash && roslaunch exploration_manager rviz.launch
    ```


6. Run the exploration launch file:


    ```bash
    docker exec -it <container_name> bash
    source devel/setup.bash && roslaunch exploration_manager exploration.launch
    ```


7. Open a new terminal and start the Micro-XRCE-DDS-Agent:


    ```bash
    cd Micro-XRCE-DDS-Agent
    MicroXRCEAgent udp4 -p 8888
    ```


8. Open another terminal and start PX4 SITL with Gazebo:


    ```bash
    cd PX4-Autopilot/
    make px4_sitl gz_x500
    ```


9. Finally, build and run the velocity control node:


    ```bash
    cd ros2_ws/src
    git clone https://github.com/PX4/px4_msgs
    cd ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build
    source install/local_setup.bash
    ros2 run velocity_control velocity_control_node
    ```


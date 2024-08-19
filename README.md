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

## Training and Running YOLOv7 Model

### Training YOLOv7 Model

Follow instructions on YOLOv7 Github page: https://github.com/WongKinYiu/yolov7

### Running YOLOv7 Model

To run YOLOv7 ROS node, first make a new workspace and copy contents of yolov7_ws:
    ```bash
    cd \
    mkdir yolov7_ws \
    cd yolov7_ws \
    git clone https://github.com/yunhajo03/yolov7-ros.git \
    ```

Next, launch yolov7:
    ```bash
    ros2 launch yolov7_ros yolov7.launch
    ```

Parameters can be modified in launch/yolov7.launch file.

## Running the FUEL-Based Exploration Algorithm


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

    To run with your own model:
    
    ```bash
    cd .gz
    mkdir models
    cd models
    mkdir model_name
    ```
    
    Run the commands above and place model.sdf, model.config, and meshes file.

    Now run 

    ```bash
    cd PX4-Autopilot/Tool/simulation/gz/worlds
    ```

    and create a new world.config file under the directory that contains your new model.

    Afterwards, 

    ```bash
    export GZ_SIM_RESOURCE_PATH="$pwd/.gz/models"
    PX4_GZ_WORLD=model_name make px4_sitl gz_x500_depth
    ```

9. Install and run ROS-gazebo bridge 

    ```bash
    sudo apt-get install ros-humble-ros-gzgarden
    ros2 run ros_gz_image image_bridge /camera /depth_camera
    ```
    
    Replace with your version of ROS and Gazebo. Tested on ROS Humble and Gazebo Garden.

10. Run ROS-Gazebo bridge for clock
    ```bash
    ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
    ```

11. Build and launch the perception node:


    ```bash
    cd ros2_ws/src
    git clone https://github.com/PX4/px4_msgs
    cd ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build
    source install/local_setup.bash
    ros2 launch perception perception_node
    ```

12. Build and run the velocity control node:


    ```bash
    cd ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build
    source install/local_setup.bash
    ros2 run velocity_control velocity_control_node
    ```

13. Run rtabmap:

    ```bash
    sudo apt update
    sudo apt install ros-humble-rtabmap
    ```

    ```bash
    ros2 launch rtabmap_launch rtabmap.launch.py \
    frame_id:=x500_depth_0/OakD-Lite/base_link/IMX214 \
    args:="-d" \
    use_sim_time:=true \
    rgb_topic:=/camera \
    depth_topic:=/depth_camera \
    camera_info_topic:=/camera_info \
    approx_sync:=true \
    qos:=2
    ```

## Parameter Details

### Map and Bounding Box

- **Map Size**: Adjust `map_size_x`, `map_size_y`, and `map_size_z` to match your simulation environment's dimensions.
- **Bounding Box**: Set `box_min_x`, `box_min_y`, `box_min_z`, `box_max_x`, `box_max_y`, and `box_max_z` to define the operational area within the map. Ensure these values are smaller than the map size to prevent errors.
- **Note about bounding box**: UAV will only explore the space defined by the bouding box. It is recommended that the map size is set much larger than the bounding box, as setting the map size close to bounding box often results in

These parameters are located in `FUEL/fuel_planner/exploration_manager/launch/exploration.launch`.

### Camera Calibration

- **Camera Parameters**: Tune `Cx`, `Cy`, `Fx`, and `Fy` according to the X_500 depth camera specifications used in your Gazebo simulation to achieve accurate depth perception.

Camera calibration settings can also be found in `FUEL/fuel_planner/exploration_manager/launch/exploration.launch`.

### Motion Control

- **Velocity and Acceleration**: Set `max_vel` and `max_acc` to 0.4 for steady and controlled UAV movement.

The motion control parameters are located in `FUEL/fuel_planner/exploration_manager/launch/exploration.launch`.

### Image Processing

- **Depth Scaling Factor**: Use a `k_depth_scaling_factor` of 3431.15 in `Algorithm.xml` to scale pixel values accurately. Alternatively, a value of 1000.0 can be used.

The `Algorithm.xml` file is located in `FUEL/fuel_planner/exploration_manager/launch`.

### Yaw Control

- **Yaw Acceleration (ydd_)**: Limit the yaw acceleration to avoid sudden changes.
- **Optimization Weights (w_dir_)**: Increase this value to reduce sharp yaw changes.
- **Relaxation Time**: Use a higher `relax_time` to achieve smoother yaw transitions.
- **Position Trajectory Duration (yd_)**: Adjust to set the minimum duration for position trajectories.

## Recommended Settings

The following values have been determined to provide the best results for our simulation:

- **exploration/yd**: 50
- **exploration/ydd**: 55
- **exploration/w_dir**: 2.5
- **exploration/relax_time**: 2.0

These settings help minimize abrupt yaw changes while maintaining efficient exploration times. Be cautious not to reduce yaw rates excessively, as this may lead to FUEL exploration failures.

### PID Velocity Controllers 
PID controllers are used to maintain precise control over the UAV's velocity in various directions. The parameters for these controllers are as follows: 
- **PID for X-axis (self.pid_x = PID(2.0, 0.1, 0.0))**
- **PID for Y-axis (self.pid_y = PID(0.8, 0.1, 0.0))**
- **PID for Z-axis (self.pid_z = PID(0.8, 0.1, 0.0))**
- **PID for Yaw (self.pid_yaw = PID(2.0, 0.1, 0.0))**

#### Tuning Guidelines 
- **Proportional (P)**: This term responds to the current error. Increasing the P value typically results in a faster response but can cause overshoot or oscillations if set too high. 
- **Integral (I)**: This term responds to accumulated errors over time. It helps eliminate steady-state errors. If the I value is too high, it can cause the system to be slow or oscillate. 
- **Derivative (D)**: This term responds to the rate of change of the error, providing a damping effect. It can help stabilize the system but may lead to instability if set too high. 

Carefully tuning these parameters will help achieve smooth and accurate UAV operation in your Gazebo simulations. Adjust settings as needed to fit specific simulation requirements or environments.


## Issues

### FUEL unable to find any frontiers

If FUEL package fails with error "No coverable frontiers" when triggered, then modify parameters in file fuel_planner/exploration_manager/launch/exploration.launch. Box_min_x, box_min_y, Box_min_z, Box_max_x, Box_max_y, Box_max_z values need to be changed.
# UAV System for Mapping Precariously Balanced Rocks (PBRs)

## Package Overview

This project aims to develop a UAV system capable of approaching, detecting, identifying, and mapping Precariously Balanced Rocks (PBRs) using advanced exploration algorithms. The mapping process is divided into three main stages:

1. **Approach Stage**: The UAV plans and follows a path to the PBR using the Fast Planner algorithm.
2. **Detection and Identification Stage**: The UAV detects rocks using YOLOv8 and identifies the target PBR with a recognition algorithm.
3. **Mapping Stage**: The UAV maps the identified PBR using the FUEL exploration algorithm and RTAB-Map.

## Dependencies

### Packages and Docker Containers

- **Fast Planner**: Path planning algorithm.
- **YOLOv8**: Object detection algorithm.
- **FUEL**: Fast UAV Exploration Algorithm.
- **RTAB-Map**: Simultaneous localization and mapping library.

#### Docker Containers

- **ROS Noetic**: Used for running ROS1 packages.
- **ROS2 Humble**: Used for running ROS2 packages.

## Installation

### Prerequisites

- Docker and Docker Compose installed on your system.

### Clone the Repository

```sh
git clone https://github.com/ZhiangChen/augmented_mapping.git
cd docker
```
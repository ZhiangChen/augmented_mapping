FROM ros:noetic

# Use an alternative mirror for the apt-get update
RUN sed -i 's/archive.ubuntu.com/mirrors.edge.kernel.org/g' /etc/apt/sources.list

# Install required packages
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-noetic-ros-base \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    git \
    libeigen3-dev \
    ros-noetic-pcl-conversions \
    libopencv-dev \
    ros-noetic-cv-bridge \
    libarmadillo-dev \
    ros-noetic-nlopt \
    ros-noetic-tf \
    ros-noetic-pcl-ros \
    ros-noetic-laser-geometry \
    ros-noetic-rviz \
    elfutils \
    libdw-dev \
    libxkbcommon-x11-0 \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep if not already initialized
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && \
    rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Remove nlopt package installed
RUN apt-get remove -y ros-noetic-nlopt

# Install nlopt from the source
RUN git clone https://github.com/stevengj/nlopt.git /root/nlopt && \ 
    cd /root/nlopt && \ 
    mkdir build && cd build && \ 
    cmake .. && \ 
    make && \ 
    make install


# Clone the FUEL repository
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    mkdir -p /root/catkin_ws/src && \
    cd /root/catkin_ws/src && \
    git clone -b px4_integration2 https://github.com/yunhajo/FUEL.git"
    # git clone https://github.com/aaryanmurgunde/Inspection-with-PX.git"

# COPY ../../../FUEL_PX4 /root/catkin_ws/src

# Create a Python script to add set(CMAKE_CXX_STANDARD 14) to all CMakeLists.txt files
RUN cat <<EOF > /root/modify_cmakelists.py
import os

for root, dirs, files in os.walk("/root/catkin_ws/src"):
    for file in files:
        if file == "CMakeLists.txt":
            file_path = os.path.join(root, file)
            with open(file_path, "r") as f:
                lines = f.readlines()
            with open(file_path, "w") as f:
                f.write("set(CMAKE_CXX_STANDARD 14)\\n")
                f.writelines(lines)
EOF

# Run the Python script to modify CMakeLists.txt files
RUN python3 /root/modify_cmakelists.py

# Build the FUEL package
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make"

# Set the entrypoint to bash
ENTRYPOINT ["/bin/bash"]

# Set the working directory to /root/catkin_ws
WORKDIR /root/catkin_ws
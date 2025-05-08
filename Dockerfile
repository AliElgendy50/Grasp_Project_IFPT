FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# 1. Install essential packages and ROS dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    cmake \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-rviz2 \
    ros-humble-diagnostic-updater \
    ros-humble-launch-pytest \
    ros-humble-xacro \
    python3-tqdm \
    python3-requests \
    software-properties-common \
    curl \
    gnupg \
    apt-transport-https \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# 2. Add RealSense repository (alternative method)
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE && \
    echo "deb http://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" > /etc/apt/sources.list.d/realsense.list

# 3. Install RealSense packages (excluding DKMS)
RUN apt-get update && apt-get install -y \
    librealsense2-utils \
    librealsense2-dev \
    && rm -rf /var/lib/apt/lists/*

# 4. Initialize rosdep (skip init if already done)
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && \
    rosdep update

# 5. Create and setup ROS2 workspace
WORKDIR /home/ros_ws/src
RUN git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master

# 6. Install ROS2 dependencies (with --skip-keys for librealsense2)
WORKDIR /home/ros_ws
RUN rosdep install --from-paths src --ignore-src -y --skip-keys "librealsense2"

# 7. Install remaining Python dependencies via pip
RUN pip install tqdm requests

# 8. Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"


# 9. Install OpenVINO using pip from PyPI
RUN pip install openvino



# 10. Source OpenVINO environment variables in new shells
RUN echo "source /opt/intel/openvino/bin/setupvars.sh" >> ~/.bashrc


# 11. Install dependencies including Eigen
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# 12. Install PCL (Point Cloud Library)
RUN apt-get update && apt-get install -y \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*



# 12. Source ROS setup in new shells
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/ros_ws/install/setup.bash" >> ~/.bashrc

# Default shell
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /home/ros_ws/install/setup.bash && exec bash"]

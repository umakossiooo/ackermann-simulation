ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive
ENV COLCON_WS=/root/colcon_ws
ENV COLCON_WS_SRC=/root/colcon_ws/src
ENV PYTHONWARNINGS="ignore:setup.py install is deprecated::setuptools.command.install"
ENV XDG_RUNTIME_DIR=/tmp

ARG GZ_VERSION=harmonic

# Update and install base packages
RUN apt-get update && apt-get install -y \
    wget git curl gnupg2 lsb-release ca-certificates \
    build-essential python3-pip \
    mesa-utils libglvnd0 vulkan-tools libx11-dev libxext-dev libgl1 libegl1 \
    && rm -rf /var/lib/apt/lists/*

# Add Gazebo key and repo
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/gz-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/gz-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gz.list && \
    apt-get update && apt-get install -y \
    gz-${GZ_VERSION} \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-actuator-msgs \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-nav2* \
    ros-${ROS_DISTRO}-behaviortree-cpp-v3 \
    ros-${ROS_DISTRO}-sdformat-urdf \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-builtin-interfaces \
    && rm -rf /var/lib/apt/lists/*

# Clone and build your project
RUN mkdir -p ${COLCON_WS_SRC} && \
    git clone https://github.com/alitekes1/ackermann-vehicle-gzsim-ros2.git ${COLCON_WS_SRC}/ackermann-vehicle-gzsim-ros2 && \
    cd ${COLCON_WS} && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build

# Environment setup
ENV GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:${COLCON_WS_SRC}/ackermann-vehicle-gzsim-ros2
ENV ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${COLCON_WS_SRC}/ackermann-vehicle-gzsim-ros2

# Auto source on shell startup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/colcon_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]

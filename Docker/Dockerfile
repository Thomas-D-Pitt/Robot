ARG ROS_VERSION="jazzy"

FROM ros:${ROS_VERSION}-ros-core
ARG ROS_VERSION

# Install additional tools
RUN --mount=target=/var/lib/apt/lists,type=cache,sharing=locked \
    --mount=target=/var/cache/apt,type=cache,sharing=locked \
    apt-get update && apt-get install -y \
    sudo \
    git \
    x11-apps \
    libgraphicsmagick++-dev \
    libxsimd-dev \
    iputils-ping \
    python3-pip \
    python3-zmq \
    # ros-${ROS_VERSION}-navigation2 \
    # ros-${ROS_VERSION}-nav2-bringup \
    # ros-${ROS_VERSION}-nav2-minimal-tb* \
    ros-${ROS_VERSION}-rviz2 \
    ros-${ROS_VERSION}-ament-cmake \
    ros-${ROS_VERSION}-geometry-msgs \
    ros-${ROS_VERSION}-geographic-msgs \
    ros-${ROS_VERSION}-bond \
    ros-${ROS_VERSION}-bondcpp \
    ros-${ROS_VERSION}-test-msgs \
    ros-${ROS_VERSION}-behaviortree-cpp \
    ros-${ROS_VERSION}-diagnostic-updater \
    ros-${ROS_VERSION}-tf2-sensor-msgs \
    ros-${ROS_VERSION}-angles \
    ros-${ROS_VERSION}-cv-bridge \
    ros-${ROS_VERSION}-joy \
    ros-${ROS_VERSION}-teleop-twist-joy \
    ros-${ROS_VERSION}-demo-nodes-cpp \
    python3-colcon-common-extensions \
    python3-rosdep \ 
    python3-pygame \
    python3-rpi-lgpio
    # rm -rf /var/lib/apt/lists/*  # Clean up to reduce image size



# Create a non-root user
# RUN groupadd -g ${GROUP_ID} ${USER_NAME} && \
#     useradd -m -u ${USER_ID} -g ${GROUP_ID} ${USER_NAME} && \
#     echo "${USER_NAME}:${USER_NAME}" | chpasswd && \
#     adduser ${USER_NAME} sudo

# delete the password for the ubuntu user
RUN passwd -d ubuntu 
# Switch to non-root user
RUN groupadd gpio
RUN usermod -aG gpio ubuntu
RUN usermod -aG dialout ubuntu
USER ubuntu

RUN sudo rosdep init && rosdep update

# Set workspace as working directory
WORKDIR /ws

# Ensure ROS 2 environment is sourced for the new user
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Entrypoint
CMD ["/bin/bash"]

FROM osrf/ros:humble-desktop-full

# Tránh prompts khi cài đặt
ENV DEBIAN_FRONTEND=noninteractive

# Cài đặt các công cụ cơ bản
RUN apt-get update \
    && apt-get install -y \
    nano \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Copy config nếu có
# COPY config/ /site_config/

# Tạo arguments cho user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Tạo non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Thiết lập sudo
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Cài đặt các package ROS 2 bổ sung
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Chuyển sang user
USER $USERNAME

# Set working directory
WORKDIR /home/$USERNAME/dev_ws

# Set entrypoint
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]
# Base ROS2 Humble Docker image for development with devcontainers
# This image includes VNC server and Code Server for easy local and remote development

FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8
ENV TZ=UTC

# Install common development tools and dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    vim \
    nano \
    sudo \
    locales \
    tzdata \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    bash-completion \
    gdb \
    xauth \
    x11-apps \
    dbus-x11 \
    && rm -rf /var/lib/apt/lists/*

# Configure locale
RUN locale-gen en_US.UTF-8

# Install desktop environment and VNC server
RUN apt-get update && apt-get install -y \
    xfce4 \
    xfce4-goodies \
    tigervnc-standalone-server \
    tigervnc-common \
    novnc \
    websockify \
    supervisor \
    && rm -rf /var/lib/apt/lists/*

# Install code-server
ARG CODE_SERVER_VERSION=4.20.0
RUN curl -fsSL https://code-server.dev/install.sh | sh -s -- --version=${CODE_SERVER_VERSION}

# VNC and Code Server password arguments
ARG VNC_PASSWORD=vncpassword
ARG CODE_SERVER_PASSWORD=codeserver

# Install VS Code extensions for ROS2 development
RUN code-server --install-extension ms-python.python \
    && code-server --install-extension ms-vscode.cpptools \
    && code-server --install-extension ms-vscode.cmake-tools \
    && code-server --install-extension twxs.cmake \
    && code-server --install-extension ms-iot.vscode-ros

# Create a non-root user for development
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Setup VNC directories and configuration
RUN mkdir -p /home/$USERNAME/.vnc \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/.vnc

# Configure VNC password (can be customized via build arg)
RUN echo "$VNC_PASSWORD" | vncpasswd -f > /home/$USERNAME/.vnc/passwd \
    && chmod 600 /home/$USERNAME/.vnc/passwd \
    && chown $USERNAME:$USERNAME /home/$USERNAME/.vnc/passwd

# Create VNC startup script
RUN mkdir -p /home/$USERNAME/.vnc \
    && echo '#!/bin/bash' > /home/$USERNAME/.vnc/xstartup \
    && echo 'unset SESSION_MANAGER' >> /home/$USERNAME/.vnc/xstartup \
    && echo 'unset DBUS_SESSION_BUS_ADDRESS' >> /home/$USERNAME/.vnc/xstartup \
    && echo 'exec startxfce4' >> /home/$USERNAME/.vnc/xstartup \
    && chmod +x /home/$USERNAME/.vnc/xstartup \
    && chown $USERNAME:$USERNAME /home/$USERNAME/.vnc/xstartup

# Setup code-server configuration directory
RUN mkdir -p /home/$USERNAME/.config/code-server \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/.config

# Create code-server config
RUN echo "bind-addr: 0.0.0.0:8080" > /home/$USERNAME/.config/code-server/config.yaml \
    && echo "auth: password" >> /home/$USERNAME/.config/code-server/config.yaml \
    && echo "password: $CODE_SERVER_PASSWORD" >> /home/$USERNAME/.config/code-server/config.yaml \
    && echo "cert: false" >> /home/$USERNAME/.config/code-server/config.yaml \
    && chown $USERNAME:$USERNAME /home/$USERNAME/.config/code-server/config.yaml

# Create supervisor configuration directory
RUN mkdir -p /etc/supervisor/conf.d

# Create supervisor config for services
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf

# Setup workspace directory
RUN mkdir -p /home/$USERNAME/ros2_ws/src \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/ros2_ws

# Switch to non-root user
USER $USERNAME
WORKDIR /home/$USERNAME

# Source ROS2 setup in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

# Expose ports
# 5901: VNC
# 6080: noVNC (web-based VNC client)
# 8080: code-server
EXPOSE 5901 6080 8080

# Create entrypoint script
USER root
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER $USERNAME

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/conf.d/supervisord.conf"]

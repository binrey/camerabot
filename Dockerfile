ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}
# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ARG USERNAME
ARG USER_UID
ARG USER_GID

# Install ROS 2 Jazzy (for webots)
RUN apt-get update && apt-get install -y ros-jazzy-vision-msgs

# Create user and group
RUN groupadd -g $USER_GID $USERNAME \
    && useradd -u $USER_UID -g $USER_GID -m $USERNAME \
    && usermod -aG sudo $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to non-root user
USER $USERNAME
WORKDIR /home/$USERNAME

ENV HOME=/home/$USERNAME

# Create a .bashrc entry to automatically source ROS 2 and navigate to workspace
RUN echo "source /opt/ros/jazzy/setup.sh" >> ~/.bashrc \
    && echo "cd /home/$USERNAME" >> ~/.bashrc

# Set default command
CMD ["bash"] 
ARG ROS_DISTRO
FROM ${ROS_DISTRO}

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ARG USERNAME
ARG USER_UID
ARG USER_GID

RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg2 \
    libgl1 \
    libqt5gui5 \
    libjpeg-dev \
    libpulse0 \
    wget \
    sudo

# 3. Install Webots 2025a
ARG WEBOTS_VERSION=2025a
# determine correct URL for .deb
RUN if echo ${WEBOTS_VERSION} | grep -q nightly; then \
      WEBOTS_URL="https://github.com/cyberbotics/webots/releases/download/${WEBOTS_VERSION}_amd64.deb"; \
    else \
      WEBOTS_URL="https://github.com/cyberbotics/webots/releases/download/R${WEBOTS_VERSION}/webots_${WEBOTS_VERSION}_amd64.deb"; \
    fi && \
    wget ${WEBOTS_URL} -O /tmp/webots.deb && \
    apt-get install -y /tmp/webots.deb && \
    rm /tmp/webots.deb


RUN apt-get update && apt-get install -y ros-humble-webots-ros2
    
# Create user and group with proper handling of existing users and groups
RUN if getent group $USER_GID > /dev/null 2>&1; then \
        # Group exists, check if it's the right name
        existing_group=$(getent group $USER_GID | cut -d: -f1); \
        if [ "$existing_group" != "$USERNAME" ]; then \
            groupmod -n $USERNAME $existing_group; \
        fi; \
    else \
        # Group doesn't exist, create it
        groupadd -g $USER_GID $USERNAME; \
    fi \
    && if getent passwd $USER_UID > /dev/null 2>&1; then \
        # User exists, modify it to match our requirements
        existing_user=$(getent passwd $USER_UID | cut -d: -f1); \
        if [ "$existing_user" != "$USERNAME" ]; then \
            usermod -l $USERNAME -d /home/$USERNAME -m $existing_user; \
        fi; \
        usermod -g $USERNAME $USERNAME; \
    else \
        # User doesn't exist, create it
        useradd -u $USER_UID -g $USER_GID -m $USERNAME; \
    fi \
    && usermod -aG sudo $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to non-root user
USER $USERNAME
WORKDIR /home/$USERNAME/camerabot
ENV HOME=/home/$USERNAME

# Create a .bashrc entry to automatically source ROS 2 and navigate to workspace
RUN echo "source /opt/ros/humble/setup.sh" >> ~/.bashrc \
    && echo "cd /home/$USERNAME/camerabot" >> ~/.bashrc

# Set default command
CMD ["bash"] 
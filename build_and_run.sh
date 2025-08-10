#!/usr/bin/env bash
set -euo pipefail

ros_distro=$1
container_name=$2
headless=${3:-false}  # Default to false (GUI mode)

# Stop and remove existing container
echo "🛑 Stopping existing container..."
docker stop $container_name 2>/dev/null || true
docker rm $container_name 2>/dev/null || true

#  2. Set environment variables 
export USER_UID=$(id -u)
export USER_GID=$(id -g)
export USERNAME=$USER
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

#  3. Create Docker network for ROS2 containers
# echo "🌐 Creating ROS2 network..."
# docker network create camerabot_network 2>/dev/null || echo "Network already exists"

#  4. Build and run Docker container 
echo "🔨 Building ROS 2 Docker image for" $ros_distro "..."
# Build the Docker image with platform with specification and build args
docker build \
  --build-arg ROS_DISTRO=$ros_distro \
  --build-arg USER_UID=${USER_UID} \
  --build-arg USER_GID=${USER_GID} \
  --build-arg USERNAME=${USERNAME} \
  -t ros2_jazzy .

# Set up X11 forwarding only if not in headless mode
if [ "$headless" != "true" ]; then
  echo "🖥️  Setting up X11 forwarding for GUI mode..."
  xhost +local:docker
  X11_ARGS="--env QT_X11_NO_MITSHM=1 \
  --device /dev/dri \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix"
else
  echo "🚫 Running in headless mode (no GUI support)"
  X11_ARGS=""
fi

echo "🚀 Starting ROS 2 container..."
# Run the container with network configuration
docker run -it \
  --name $container_name \
  --tty \
  --interactive \
  --network host \
  --env ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
  $X11_ARGS \
  --volume .:/home/$USERNAME/camerabot:rw \
  --user "${USER_UID}:${USER_GID}" \
  ros2_jazzy \
  bash -c "echo ✅  Done! && echo Next: colcon build && source install/local_setup.bash && bash"
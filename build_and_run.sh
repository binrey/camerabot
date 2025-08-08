#!/usr/bin/env bash
set -euo pipefail

ros_distro=$1
container_name=$2

# Stop and remove existing container
echo "🛑 Stopping existing container..."
docker stop $container_name 2>/dev/null || true
docker rm $container_name 2>/dev/null || true

#  2. Set environment variables 
export USER_UID=$(id -u)
export USER_GID=$(id -g)
export USERNAME=$USER
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

echo "🔧 Building ROS 2 Docker image..."

#  3. Create Docker network for ROS2 containers
# echo "🌐 Creating ROS2 network..."
# docker network create camerabot_network 2>/dev/null || echo "Network already exists"

#  4. Build and run Docker container 
echo "🔨 Building ROS 2 Docker image..."
# Build the Docker image with platform specification and build args
docker build \
  --build-arg ROS_DISTRO=$ros_distro \
  --build-arg USER_UID=${USER_UID} \
  --build-arg USER_GID=${USER_GID} \
  --build-arg USERNAME=${USERNAME} \
  -t ros2_jazzy .

xhost +local:docker

echo "🚀 Starting ROS 2 container..."
# Run the container with network configuration
docker run -it \
  --name $container_name \
  --tty \
  --interactive \
  --network host \
  --env ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
  --env QT_X11_NO_MITSHM=1 \
  --device /dev/dri \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume .:/home/$USERNAME/camerabot:rw \
  --user "${USER_UID}:${USER_GID}" \
  ros2_jazzy \
  bash -c "source install/local_setup.bash && echo ✅  Done! && bash"
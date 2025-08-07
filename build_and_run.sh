#!/usr/bin/env bash
set -euo pipefail

ros_distro_variant=$1
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

echo "🔧 Building minimal ROS 2 Docker image..."

#  3. Create Docker network for ROS2 containers
# echo "🌐 Creating ROS2 network..."
# docker network create camerabot_network 2>/dev/null || echo "Network already exists"

#  4. Build and run Docker container 
echo "🔨 Building ROS 2 Docker image..."
# Build the Docker image with platform specification and build args
docker build
  --build-arg BASE_VARIANT=${ros_distro_variant} \
  --build-arg USER_UID=${USER_UID} \
  --build-arg USER_GID=${USER_GID} \
  --build-arg USERNAME=${USERNAME} \
  -t ros2_jazzy .

echo "🚀 Starting ROS 2 container..."
# Run the container with network configuration
docker run -d \
  --name $container_name \
  --tty \
  --interactive \
  --network host \
  --env ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
  --volume .:/home/$USERNAME:rw \
  --user "${USER_UID}:${USER_GID}" \
  ros2_jazzy \
  bash

echo -e "\n✅  Done!  Try it out:\n"
echo "   docker exec -it $container_name bash"
echo "   # inside container:"
echo "   source /opt/ros/jazzy/setup.sh"
echo "   source install/local_setup.bash"
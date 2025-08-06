#!/usr/bin/env bash
set -euo pipefail

#  1. Sanity checks 
ARCH=$(uname -m)
if [[ "$ARCH" != "aarch64" ]]; then
  echo "❌  This script targets 64-bit Raspberry Pi boards (aarch64)."
  exit 1
fi

#  2. Set environment variables 
export USER_UID=$(id -u)
export USER_GID=$(id -g)
export USERNAME=$USER

#  3. Build and run Docker container 
echo "🔨 Building ROS 2 Docker image..."
# Build the Docker image with platform specification and build args
docker build --platform linux/arm64 \
  --build-arg USER_UID=${USER_UID} \
  --build-arg USER_GID=${USER_GID} \
  --build-arg USERNAME=${USERNAME} \
  -t ros2_jazzy .

echo "🚀 Starting ROS 2 container..."
# Run the container with all the configuration from docker-compose.yml
docker run -d \
  --name ros2_jazzy \
  --network host \
  --tty \
  --interactive \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume .:/home/$USERNAME:rw \
  --volume ~/.Xauthority:/home/rosuser/.Xauthority:rw \
  --user "${USER_UID}:${USER_GID}" \
  ros2_jazzy \
  bash

echo -e "\n✅  Done!  Try it out:\n"
echo "   docker exec -it ros2_jazzy bash"
echo "   # inside container:"
echo "   source /opt/ros/jazzy/setup.sh"
echo "   ros2 run demo_nodes_cpp talker"
echo ""
echo "To stop the container:"
echo "   docker stop ros2_jazzy && docker rm ros2_jazzy" 
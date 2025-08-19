#!/usr/bin/bash
set -euo pipefail

ros_distro=$1
container_name=$2
image_name=$3
headless=${4:-false}  # Default to false (GUI mode)

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
  --build-arg HEADLESS=$headless \
  --build-arg USER_UID=${USER_UID} \
  --build-arg USER_GID=${USER_GID} \
  --build-arg USERNAME=${USERNAME} \
  -t $image_name .

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
echo "📷 Enabling camera access for Raspberry Pi..."

# Run the container with network configuration and camera support
docker run -it \
  --name $container_name \
  --tty \
  --interactive \
  --network host \
  --env ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
  $X11_ARGS \
  --volume .:/home/$USERNAME/camerabot:rw \
  --volume /dev:/dev \
  --volume /run/udev:/run/udev:ro \  
  --device /dev/vchiq:/dev/vchiq \
  --device /dev/video0:/dev/video0 \
  --group-add video \
  --privileged \
  $image_name \
  bash -c "echo ✅  Done! && echo Next: colcon build && bash"
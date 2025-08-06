```
#!/usr/bin/env bash
set -euo pipefail

# ──────────────── 1. Sanity checks ────────────────
if [[ $(id -u) -eq 0 ]]; then
  SUDO=""
else
  SUDO="sudo"
fi

ARCH=$(uname -m)
if [[ "$ARCH" != "aarch64" ]]; then
  echo "❌  This script targets 64-bit Raspberry Pi boards (aarch64)."
  exit 1
fi

# ──────────────── 2. System update & prerequisites ────────────────
$SUDO apt-get update
$SUDO apt-get install -y \
     ca-certificates curl gnupg lsb-release software-properties-common

# ──────────────── 3. Install Docker Engine & Compose (repo method) ────────────────
# Add Docker’s official GPG key
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | \
    $SUDO gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Add Docker apt repo for Noble
echo \
  "deb [arch=arm64 signed-by=/etc/apt/keyrings/docker.gpg] \
  https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | \
  $SUDO tee /etc/apt/sources.list.d/docker.list > /dev/null

$SUDO apt-get update
$SUDO apt-get install -y docker-ce docker-ce-cli containerd.io \
                         docker-buildx-plugin docker-compose-plugin
# Enable & start daemon
$SUDO systemctl enable --now docker

# ──────────────── 4. Manage Docker as non-root user ────────────────
$SUDO usermod -aG docker "$USER"
echo "ℹ️  Log out & back in (or run 'newgrp docker') so group changes take effect."  [oai_citation:2‡Docker Documentation](https://docs.docker.com/engine/install/linux-postinstall/?utm_source=chatgpt.com)

# ──────────────── 5. Pull ROS 2 image ────────────────
docker pull --platform linux/arm64 ros:jazzy-ros-base

# ──────────────── 6. Create project folder & docker-compose.yml ────────────────
mkdir -p ~/ros2_jazzy/ros2_ws
cat > ~/ros2_jazzy/docker-compose.yml <<'EOF'
version: "3.9"
services:
  ros:
    image: ros:jazzy-ros-base   # multi-arch; pulls arm64 on Pi
    platform: linux/arm64
    container_name: ros2_jazzy
    tty: true
    stdin_open: true
    network_mode: host         # share localhost → easier for DDS
    environment:
      - DISPLAY=${DISPLAY}     # forward X11 if you run Rviz etc.
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix    # X11 socket
      - ./ros2_ws:/root/ros2_ws          # your workspace
EOF

echo -e "\n✅  Done!  Try it out:\n"
echo "   cd ~/ros2_jazzy && docker compose up -d"
echo "   docker exec -it ros2_jazzy bash"
echo "   # inside container:"
echo "   ros2 run demo_nodes_cpp talker"
```

```
chmod +x install_ros2_docker.sh
./install_ros2_docker.sh
# log out / log back in (or `newgrp docker`)
cd ~/ros2_jazzy
docker compose up -d        # starts ROS 2 container
docker exec -it ros2_jazzy bash
ros2 topic list             # verify DDS nodes are visible
```

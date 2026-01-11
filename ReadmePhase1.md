# URDF Robot Visualization with ROS 2 in Docker

Complete guide to run the URDF example project using Docker with ROS 2, RViz, and Gazebo pre-installed.

## Prerequisites

- Docker installed on your system
- X11 server (for GUI applications)
- Your project workspace at `~/DEV_WS` (or adjust paths accordingly)

## Project Structure

```
DEV_WS/
├── src/
│   └── urdf_example/
│       ├── description/
│       ├── launch/
│       ├── worlds/
│       ├── CMakeLists.txt
│       └── package.xml
├── Dockerfile
├── docker-compose.yml
├── entrypoint.sh
└── README.md
```

## Step 1: Create entrypoint.sh

Create `entrypoint.sh` in your `DEV_WS` directory:

```bash
#!/bin/bash
set -e

# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Source workspace if it exists
if [ -f /root/dev_ws/install/setup.bash ]; then
    source /root/dev_ws/install/setup.bash
fi

# Execute the command passed to the container
exec "$@"
```

Make it executable:

```bash
chmod +x entrypoint.sh
```

## Step 2: Create Dockerfile

Create `Dockerfile` in your `DEV_WS` directory:

```dockerfile
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Install ROS 2 packages and tools
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-rviz2 \
    python3-colcon-common-extensions \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/dev_ws

# Copy entrypoint script
COPY entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

# Copy workspace
COPY . /root/dev_ws/

# Setup ROS 2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/dev_ws/install/setup.bash" >> ~/.bashrc

# Build workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /root/dev_ws && \
    colcon build --symlink-install"

ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["/bin/bash"]
```

## Step 3: Create docker-compose.yml

Create `docker-compose.yml` in your `DEV_WS` directory:

```yaml
services:
  ros2-workspace:
    build: .
    container_name: ros2-urdf-dev
    image: ros2-urdf-workspace:latest
    network_mode: host
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - .:/root/dev_ws:rw
    stdin_open: true
    tty: true
    privileged: true
```

## Step 4: Enable GUI Access

On your **host machine**, allow Docker to access the display:

```bash
xhost +local:docker
```

> **Note**: Run this command each time you restart your computer, or add it to your `~/.bashrc`

## Step 5: Build the Workspace (First Time)

Before building the Docker image, ensure to build your workspace. There are lines of code building the workspace in dockerfile. But if not, do it here.

```bash
cd ~/DEV_WS
# If you haven't built yet, or need to rebuild
# Note: This is optional since Dockerfile will build it
# But useful if you want to test locally first
colcon build --symlink-install
```

## Step 6: Build Docker Image

```bash
cd ~/DEV_WS
docker compose build
```

This will:
- Pull ROS 2 Humble Desktop Full image
- Install Gazebo, RViz, and required tools
- Run `colcon build --symlink-install` automatically inside the container

## Step 7: Start the Container

```bash
docker compose up -d
```

## Step 8: Run the Project

### Terminal 1 - Launch Robot State Publisher

```bash
docker compose exec ros2-workspace bash
ros2 launch urdf_example rsp.launch.py
```

### Terminal 2 - Launch Joint State Publisher GUI

Open a new terminal on your host:

```bash
docker compose exec ros2-workspace bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

### Terminal 3 - Launch RViz

Open another new terminal on your host:

```bash
docker compose exec ros2-workspace bash
rviz2
```

## Step 9: Configure RViz

In RViz window:

1. **Fixed Frame**: Set to `world` (in Global Options panel)
2. **Add RobotModel**:
   - Click "Add" button
   - Select "RobotModel"
   - Set "Description Topic" to `/robot_description`
   - Set "Alpha" to `0.8`
3. **Add TF Display**:
   - Click "Add" button
   - Select "TF"
   - Enable "Show Names" checkbox

## Useful Commands

### Container Management

```bash
# Start container
docker compose start

# Stop container
docker compose stop

# Restart container
docker compose restart

# View logs
docker compose logs -f

# Remove container (keeps image)
docker compose down

# Remove everything including image
docker compose down --rmi all
```

### Rebuild After Changes

```bash
# Rebuild workspace inside container
docker compose exec ros2-workspace bash
cd /root/dev_ws
colcon build --symlink-install
source install/setup.bash

# Or rebuild entire Docker image
docker compose build --no-cache
```

### Multiple Terminal Access

```bash
# Open additional terminals in the same container
docker compose exec ros2-workspace bash
```

## Troubleshooting

### GUI Applications Don't Display

```bash
# On host machine, run:
xhost +local:docker

# If still not working, try:
xhost +local:root
```

### Permission Denied Errors

```bash
# Ensure files have correct permissions
sudo chown -R $USER:$USER ~/DEV_WS
```

### Container Won't Start

```bash
# Check container logs
docker-compose logs

# Remove and rebuild
docker-compose down
docker-compose build --no-cache
docker-compose up -d
```

### Build Fails Inside Container

```bash
# Enter container
docker-compose exec ros2-workspace bash

# Clean build
cd /root/dev_ws
rm -rf build install log
colcon build --symlink-install
```

## Alternative: Docker Run (Without Docker Compose)

If you prefer using `docker run` directly:

```bash
# Build
docker build -t ros2-urdf-workspace .

# Run
docker run -it \
    --name ros2-urdf-dev \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd):/root/dev_ws:rw \
    --privileged \
    ros2-urdf-workspace

# Additional terminals
docker exec -it ros2-urdf-dev bash
```

## Notes

- **ROS 2 Distribution**: This uses Humble (for Ubuntu 22.04). For Ubuntu 24.04 with Jazzy, change to `osrf/ros:jazzy-desktop-full` and replace `humble` with `jazzy` in all commands.
- **Workspace Syncing**: Changes made to files in `~/DEV_WS` on your host are immediately reflected in the container due to volume mounting.
- **Performance**: First build takes 5-10 minutes. Subsequent builds are faster.

## Quick Start Summary

```bash
# One-time setup
cd ~/DEV_WS

# Create entrypoint.sh and make it executable
chmod +x entrypoint.sh

xhost +local:docker
docker-compose build

# Every time you work
docker-compose up -d

# Terminal 1
docker-compose exec ros2-workspace bash
ros2 launch urdf_example rsp.launch.py

# Terminal 2
docker-compose exec ros2-workspace bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Terminal 3
docker-compose exec ros2-workspace bash
rviz2
```

---

**Project ready to run!** 🚀
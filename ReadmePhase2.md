# Gazebo Simulation with ROS 2 Humble

Complete guide for launching Gazebo and spawning robots/objects in simulation.

---

## Table of Contents
- [Part 1: Launching Gazebo](#part-1-launching-gazebo)
- [Part 2: Spawning Entities](#part-2-spawning-entities)
- [Complete Workflow](#complete-workflow)
- [Troubleshooting](#troubleshooting)

---

## Part 1: Launching Gazebo

### Method 1: Empty Gazebo (Quick Testing)

**Command:**
```bash
ros2 launch gazebo_ros gazebo.launch.py
```

**Use case:** Quick testing, learning Gazebo, testing plugins

**Files needed:** None (uses default empty world)

---

### Method 2: Complete Launch File (Recommended)

Launches Gazebo with robot state publisher and spawns robot automatically.

**Create `launch/gazebo_complete.launch.py`:**

```python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('urdf_example')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Path to URDF file
    urdf_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    
    # Process URDF with xacro
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(pkg_share, 'worlds', 'empty.world'),
            'verbose': 'true'
        }.items()
    )
    
    # Spawn Robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'my_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
    ])
```

**Launch:**
```bash
ros2 launch urdf_example gazebo_complete.launch.py
```

---

## Part 2: Spawning Entities

Use these methods when Gazebo is already running and you want to add entities dynamically.

### Method 1: Spawn Robot via spawn_entity.py

**Prerequisites:**
- Gazebo must be running
- Robot description must be published (via robot_state_publisher)

**From topic (recommended):**
```bash
ros2 run gazebo_ros spawn_entity.py \
    -topic /robot_description \
    -entity my_robot \
    -x 0.0 -y 0.0 -z 0.5
```

**From file:**
```bash
ros2 run gazebo_ros spawn_entity.py \
    -file /path/to/robot.urdf \
    -entity my_robot
```

**Common arguments:**

| Argument | Description | Example |
|----------|-------------|---------|
| `-entity` | Name (required) | `my_robot` |
| `-topic` | URDF topic | `/robot_description` |
| `-file` | URDF file path | `/path/to/robot.urdf` |
| `-x`, `-y`, `-z` | Position (meters) | `0.0 0.0 0.5` |
| `-Y` | Yaw rotation (radians) | `1.57` (90°) |

**Spawn multiple robots:**
```bash
# Robot 1
ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity robot1 -x 0.0

# Robot 2
ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity robot2 -x 2.0
```

**In launch file:**
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'my_robot',
            '-x', '0.0', '-y', '0.0', '-z', '0.5'
        ],
        output='screen'
    )
    
    return LaunchDescription([spawn_robot])
```

---

### Method 2: Spawn Objects from Model Database

**Common models:**
- Furniture: `cafe_table`, `bookshelf`
- Objects: `coke_can`, `cricket_ball`, `construction_cone`
- More at: `/usr/share/gazebo-11/models/`

**Basic usage:**
```bash
ros2 run gazebo_ros spawn_entity.py \
    -database [model_name] \
    -entity [instance_name] \
    -x [x] -y [y] -z [z]
```

**Examples:**
```bash
# Spawn table
ros2 run gazebo_ros spawn_entity.py -database cafe_table -entity table1 -x 2.0 -y 2.0

# Spawn can on table
ros2 run gazebo_ros spawn_entity.py -database coke_can -entity can1 -x 2.0 -z 0.8

# Spawn ball
ros2 run gazebo_ros spawn_entity.py -database cricket_ball -entity ball1 -x 0.5 -z 2.0
```

**In launch file:**
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    spawn_table = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-database', 'cafe_table', '-entity', 'table1', '-x', '2.0'],
        output='screen'
    )
    
    spawn_can = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-database', 'coke_can', '-entity', 'can1', '-x', '2.0', '-z', '0.8'],
        output='screen'
    )
    
    return LaunchDescription([spawn_table, spawn_can])
```

---

## Complete Workflow

### Option A: All-in-One Launch (Recommended)

**Use when:** You want everything launched automatically

**Launch file includes:**
- Gazebo
- Robot state publisher
- Robot spawn
- Object spawns (optional)

```python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('urdf_example')
    pkg_gazebo = get_package_share_directory('gazebo_ros')
    
    urdf_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
        )
    )
    
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'my_robot', '-z', '0.5']
    )
    
    spawn_table = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-database', 'cafe_table', '-entity', 'table1', '-x', '2.0']
    )
    
    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        spawn_table,
    ])
```

**Usage:**
```bash
ros2 launch urdf_example gazebo_complete.launch.py
```

---

### Option B: Manual Step-by-Step

**Use when:** You want control over each step or testing individual components

**Step 1: Launch Gazebo**
```bash
ros2 launch gazebo_ros gazebo.launch.py
```

**Step 2: Publish robot description**
```bash
ros2 launch your_package rsp.launch.py
```

**Step 3: Spawn robot**
```bash
ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity my_robot -z 0.5
```

**Step 4: Spawn objects (optional)**
```bash
ros2 run gazebo_ros spawn_entity.py -database cafe_table -entity table1 -x 2.0
ros2 run gazebo_ros spawn_entity.py -database coke_can -entity can1 -x 2.0 -z 0.8
```

---

## Package Structure

```
your_package/
├── launch/
│   ├── gazebo_complete.launch.py     # All-in-one
│   ├── spawn_robot.launch.py         # Spawn only
│   └── spawn_objects.launch.py       # Objects only
├── worlds/
│   ├── empty.world
│   └── custom.world
├── description/
│   └── robot.urdf.xacro
├── CMakeLists.txt
└── package.xml
```

**CMakeLists.txt:**
```cmake
install(DIRECTORY
  launch
  worlds
  description
  DESTINATION share/${PROJECT_NAME}
)
```

**package.xml:**
```xml
<exec_depend>gazebo_ros</exec_depend>
<exec_depend>gazebo_ros_pkgs</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>xacro</exec_depend>
```

---

## Testing Your Setup

```bash
# 1. Build workspace
cd ~/dev_ws
colcon build --symlink-install
source install/setup.bash

# 2. Test empty Gazebo
ros2 launch gazebo_ros gazebo.launch.py

# 3. Test complete launch
ros2 launch your_package gazebo_complete.launch.py

# 4. Verify topics
ros2 topic list
# Should see: /robot_description, /clock, /tf, /tf_static
```

---

## Troubleshooting

### Entity Already Exists
**Error:** `An entity named [my_robot] already exists`

**Solution:**
```bash
# Delete entity
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'my_robot'}"

# Or restart Gazebo
pkill -9 gazebo
ros2 launch gazebo_ros gazebo.launch.py
```

### Robot Description Topic Not Found
**Error:** `Topic [/robot_description] does not appear to be published`

**Solution:**
```bash
# Check if topic exists
ros2 topic list | grep robot_description

# Launch robot_state_publisher first
ros2 launch your_package rsp.launch.py

# Then spawn robot
ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity my_robot
```

### Model Not Found
**Error:** `Unable to find model [cafe_table]`

**Solution:**
```bash
# Check available models
ls /usr/share/gazebo-11/models/

# Set model path
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH

# Add to ~/.bashrc for persistence
echo "export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH" >> ~/.bashrc
```

### Robot Spawns Underground
**Problem:** Robot appears below ground plane

**Solution:**
```bash
# Increase z position
ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity my_robot -z 0.5
```

### Gazebo Not Running
**Error:** `Unable to connect to Gazebo server`

**Solution:**
```bash
# Start Gazebo first and wait for it to fully load
ros2 launch gazebo_ros gazebo.launch.py

# Wait 5-10 seconds, then spawn entities
```

---

## Quick Reference

### Launch Commands
```bash
# Empty Gazebo
ros2 launch gazebo_ros gazebo.launch.py

# Complete launch (all-in-one)
ros2 launch your_package gazebo_complete.launch.py
```

### Spawn Commands
```bash
# Spawn robot from topic
ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity my_robot -z 0.5

# Spawn robot from file
ros2 run gazebo_ros spawn_entity.py -file /path/to/robot.urdf -entity my_robot

# Spawn database model
ros2 run gazebo_ros spawn_entity.py -database cafe_table -entity table1 -x 2 -y 2

# Delete entity
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'my_robot'}"
```

### Utility Commands
```bash
# List available models
ls /usr/share/gazebo-11/models/

# List entities in Gazebo
ros2 service call /get_model_list gazebo_msgs/srv/GetModelList

# Check robot_description
ros2 topic echo /robot_description

# Kill Gazebo
pkill -9 gazebo
```

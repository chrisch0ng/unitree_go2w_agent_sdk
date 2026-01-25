# BCIT Make+ Private Documentation
## Unitree Go2W Agent SDK - Complete Codebase Reference

**Author:** Christopher Chong
**Organization:** BCIT Make+
**Purpose:** Assistive Technology Development
**Last Updated:** January 2025

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Maestro Integration](#maestro-integration)
3. [Languages Used](#languages-used)
4. [Directory Structure](#directory-structure)
5. [Core System Components](#core-system-components)
6. [ROS1 Components (unitree_ros1/)](#ros1-components)
7. [ROS2 Components (unitree_ros2/)](#ros2-components)
8. [Configuration Files](#configuration-files)
9. [Key Python Files Reference](#key-python-files-reference)
10. [ROS Topics & Services](#ros-topics--services)
11. [Hardware Interfaces](#hardware-interfaces)
12. [Build & Run Commands](#build--run-commands)
13. [Extension Points for Assistive Tech](#extension-points-for-assistive-tech)

---

## Project Overview

This SDK provides a unified control system for the **Unitree Go2W** quadruped robot (wheeled variant) with an **AgileX Piper** 6-DOF robotic arm. The system bridges ROS1 (Noetic) and ROS2 (Foxy) ecosystems to provide:

- **Locomotion Control**: Velocity-based movement of the robot dog
- **Robotic Arm Control**: 6 joint angles + gripper via CAN bus
- **SLAM & Navigation**: LiDAR-based mapping and autonomous navigation
- **Object Detection**: YOLOv8/v9 with 3D localization
- **Sensor Integration**: RealSense RGB-D camera, Hesai LiDAR, IMU

### System Architecture (with Maestro)

```
┌─────────────────────────────────────────────────────────────────┐
│                         MAESTRO                                  │
│            VLM-Based Coding Agent (Policy Planner)               │
│    ┌─────────────────────────────────────────────────────────┐  │
│    │  Vision-Language Model (Qwen2.5-VL / Claude / GPT)      │  │
│    │  - Interprets natural language instructions             │  │
│    │  - Generates executable Python code                     │  │
│    │  - Closed-loop feedback from execution results          │  │
│    └─────────────────────────────────────────────────────────┘  │
│                              │                                   │
│    ┌─────────────────────────┼─────────────────────────────┐    │
│    │              Hierarchical Toolset                      │    │
│    ├──────────────┬──────────────┬──────────────┬──────────┤    │
│    │  Perception  │   Planning   │   Control    │ Geometry │    │
│    │  - YOLO      │   - Nav2     │   - Arm SDK  │ - tf2    │    │
│    │  - 3D Det    │   - Motion   │   - Dog SDK  │ - NumPy  │    │
│    │  - Keypoints │   - Explore  │   - Gripper  │ - PCL    │    │
│    └──────────────┴──────────────┴──────────────┴──────────┘    │
└─────────────────────────────────────────────────────────────────┘
                                │
                   Python API / ROS Topics & Services
                                │
┌───────────────────────────────┼───────────────────────────────┐
│                         ROS1 Bridge                            │
│                    (dynamic_bridge)                            │
└───────────────────────────────┼───────────────────────────────┘
        │                                       │
┌───────┴───────┐                     ┌────────┴────────┐
│    ROS1       │                     │      ROS2       │
│   (Noetic)    │                     │     (Foxy)      │
├───────────────┤                     ├─────────────────┤
│ - Faster-LIO  │                     │ - Nav2          │
│ - HesaiLiDAR  │                     │ - YOLO          │
│ - SLAM        │                     │ - RealSense     │
└───────────────┘                     │ - Piper Arm     │
                                      │ - Msg Converter │
                                      └─────────────────┘
                                              │
                              ┌───────────────┼───────────────┐
                              │               │               │
                        ┌─────┴─────┐   ┌─────┴─────┐   ┌─────┴─────┐
                        │  Go2 Dog  │   │  Piper    │   │  Sensors  │
                        │(Ethernet) │   │(CAN Bus)  │   │(USB/ETH)  │
                        └───────────┘   └───────────┘   └───────────┘
```

---

## Maestro Integration

**Website:** https://maestro-robot.github.io/

### What is Maestro?

Maestro is a **VLM-based (Vision-Language Model) coding agent** that generates programmatic policies for robot control. Instead of end-to-end trained models, Maestro uses a VLM to interpret natural language instructions and write executable Python code that composes specialized robotics modules.

**Key Insight:** Maestro was specifically designed for and tested on the **Unitree Go2-W with AgileX PiPER arm** - exactly this hardware platform!

### How Maestro Works

```
┌─────────────────────────────────────────────────────────────────┐
│                    MAESTRO EXECUTION LOOP                        │
│                                                                  │
│   1. USER INSTRUCTION                                            │
│      "Pick up the red cup and bring it to me"                   │
│                         │                                        │
│                         ▼                                        │
│   2. VLM INTERPRETS & GENERATES CODE                            │
│      ┌─────────────────────────────────────────────────────┐    │
│      │ # Maestro-generated policy                          │    │
│      │ cup = detect_object("red cup")                      │    │
│      │ position_3d = get_3d_position(cup)                  │    │
│      │ navigate_to(position_3d, offset=0.5)                │    │
│      │ arm_reach(position_3d)                              │    │
│      │ gripper_close()                                     │    │
│      │ navigate_to(user_position)                          │    │
│      │ gripper_open()                                      │    │
│      └─────────────────────────────────────────────────────┘    │
│                         │                                        │
│                         ▼                                        │
│   3. EXECUTE & OBSERVE                                          │
│      - Run code against ROS modules                             │
│      - Capture camera images + stdout                           │
│                         │                                        │
│                         ▼                                        │
│   4. FEEDBACK LOOP                                              │
│      - VLM analyzes execution results                           │
│      - Replans if needed (object moved, obstacle detected)      │
│      - Continues until task complete                            │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Maestro's Hierarchical Toolset

Maestro organizes robot capabilities into a carefully curated set of modules:

#### Perception Modules (Tiered: Fast → Precise)

| Tier | Module | Speed | This Codebase Component |
|------|--------|-------|-------------------------|
| 1 | Raw camera input | Fastest | `/camera/color/image_raw` |
| 2 | YOLO bounding boxes | Fast | `yolov8_node.py` → `/yolov8_node/detections` |
| 3 | Mask centroids | Medium | `yolov8_node.py` (segmentation masks) |
| 4 | VLM-selected keypoints | Precise | `detect_3d_node.py` → `/detections_3d` |

#### Planning Modules

| Module | Purpose | This Codebase Component |
|--------|---------|-------------------------|
| Motion Planning | Collision-free paths | Nav2 + point cloud costmaps |
| Active Exploration | Information gathering | LiDAR-Inertial Odometry (Faster-LIO) |
| Semantic Mapping | Object tracking | YOLO + 3D projection |

#### Control Modules

| Module | Purpose | This Codebase Component |
|--------|---------|-------------------------|
| Locomotion | Move the dog | `unitree_nav2.py` → `/cmd_vel` |
| Arm Control | Move end-effector | `piper_ctrl_single_node.py` → `/pos_cmd` |
| Gripper | Grasp objects | Piper gripper via CAN |

#### Geometry Tools

| Module | Purpose | This Codebase Component |
|--------|---------|-------------------------|
| TF2 Transforms | Coordinate frame math | `tf_pub.py`, ROS2 tf2 |
| Point Cloud Processing | 3D spatial reasoning | Faster-LIO `/cloud_registered` |
| Camera Projection | 2D→3D conversion | `detect_3d_node.py` |

### How Maestro Uses This Codebase

Maestro acts as the **"brain"** that orchestrates all the ROS components in this SDK:

```
┌──────────────────────────────────────────────────────────────────┐
│                     MAESTRO (VLM Agent)                          │
│                                                                  │
│  Input: "Find my water bottle and bring it here"                │
│                                                                  │
│  VLM generates Python code that calls:                          │
└──────────────────────────────────────────────────────────────────┘
         │                    │                    │
         ▼                    ▼                    ▼
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│   PERCEPTION    │  │    PLANNING     │  │    CONTROL      │
│                 │  │                 │  │                 │
│ yolov8_node.py  │  │ Nav2 (ROS2)    │  │ unitree_nav2.py │
│ detect_3d_node  │  │ Faster-LIO     │  │ piper_ctrl_node │
│ RealSense       │  │ Costmaps       │  │ Gripper         │
└────────┬────────┘  └────────┬───────┘  └────────┬────────┘
         │                    │                    │
         └────────────────────┴────────────────────┘
                              │
                              ▼
                     ┌─────────────────┐
                     │   HARDWARE      │
                     │  Go2W + PiPER   │
                     └─────────────────┘
```

### Mapping Maestro Modules to ROS Topics

| Maestro Function | ROS Topic/Service | Node |
|------------------|-------------------|------|
| `detect_object(class)` | `/yolov8_node/detections` | yolov8_node.py |
| `get_3d_position(det)` | `/detect_3d_node/detections_3d` | detect_3d_node.py |
| `get_camera_image()` | `/camera/color/image_raw` | RealSense |
| `get_depth_image()` | `/camera/depth/image_rect_raw` | RealSense |
| `navigate_to(pose)` | `/goal_pose` | Nav2 |
| `move_base(vx,vy,vyaw)` | `/cmd_vel` | unitree_nav2.py |
| `arm_move_to(x,y,z,r,p,y)` | `/pos_cmd` | piper_ctrl_single_node.py |
| `gripper_close()` | `/pos_cmd` (gripper field) | piper_ctrl_single_node.py |
| `get_robot_pose()` | `/Odometry` | Faster-LIO |
| `get_point_cloud()` | `/cloud_registered` | Faster-LIO |

### VLM Monitoring

Maestro uses a locally-hosted VLM (Qwen2.5-VL-72B-Instruct) running at **2 Hz** to:
- Monitor task progress via camera feed
- Detect unexpected situations
- Trigger replanning when needed

### Example Maestro Policy for Assistive Tech

**User says:** "Get me the medicine bottle from the kitchen counter"

**Maestro generates:**
```python
# Maestro-generated assistive task policy

# 1. Perception: Find the medicine bottle
detections = call_yolo_detection()
medicine = filter_by_class(detections, "bottle")

if not medicine:
    # Active perception: explore to find it
    explore_area("kitchen")
    detections = call_yolo_detection()
    medicine = filter_by_class(detections, "bottle")

# 2. Get 3D position
bottle_3d = get_3d_position(medicine[0])

# 3. Navigate to the bottle
navigate_to(bottle_3d, standoff_distance=0.5)

# 4. Reach and grasp
arm_move_to(bottle_3d.x, bottle_3d.y, bottle_3d.z + 0.1)  # Approach
arm_move_to(bottle_3d.x, bottle_3d.y, bottle_3d.z)        # Lower
gripper_close()

# 5. Retract
arm_move_to(bottle_3d.x, bottle_3d.y, bottle_3d.z + 0.2)

# 6. Return to user
user_position = get_user_position()  # From person detection
navigate_to(user_position)

# 7. Deliver
arm_extend_to_user()
gripper_open()
arm_retract()

print("Medicine delivered!")
```

### Why Maestro + This Codebase for Assistive Tech

| Benefit | Explanation |
|---------|-------------|
| **Zero-shot generalization** | VLM understands novel instructions without retraining |
| **Natural language interface** | Users can give commands in plain English |
| **Closed-loop execution** | Automatically handles failures and replans |
| **Modular architecture** | Easy to add new capabilities (voice, alerts, etc.) |
| **Tested on this hardware** | Maestro was developed specifically for Go2W + PiPER |

### Getting Started with Maestro

1. **This codebase provides the low-level modules** that Maestro calls
2. **Maestro provides the high-level reasoning** that interprets user intent
3. **Together** they enable complex assistive tasks from simple commands

**Resources:**
- Maestro Paper & Code: https://maestro-robot.github.io/
- This SDK provides all the ROS modules Maestro needs

---

## Languages Used

| Language | Percentage | Primary Use |
|----------|------------|-------------|
| **Python** | ~45% | ROS2 nodes, YOLO detection, arm control, message converters, launch scripts |
| **C++** | ~40% | ROS1 SLAM (Faster-LIO), LiDAR drivers, RealSense drivers, CycloneDDS middleware |
| **XML** | ~8% | ROS package manifests (package.xml), launch files (.launch), URDF robot descriptions |
| **YAML** | ~4% | Configuration files, Nav2 parameters, LiDAR config |
| **Bash/Shell** | ~2% | Environment setup scripts, CAN bus configuration |
| **CMake** | ~1% | Build system configuration |

### Language Breakdown by Component

```
unitree_ros1/
├── C++ (90%) - Faster-LIO SLAM, HesaiLiDAR driver
├── Python (5%) - Plotting utilities
└── XML/YAML (5%) - Launch files, configs

unitree_ros2/
├── Python (60%) - All ROS2 nodes
├── C++ (25%) - RealSense drivers, CycloneDDS
├── XML (10%) - Package manifests
└── YAML (5%) - Nav2 params, configs

Root/
├── Python (100%) - launch_ros.py, cleanup_ros.py
└── YAML - robot_config.yaml
```

---

## Directory Structure

```
unitree_go2w_agent_sdk/
│
├── launch_ros.py              # [Python] Main launcher - starts all components
├── cleanup_ros.py             # [Python] Kills all ROS processes
├── robot_config.yaml          # [YAML] Module enable/disable configuration
├── requirements.txt           # [Text] Python dependencies
│
├── cad_parts/                 # CAD files for custom mounting hardware
│   ├── BOM.csv               # Bill of materials
│   ├── parts/
│   │   ├── png/              # Part images
│   │   ├── steps/            # STEP CAD files (editable)
│   │   └── stl/              # STL files (3D printing)
│   └── png/                  # Assembly images
│
├── images/                    # Documentation images
│   ├── banner.png
│   ├── dog_demo.gif
│   ├── nav2.gif
│   ├── rviz_view.png
│   └── system_overview.png
│
├── unitree_ros1/              # ROS1 Noetic workspace
│   ├── launch/
│   │   └── ros1_system.launch    # [XML] Combined ROS1 launch
│   └── src/
│       ├── faster-lio/           # [C++] LiDAR-Inertial SLAM
│       └── HesaiLidar_ROS_2.0/   # [C++] Hesai LiDAR driver
│
└── unitree_ros2/              # ROS2 Foxy workspace
    ├── setup.sh                  # [Bash] Environment setup
    ├── launch/                   # [Python] Launch files
    ├── cyclonedds_ws/            # [C++] DDS middleware + Unitree messages
    └── src/
        ├── nav2_cloud_bringup/   # [Python] Navigation + dog control
        ├── yolo_ros/             # [Python] Object detection
        ├── piper_ros/            # [Python] Robotic arm control
        ├── realsense-ros/        # [C++] Intel RealSense driver
        ├── unitree_msg_converter/ # [Python] Message converters
        └── rclpy_lifecycle/      # [Python] Lifecycle node utilities
```

---

## Core System Components

### 1. launch_ros.py (Main Launcher)

**Location:** `./launch_ros.py`
**Language:** Python
**Purpose:** Orchestrates the startup of all ROS components in proper sequence

**Key Features:**
- Loads configuration from `robot_config.yaml`
- Manages ROS1 and ROS2 environment sourcing
- Launches components with configurable delays
- Handles graceful shutdown with process group management
- Filters verbose log output

**Class: `UnitreeRobotLauncher`**
```python
Methods:
- load_config(config_file)      # Load YAML configuration
- setup_environment()           # Source ROS1/ROS2 environments
- launch_component(name, cmd)   # Start a single component
- launch_all_components()       # Start everything in sequence
- signal_handler()              # Graceful shutdown
- cleanup_all_processes()       # Force kill all ROS processes
```

**Launch Sequence:**
1. ROS1 Core (roscore)
2. ROS Bridge (ros1_bridge)
3. RealSense Camera
4. YOLO Detection (if enabled)
5. IMU Converter
6. TF Publishers
7. Arm Module (if enabled)
8. LiDAR Driver (if enabled)
9. SLAM (if enabled)
10. Navigation (if enabled)

---

### 2. cleanup_ros.py (Process Cleanup)

**Location:** `./cleanup_ros.py`
**Language:** Python
**Purpose:** Emergency cleanup script to kill all ROS processes

**Processes Killed:**
- roscore, rosmaster
- hesai_ros_driver, faster_lio
- ros2, realsense2_camera
- robot_state_publisher
- imu_converter, relay_joint
- ros1_bridge
- nav2 components

---

### 3. robot_config.yaml (Configuration)

**Location:** `./robot_config.yaml`
**Language:** YAML
**Purpose:** Enable/disable modules and set parameters

**Sections:**
```yaml
core_infrastructure:     # ROS1 core + bridge (always needed)
realsense:              # Intel RealSense RGB-D camera
yolo:                   # Object detection settings
arm_module:             # AgileX Piper robotic arm
lidar_driver:           # Hesai LiDAR driver
slam:                   # Faster-LIO SLAM algorithm
navigation:             # Nav2 autonomous navigation
message_converters:     # IMU + TF converters
delays:                 # Startup delay timing
log_filtering:          # Suppress verbose logs
```

---

## ROS1 Components

### unitree_ros1/src/faster-lio/

**Language:** C++
**Purpose:** High-performance LiDAR-Inertial Odometry (SLAM)
**Performance:** 1000-2000 Hz processing

**Key Files:**
| File | Purpose |
|------|---------|
| `src/laser_mapping.cc` | Main SLAM algorithm implementation |
| `src/pointcloud_preprocess.cc` | LiDAR point cloud preprocessing |
| `src/options.cc` | Configuration options parsing |
| `app/run_mapping_online.cc` | Online mapping node |
| `app/run_mapping_offline.cc` | Offline mapping from bag files |

**Launch Files:**
- `mapping_hesai.launch` - For Hesai LiDAR
- `mapping_velodyne.launch` - For Velodyne LiDAR
- `mapping_ouster64.launch` - For Ouster LiDAR

**Published Topics:**
- `/cloud_registered` - Registered point cloud
- `/Odometry` - Robot odometry
- `/path` - Trajectory path

---

### unitree_ros1/src/HesaiLidar_ROS_2.0/

**Language:** C++
**Purpose:** Driver for Hesai LiDAR sensors

**Supported LiDARs:**
- Pandar40P, Pandar64
- PandarXT, PandarQT
- AT128, Pandar128

**Key Files:**
| File | Purpose |
|------|---------|
| `node/hesai_ros_driver_node.cc` | Main ROS node |
| `config/config.yaml` | LiDAR IP, port, frame settings |

**Configuration (config.yaml):**
```yaml
lidar:
  - driver:
      lidar_type: "PandarXT-32"
      msop_port: 2368        # Data port
      device_ip: "192.168.1.201"
      frame_id: "hesai_lidar"
```

---

## ROS2 Components

### unitree_ros2/src/nav2_cloud_bringup/

**Language:** Python
**Purpose:** Navigation and dog locomotion control

#### unitree_nav2.py - Dog Movement Controller

**Location:** `nav2_cloud_bringup/unitree_nav2.py`
**Class:** `CmdVelToUnitree`

**What it does:**
1. Subscribes to `/cmd_vel` (Twist messages from Nav2)
2. Converts to Unitree SDK Move() commands
3. Sends velocity commands to the Go2 robot via subprocess

**Parameters:**
```python
cmd_vel_topic: '/cmd_vel'      # Input topic
stream_rate_hz: 3.0            # Command frequency
stale_timeout: 0.25            # Stop if no commands
vx_max: 1.2                    # Max forward speed (m/s)
vy_max: 0.5                    # Max lateral speed (m/s)
vyaw_max: 2.5                  # Max rotation speed (rad/s)
```

**How Movement Works:**
```python
# Commands are sent via subprocess to avoid SDK conflicts
dog_client = SportClient()
dog_client.Init()
dog_client.Move(vx, vy, vyaw)  # Velocity control
```

#### move_dog.py - Direct Movement Script

**Location:** `nav2_cloud_bringup/move_dog.py`
**Purpose:** Standalone script for direct dog control

---

### unitree_ros2/src/yolo_ros/

**Language:** Python
**Purpose:** YOLOv8/v9 object detection with ROS2 integration

#### yolov8_node.py - 2D Object Detection

**Location:** `yolov8_ros/yolov8_ros/yolov8_node.py`
**Class:** `Yolov8Node`

**What it does:**
1. Loads YOLO model (yolov8m.pt or yolov9c-seg.pt)
2. Subscribes to camera images
3. Runs inference on each frame
4. Publishes detection results

**Parameters:**
```python
model: "yolov8m.pt"            # Model weights file
device: "cuda:0"               # GPU device (or "cpu")
threshold: 0.5                 # Confidence threshold
enable: True                   # Detection on/off
```

**Detection Output (DetectionArray):**
- `class_id` - Object class ID
- `class_name` - Object class name (e.g., "person", "bottle")
- `score` - Confidence score (0-1)
- `bbox` - 2D bounding box (center x, y, width, height)
- `mask` - Segmentation mask (if using seg model)
- `keypoints` - Pose keypoints (if using pose model)

**Subscribes:**
- `/camera/color/image_raw` - RGB camera images

**Publishes:**
- `/yolov8_node/detections` - DetectionArray messages

**Services:**
- `/yolov8_node/enable` - SetBool to toggle detection

---

#### detect_3d_node.py - 3D Object Localization

**Location:** `yolov8_ros/yolov8_ros/detect_3d_node.py`
**Class:** `Detect3DNode`

**What it does:**
1. Receives 2D detections + depth image
2. Projects 2D bounding boxes to 3D space
3. Transforms to robot base frame
4. Publishes 3D bounding boxes

**How 3D Projection Works:**
```python
# Get depth at bounding box center
z = depth_image[center_y, center_x] / 1000  # mm to meters

# Project to 3D using camera intrinsics
x = z * (center_x - cx) / fx
y = z * (center_y - cy) / fy

# Result: 3D position (x, y, z) in camera frame
```

**Subscribes:**
- `/camera/depth/image_rect_raw` - Depth image
- `/camera/depth/camera_info` - Camera calibration
- `/yolov8_node/detections` - 2D detections

**Publishes:**
- `/detect_3d_node/detections_3d` - DetectionArray with 3D boxes

---

### unitree_ros2/src/piper_ros/

**Language:** Python
**Purpose:** AgileX Piper 6-DOF robotic arm control

#### piper_ctrl_single_node.py - Arm Controller

**Location:** `piper/piper/piper_ctrl_single_node.py`
**Class:** `PiperRosNode`

**What it does:**
1. Connects to Piper arm via CAN bus
2. Publishes joint states at 200 Hz
3. Accepts end-effector pose commands
4. Controls gripper

**Parameters:**
```python
can_port: 'can0'               # CAN interface name
auto_enable: False             # Auto-enable motors on start
gripper_exist: True            # Gripper attached
rviz_ctrl_flag: False          # RViz control mode
```

**Published Topics:**
- `/joint_states_single` - 7 joint angles (6 arm + 1 gripper)
- `/arm_status` - Motor status, errors, limits
- `/end_pose` - End-effector pose (position + quaternion)

**Subscribed Topics:**
- `/pos_cmd` - End-effector pose command (PosCmd message)
- `/enable_flag` - Enable/disable arm motors

**Services:**
- `/piper/enable_srv` - Enable/disable motors with timeout

**Arm Control Methods:**
```python
# Joint control (angles in degrees * 1000)
piper.JointCtrl(j0, j1, j2, j3, j4, j5)

# End-effector pose control (mm * 1000, degrees * 1000)
piper.EndPoseCtrl(x, y, z, rx, ry, rz)

# Gripper control
piper.GripperCtrl(angle, force, mode, extra)
# angle: 0-80000 (closed to open)
# force: grip force
# mode: 0x01=position, 0x02=force
```

---

#### Shell Scripts for CAN Bus

**can_activate.sh:**
```bash
# Activate CAN interface
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```

**find_all_can_port.sh:**
```bash
# Auto-detect CAN ports
ip link show | grep can
```

---

### unitree_ros2/src/unitree_msg_converter/

**Language:** Python
**Purpose:** Convert between Unitree proprietary messages and standard ROS messages

#### imu_converter.py - IMU Message Converter

**Location:** `unitree_msg_converter/imu_converter.py`
**Class:** `LowstateToImu`

**What it does:**
1. Subscribes to Unitree `/lowstate` messages
2. Extracts IMU data (quaternion, gyroscope, accelerometer)
3. Publishes standard `sensor_msgs/Imu` messages

**Subscribes:**
- `/lowstate` - Unitree LowState (proprietary)

**Publishes:**
- `/imu/data` - Standard IMU message

**Data Conversion:**
```python
# Unitree LowState contains:
# - imu_state.quaternion[4]    -> orientation
# - imu_state.gyroscope[3]     -> angular_velocity
# - imu_state.accelerometer[3] -> linear_acceleration
```

---

#### relay_joint.py - Joint State Relay

**Purpose:** Maps gripper angles to prismatic joint goals for URDF visualization

#### tf_pub.py - TF Publisher

**Purpose:** Publishes static transforms between coordinate frames

---

### unitree_ros2/src/realsense-ros/

**Language:** C++
**Purpose:** Intel RealSense RGB-D camera driver

**Key Files:**
| File | Purpose |
|------|---------|
| `src/base_realsense_node.cpp` | Main camera node |
| `src/ros_sensor.cpp` | Sensor abstraction |
| `launch/rs_launch.py` | Launch configuration |

**Published Topics:**
- `/camera/color/image_raw` - RGB image (640x480)
- `/camera/depth/image_rect_raw` - Depth image
- `/camera/depth/camera_info` - Camera calibration
- `/camera/aligned_depth_to_color/image_raw` - Aligned depth

**Launch Parameters:**
```python
depth_module.profile: '640x480x6'    # Resolution @ 6fps
rgb_camera.profile: '640x480x6'
enable_sync: True
pointcloud.enable: True
```

---

### unitree_ros2/cyclonedds_ws/

**Language:** C++
**Purpose:** DDS middleware and Unitree message definitions

**Key Components:**
- `cyclonedds/` - CycloneDDS implementation
- `rmw_cyclonedds/` - ROS2 middleware wrapper
- `unitree/` - Unitree message types:
  - `unitree_api/` - Request/Response messages
  - `unitree_go/` - Go2 robot messages (LowState, etc.)
  - `unitree_hg/` - H1 humanoid messages

---

## Configuration Files

### Nav2 Parameters

**Location:** `unitree_ros2/src/nav2_cloud_bringup/launch/nav2_params_online_fasterlio.yaml`

**Key Parameters:**
```yaml
controller_server:
  FollowPath:
    plugin: "dwb_core::DWBLocalPlanner"
    max_vel_x: 0.5          # Max forward speed
    max_vel_theta: 0.5      # Max rotation speed
    acc_lim_x: 2.5          # Acceleration limit

local_costmap:
  obstacle_layer:
    observation_sources: scan
    scan:
      topic: /cloud_registered
      max_obstacle_height: 2.0
      clearing: True
```

---

## Key Python Files Reference

| File | Location | Purpose |
|------|----------|---------|
| `launch_ros.py` | Root | Main system launcher |
| `cleanup_ros.py` | Root | Process cleanup utility |
| `unitree_nav2.py` | nav2_cloud_bringup | Dog velocity control |
| `move_dog.py` | nav2_cloud_bringup | Direct dog movement |
| `yolov8_node.py` | yolov8_ros | 2D object detection |
| `detect_3d_node.py` | yolov8_ros | 3D object localization |
| `tracking_node.py` | yolov8_ros | Object tracking |
| `debug_node.py` | yolov8_ros | Detection visualization |
| `piper_ctrl_single_node.py` | piper | Arm control node |
| `imu_converter.py` | unitree_msg_converter | IMU message conversion |
| `tf_pub.py` | unitree_msg_converter | TF publishing |
| `relay_joint.py` | unitree_msg_converter | Joint state relay |

---

## ROS Topics & Services

### Control Topics

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/cmd_vel` | geometry_msgs/Twist | Subscribe | Dog velocity commands |
| `/goal_pose` | geometry_msgs/PoseStamped | Subscribe | Navigation goal |
| `/pos_cmd` | piper_msgs/PosCmd | Subscribe | Arm end-effector command |
| `/enable_flag` | std_msgs/Bool | Subscribe | Arm enable/disable |

### State Topics

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/lowstate` | unitree_go/LowState | Publish | Dog state (IMU, motors) |
| `/imu/data` | sensor_msgs/Imu | Publish | Converted IMU data |
| `/joint_states_single` | sensor_msgs/JointState | Publish | Arm joint angles |
| `/end_pose` | geometry_msgs/Pose | Publish | Arm end-effector pose |
| `/arm_status` | piper_msgs/PiperStatusMsg | Publish | Arm status/errors |

### Perception Topics

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/camera/color/image_raw` | sensor_msgs/Image | Publish | RGB camera |
| `/camera/depth/image_rect_raw` | sensor_msgs/Image | Publish | Depth image |
| `/yolov8_node/detections` | yolov8_msgs/DetectionArray | Publish | 2D detections |
| `/detect_3d_node/detections_3d` | yolov8_msgs/DetectionArray | Publish | 3D detections |
| `/cloud_registered` | sensor_msgs/PointCloud2 | Publish | LiDAR point cloud |

### Services

| Service | Type | Purpose |
|---------|------|---------|
| `/piper/enable_srv` | piper_msgs/Enable | Enable/disable arm |
| `/yolov8_node/enable` | std_srvs/SetBool | Enable/disable detection |

---

## Hardware Interfaces

### Unitree Go2W Dog

**Connection:** Ethernet (eth0)
**IP:** Configured via DHCP or static
**SDK:** unitree_sdk2py

```python
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

ChannelFactoryInitialize(0, "eth0")
dog = SportClient()
dog.Init()
dog.Move(vx, vy, vyaw)  # Velocity control
```

### AgileX Piper Arm

**Connection:** CAN Bus (can0)
**Baud Rate:** 1,000,000 bps
**SDK:** piper-sdk

```python
from piper_sdk import C_PiperInterface

arm = C_PiperInterface(can_name="can0")
arm.ConnectPort()
arm.EnableArm(7)  # Enable all 7 axes
arm.JointCtrl(j0, j1, j2, j3, j4, j5)  # Joint angles
arm.EndPoseCtrl(x, y, z, rx, ry, rz)   # Cartesian
arm.GripperCtrl(angle, force, mode, 0)  # Gripper
```

### Intel RealSense

**Connection:** USB 3.0
**Models Supported:** D400 series, T265

### Hesai LiDAR

**Connection:** Ethernet
**Default IP:** 192.168.1.201
**Data Port:** 2368

---

## Build & Run Commands

### Initial Setup

```bash
# Install ROS dependencies
sudo apt install ros-foxy-nav2-* ros-foxy-cv-bridge ros-foxy-tf2-ros

# Install Python dependencies
pip3 install piper-sdk>=0.6.0 python-can>=4.0.0 ultralytics

# Build ROS1 workspace
cd unitree_ros1
catkin_make

# Build ROS2 workspace
cd unitree_ros2
colcon build
```

### Running the System

```bash
# Full system launch
python3 launch_ros.py robot_config.yaml

# With custom config
python3 launch_ros.py my_config.yaml

# Cleanup after crash
python3 launch_ros.py --cleanup
# or
python3 cleanup_ros.py
```

### CAN Bus Setup (for arm)

```bash
# Activate CAN interface
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# Verify
ip link show can0
```

---

## Extension Points for Assistive Tech

### Option A: Use Maestro (Recommended for Complex Tasks)

For sophisticated assistive tasks that require natural language understanding, planning, and error recovery, **use Maestro** as your high-level controller:

```
┌────────────────────────────────────────────────────────┐
│                   YOUR APPLICATION                      │
│                                                        │
│   User Interface (Voice/App/Button)                    │
│              │                                         │
│              ▼                                         │
│   ┌─────────────────────────────────┐                 │
│   │         MAESTRO                  │                 │
│   │   - Interprets user requests     │                 │
│   │   - Plans multi-step tasks       │                 │
│   │   - Handles failures/replanning  │                 │
│   │   - Uses VLM for reasoning       │                 │
│   └─────────────────────────────────┘                 │
│              │                                         │
│              ▼                                         │
│   This Codebase (ROS modules)                         │
└────────────────────────────────────────────────────────┘
```

**When to use Maestro:**
- Tasks requiring multi-step planning ("get X from Y and bring it here")
- Tasks needing visual reasoning ("find the red object")
- Tasks that may fail and need replanning
- Natural language interfaces

**Setup:**
1. Run this codebase's ROS system: `python3 launch_ros.py robot_config.yaml`
2. Run Maestro with connection to the ROS topics
3. Maestro generates Python code that calls ROS topics/services

### Option B: Write Custom ROS Nodes (Simpler Tasks)

For simpler, well-defined tasks, write custom ROS2 nodes directly:

### 1. Create a New Assistive Task Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from yolov8_msgs.msg import DetectionArray
from geometry_msgs.msg import PoseStamped, Twist
from piper_msgs.msg import PosCmd

class AssistiveTaskNode(Node):
    def __init__(self):
        super().__init__('assistive_task')

        # Subscribe to detections
        self.detection_sub = self.create_subscription(
            DetectionArray,
            '/yolov8_node/detections',
            self.detection_callback,
            10
        )

        # Publishers for control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.arm_pub = self.create_publisher(PosCmd, '/pos_cmd', 10)

    def detection_callback(self, msg):
        for detection in msg.detections:
            if detection.class_name == "bottle":
                self.get_logger().info(f"Found bottle at confidence {detection.score}")
                # Navigate to object, pick it up, etc.

def main():
    rclpy.init()
    node = AssistiveTaskNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Add Voice Commands

```python
# Install: pip3 install vosk sounddevice

import vosk
import sounddevice as sd

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command')
        self.model = vosk.Model("model")
        self.recognizer = vosk.KaldiRecognizer(self.model, 16000)

    def process_audio(self, audio_data):
        if self.recognizer.AcceptWaveform(audio_data):
            result = self.recognizer.Result()
            text = json.loads(result)["text"]

            if "fetch" in text and "water" in text:
                self.fetch_object("bottle")
            elif "come here" in text:
                self.navigate_to_user()
```

### 3. Person Following Mode

```python
class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__('person_follower')
        self.create_subscription(
            DetectionArray,
            '/detect_3d_node/detections_3d',
            self.follow_person,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_distance = 1.5  # meters

    def follow_person(self, msg):
        for det in msg.detections:
            if det.class_name == "person":
                # Get 3D position of person
                x = det.bbox3d.center.position.x
                z = det.bbox3d.center.position.z

                # Calculate velocity to maintain distance
                distance = math.sqrt(x**2 + z**2)
                error = distance - self.target_distance

                cmd = Twist()
                cmd.linear.x = 0.3 * error  # Proportional control
                cmd.angular.z = -0.5 * math.atan2(x, z)  # Turn towards person

                self.cmd_pub.publish(cmd)
```

### 4. Object Retrieval Pipeline

```
1. User requests object (voice/app/button)
2. YOLO detects object in camera view
3. 3D node localizes object position
4. Nav2 navigates robot to object
5. Piper arm picks up object
6. Robot returns to user
7. Arm delivers object
```

### 5. Custom YOLO Model Training

Train for assistive-specific objects:
- Wheelchairs, walkers, canes
- Medication bottles
- Common household items
- User's personal items

```bash
# Train custom model
yolo train model=yolov8n.pt data=assistive_objects.yaml epochs=100
```

---

## Troubleshooting

### Common Issues

**1. "Cannot connect to rosmaster"**
- ROS1 core not running
- Run: `roscore` in separate terminal

**2. "CAN interface not found"**
- CAN not activated
- Run: `sudo ip link set up can0`

**3. "No images from RealSense"**
- USB bandwidth issue
- Try different USB port or lower resolution

**4. "YOLO out of memory"**
- Set `device: "cpu"` in robot_config.yaml
- Or reduce image resolution

**5. "Nav2 not responding"**
- Check if SLAM is providing odometry
- Verify `/cloud_registered` topic is publishing

### Debug Commands

```bash
# Check ROS2 topics
ros2 topic list
ros2 topic echo /yolov8_node/detections

# Check transforms
ros2 run tf2_tools view_frames

# View camera
ros2 run rqt_image_view rqt_image_view

# Monitor CAN bus
candump can0
```

---

## Contact & Resources

### Primary Resources
- **Maestro (VLM Robot Control):** https://maestro-robot.github.io/
- **Unitree SDK Documentation:** https://github.com/unitreerobotics
- **Piper SDK:** https://github.com/agilexrobotics/piper_sdk

### Supporting Technologies
- **Nav2 Documentation:** https://navigation.ros.org/
- **YOLOv8 Documentation:** https://docs.ultralytics.com/
- **ROS2 Foxy:** https://docs.ros.org/en/foxy/
- **Intel RealSense:** https://github.com/IntelRealSense/realsense-ros

### The Full Stack

```
┌─────────────────────────────────────────────────────────────────┐
│  YOUR ASSISTIVE TECH APPLICATION                                │
│  (Voice interface, mobile app, accessibility controls)          │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│  MAESTRO - VLM Policy Planner                                   │
│  https://maestro-robot.github.io/                               │
│  - Natural language understanding                               │
│  - Task planning & code generation                              │
│  - Closed-loop execution with replanning                        │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│  THIS CODEBASE - unitree_go2w_agent_sdk                         │
│  - ROS1/ROS2 bridge infrastructure                              │
│  - Perception modules (YOLO, 3D detection, RealSense)           │
│  - Planning modules (Nav2, Faster-LIO SLAM)                     │
│  - Control modules (dog movement, arm control)                  │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│  HARDWARE                                                       │
│  - Unitree Go2W quadruped robot                                 │
│  - AgileX Piper 6-DOF arm + gripper                            │
│  - Intel RealSense RGB-D camera                                 │
│  - Hesai LiDAR                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

*This documentation is for internal BCIT Make+ use. Last updated January 2025.*

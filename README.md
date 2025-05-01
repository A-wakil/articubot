Here’s an updated **README.md** that assumes **slam_toolbox** and **Nav2** are installed via apt (not in your workspace repo). Just replace `<distro>` with your ROS 2 distribution (e.g. `humble` or `foxy`).

```markdown
# ROS 2 Waffle Bot Workspace

This workspace contains your custom waffle-bot packages (control, config, launch, etc.) and hooks into RPLIDAR, SLAM (slam_toolbox) and Nav2 for full autonomous navigation.

---

## 🔧 Prerequisites

1. **OS**  
   - Ubuntu 20.04 (Focal) or 22.04 (Jammy)

2. **ROS 2**  
   - ROS 2 `<distro>` (e.g. Humble Hawksbill)

3. **ROS 2 Packages**  
   Install drivers, SLAM & Nav2 from binaries:
   ```bash
   sudo apt update
   sudo apt install \
     ros-<distro>-rplidar-ros \
     ros-<distro>-slam-toolbox \
     ros-<distro>-nav2-bringup \
     ros-<distro>-nav2-amcl \
     ros-<distro>-nav2-behavior-tree \
     ros-<distro>-nav2-lifecycle-manager
   ```

4. **Build Tools**  
   ```bash
   sudo apt install python3-colcon-common-extensions python3-rosdep
   ```

5. **Hardware**  
   - RPLIDAR A1 (USB)  
   - Raspberry Pi / Jetson Orin Nano (4 GB+)  
   - Arduino Mega (wheel encoders)

---

## 📥 Setup Your Workspace

```bash
# 1. Create workspace & clone *your* packages only
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <your-repo-url>  # auto_waffle, config/, etc.

# 2. Return to workspace root
cd ~/ros2_ws
```

---

## 🔨 Build

```bash
# 1. Install any missing ROS deps
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 2. Build your packages
colcon build --symlink-install
```

---

## 🚀 Environment

Add to your `~/.bashrc` (or run in each terminal):

```bash
# Source ROS 2
source /opt/ros/<distro>/setup.bash

# Source your workspace
source ~/ros2_ws/install/setup.bash
```

---

## 📡 Launch & Run

1. **RPLIDAR driver**  
   ```bash
   ros2 launch rplidar_ros rplidar_a1_launch.py
   ```

2. **Waffle bringup**  
   ```bash
   ros2 launch waffle rsp.launch.py
   ros2 run auto_waffle waffle_bot_node
   ```

3. **Online SLAM (slam_toolbox)**  
   ```bash
   ros2 launch slam_toolbox online_async_launch.py \
     params_file:=~/ros2_ws/src/auto_waffle/src/config/mapper_params_online_async.yaml
   ```

4. **Navigation (Nav2)**  
   ```bash
   ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false
   ```

---

## 🖥 Visualization

```bash
ros2 run rviz2 rviz2
```

- Optionally load a saved config:  
  `File → Load Config → ~/ros2_ws/src/auto_waffle/config/waffle_config.rviz`
- Common displays:
  - **LaserScan** → `/scan`
  - **Map** → `/map`
  - **RobotModel** → `robot_description`
  - **Path** → `/plan`
  - **Odometry** → `/odom`

---

## 📝 Tips & Troubleshooting

- **TF Extrapolation Errors**  
  Ensure your odom→base_link and base_link→laser_frame TFs are published with synced timestamps.
- **Costmap Stuck**  
  Verify `/scan`, `/odom`, and `/tf` topics match your `use_sim_time` setting.
- **Missing SLAM/Navigation nodes**  
  Confirm `slam_toolbox` and `nav2_bringup` installed via apt:  
  ```bash
  apt list --installed | grep ros-<distro>-(slam-toolbox|nav2-bringup)
  ```

---

## 📂 Repo Structure

```
ros2_ws/
├── src/
│   └── auto_waffle/
│       ├── launch/
│       │   └── rsp.launch.py
│       ├── src/
│       │   └── config/                
│       │       └── mapper_params_online_async.yaml
│       └── package.xml
└── install/  # generated after build
```

---

## 🙌 Contributing

1. Fork this repo  
2. Create a branch (`feat/…`)  
3. Commit & push  
4. Open a PR

---

## 📄 License

MIT — see `LICENSE` for details.

---

*Happy mapping & navigating!*  
```

Feel free to tweak any paths or package names to match your setup.

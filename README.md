# ROS 1 Mecanum Navigation Stack

This document summarizes the **latest, recommended ROS 1 (Noetic) navigation stack** for a **4-mecanum robot with RPLIDAR**, aligned with the **current code structure, headless Raspberry Pi operation, tmux bringup, and RViz-on-laptop workflow**.

---

## 1. System Overview

### High-level data flow (authoritative)

```
RViz (Laptop)
   │  /move_base_simple/goal
   ▼
move_base  (Robot)
   │  /cmd_vel
   ▼
cmd_vel_to_wheels.py
   │  wheel velocities
   ▼
Motor Driver / MCU
   │  encoder ticks
   ▼
mecanum_odometry.py
   │  /odom  +  TF (odom → base_link)
   ▼
AMCL  ◄────────── map_server
   │  TF (map → odom)
   ▼
move_base
   ▲
   │  /scan
RPLIDAR
```

---

## 2. Required ROS Packages (Current Stack)

### Install set (Noetic)

```
ros-noetic-navigation
ros-noetic-amcl
ros-noetic-map-server
ros-noetic-gmapping
ros-noetic-teb-local-planner
ros-noetic-rplidar-ros
ros-noetic-tf2-tools
ros-noetic-robot-state-publisher
ros-noetic-teleop-twist-keyboard
```

> `teb_local_planner` is recommended for **mecanum / holonomic motion**.

---

## 3. Custom Robot Nodes (Your Code)

| Node | Purpose |
|-----|--------|
| `cmd_vel_to_wheels.py` | Convert `/cmd_vel` → 4 mecanum wheel speeds |
| `mecanum_odometry.py` | Encoder ticks → `/odom` + TF |
| `static_tf.launch` | Publish `base_link → laser` |

---

## 4. TF Tree (MANDATORY & VERIFIED)

```
map
 └── odom
      └── base_link
           └── laser
```

Validation:
```bash
rosrun tf2_tools view_frames.py
```

If this tree is broken → **navigation will not work**.

---

## 5. Operating Modes (Updated)

There are **two supported modes**:

| Mode | Purpose |
|----|-------|
| Mapping | Create static map (run once) |
| Navigation | Daily autonomous operation |

---

## MODE 1 — Mapping (NO AMCL, NO move_base)

### Nodes launched

```
roscore
```

```
roslaunch rplidar_ros rplidar.launch
```

```
rosrun mecanum_base_controller mecanum_odometry.py
```

```
roslaunch mecanum_base_controller static_tf.launch
```

```
roslaunch mecanum_base_controller gmapping.launch
```

### Manual driving

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Drive slowly, cover all reachable space.

### Save map

```
rosrun map_server map_saver -f ~/catkin_ws/src/mecanum_base_controller/maps/map
```

Creates:
```
map.yaml
map.pgm
```

❗ Stop `gmapping` after saving.

---

## MODE 2 — Navigation (Daily Use)

### Launch order (authoritative)

#### 1. Core
```
roscore
```

#### 2. Sensors
```
roslaunch rplidar_ros rplidar.launch
```

#### 3. Robot base
```
rosrun mecanum_base_controller mecanum_odometry.py
```

```
roslaunch mecanum_base_controller static_tf.launch
```

```
rosrun mecanum_base_controller cmd_vel_to_wheels.py
```

#### 4. Localization
```
roslaunch mecanum_base_controller amcl.launch
```

#### 5. Navigation
```
roslaunch mecanum_base_controller move_base.launch
```

> `move_base` performs **both path planning and obstacle avoidance** via global + local planners.

---

## 6. RViz (Laptop-Side)

```
export ROS_MASTER_URI=http://<ROBOT_IP>:11311
export ROS_IP=<LAPTOP_IP>
rviz
```

RViz steps:
1. Fixed Frame → `map`
2. **2D Pose Estimate** (initialize AMCL)
3. **2D Nav Goal** (send waypoint)

---

## 7. Health & Debug Checklist

### Topics
```
rostopic echo /scan
rostopic echo /odom
rostopic echo /cmd_vel
rostopic echo /amcl_pose
```

### TF
```
rosrun tf tf_echo map base_link
```

### move_base
```
rosnode info /move_base
```

---

## 8. Headless Raspberry Pi Notes

- No GUI required on robot
- RViz runs on laptop only
- tmux used for persistent bringup
- Multiple SSH sessions supported

---

## 9. Authoritative Rule Set

- Only **one** `/cmd_vel` publisher
- Only **one** `roscore`
- Never run `gmapping` and `amcl` together
- TF must match section 4 exactly

---

## 10. Mental Model (Final)

```
Map → AMCL → move_base → cmd_vel → wheels → odom → TF → AMCL
```

If this loop is closed → navigation works.

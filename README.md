# ROS 1 Mecanum Navigation Stack – Summary & Runbook

This document summarizes the **complete ROS 1 (Noetic) navigation stack** for a **real 4-mecanum robot with RPLIDAR**, including **what each component does** and **what to run in each mode**.

---

## 1. System Overview

### High-level data flow

```
            move_base
                │ /cmd_vel
                ▼
      cmd_vel_to_wheels.py
                │ wheel speeds
                ▼
        Motor Driver / MCU
                │ encoder ticks
                ▼
        mecanum_odometry.py
                │ /odom + TF
                ▼
               AMCL  ◄──── map_server
                │ TF (map → odom)
                ▼
            move_base
                ▲
                │ /scan
             RPLIDAR
```

---

## 2. Required ROS Nodes & Packages

### Core robot nodes (custom)

| Node                         | Purpose                                   |
| ---------------------------- | ----------------------------------------- |
| `cmd_vel_to_wheels.py`       | Convert `/cmd_vel` → mecanum wheel speeds |
| `mecanum_odometry.py`        | Convert encoder ticks → `/odom` + TF      |
| `static_transform_publisher` | Publish `base_link → laser`               |

### Standard ROS nodes

| Node          | Purpose                   |
| ------------- | ------------------------- |
| `rplidar_ros` | Publish `/scan`           |
| `map_server`  | Serve static map          |
| `amcl`        | Lidar-based localization  |
| `move_base`   | Global + local navigation |

---

## 3. TF Tree (MANDATORY)

The TF tree **must look exactly like this**:

```
map
 └── odom
      └── base_link
           └── laser
```

Check anytime with:

```bash
rosrun tf view_frames
```

---

## 4. Operating Modes

There are **two modes**: **Mapping** (run once) and **Navigation** (daily use).

---

## MODE 1 — Mapping (Run Once)

Used only to **create the map**.

### Nodes to run (order matters)

```bash
roscore
```

```bash
roslaunch rplidar_ros rplidar.launch
```

```bash
rosrun mecanum_base_controller mecanum_odometry.py
```

```bash
roslaunch mecanum_base_controller static_tf.launch
```

```bash
roslaunch mecanum_base_controller gmapping.launch
```

### Drive the robot

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Drive **slowly**, cover the whole area, avoid fast spins.

### Save the map

```bash
rosrun map_server map_saver -f ~/catkin_ws/src/mecanum_base_controller/maps/map
```

This creates:

```
map.pgm
map.yaml
```

🚫 **Stop `gmapping` after saving the map**.

---

## MODE 2 — Localization + Navigation (Normal Operation)

Used for **daily autonomous navigation**.

### Nodes to run (order matters)

#### 1. ROS core

```bash
roscore
```

#### 2. Sensors

```bash
roslaunch rplidar_ros rplidar.launch
```

#### 3. Robot base

```bash
rosrun mecanum_base_controller mecanum_odometry.py
```

```bash
roslaunch mecanum_base_controller static_tf.launch
```

```bash
rosrun mecanum_base_controller cmd_vel_to_wheels.py
```

#### 4. Localization

```bash
roslaunch mecanum_base_controller amcl.launch
```

#### 5. Navigation

```bash
roslaunch mecanum_base_controller move_base.launch
```

#### 6. Visualization

```bash
rviz
```

---

## 5. RViz Workflow

1. Set **Fixed Frame** → `map`
2. Click **2D Pose Estimate** (initialize localization)
3. Click **2D Nav Goal** (send navigation goal)
4. Robot navigates autonomously

---

## 6. Quick Health Checks

```bash
rostopic list
```

```bash
rostopic echo /cmd_vel
```

```bash
rostopic echo /odom
```

```bash
rostopic echo /scan
```

```bash
rosrun tf tf_echo map base_link
```

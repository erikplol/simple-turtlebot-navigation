#!/usr/bin/env bash

# =============================================================
# ROS1 NOETIC NAVIGATION STACK INSTALL SCRIPT
# Target: Raspberry Pi / Ubuntu (Headless OK)
# =============================================================

set -e

ROS_DISTRO=noetic

echo "=============================================="
echo " Installing ROS1 Navigation Requirements"
echo " ROS Distro: $ROS_DISTRO"
echo "=============================================="

# -------------------------------------------------------------
# 1. Update system
# -------------------------------------------------------------
echo "[1/6] Updating system..."
sudo apt update && sudo apt upgrade -y

# -------------------------------------------------------------
# 2. Core ROS navigation packages
# -------------------------------------------------------------
echo "[2/6] Installing core navigation packages..."
sudo apt install -y \
  ros-$ROS_DISTRO-navigation \
  ros-$ROS_DISTRO-move-base \
  ros-$ROS_DISTRO-map-server \
  ros-$ROS_DISTRO-amcl

# -------------------------------------------------------------
# 3. Local planners (TEB recommended for mecanum)
# -------------------------------------------------------------
echo "[3/6] Installing local planners..."
sudo apt install -y \
  ros-$ROS_DISTRO-teb-local-planner

# -------------------------------------------------------------
# 4. SLAM (for mapping)
# -------------------------------------------------------------
echo "[4/6] Installing SLAM packages..."
sudo apt install -y \
  ros-$ROS_DISTRO-gmapping

# -------------------------------------------------------------
# 5. Sensors & utilities
# -------------------------------------------------------------
echo "[5/6] Installing sensor & utility packages..."
sudo apt install -y \
  ros-$ROS_DISTRO-rplidar-ros \
  ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-tf2-ros \
  ros-$ROS_DISTRO-tf2-tools \
  ros-$ROS_DISTRO-xacro

# -------------------------------------------------------------
# 6. Dev & runtime tools
# -------------------------------------------------------------
echo "[6/6] Installing dev & runtime tools..."
sudo apt install -y \
  tmux \
  git \
  python3-rosdep \
  python3-catkin-tools

# -------------------------------------------------------------
# rosdep init (safe to rerun)
# -------------------------------------------------------------
echo "Initializing rosdep..."
sudo rosdep init 2>/dev/null || true
rosdep update

# -------------------------------------------------------------
# Final message
# -------------------------------------------------------------
echo "=============================================="
echo " Installation complete ✅"
echo " Next steps:"
echo " 1) Create catkin workspace"
echo " 2) Build your mecanum packages"
echo " 3) Run headless tmux launcher"
echo "=============================================="

# 🤖 Multi-PC ROS 2 TurtleBot3 Control (Gazebo)

This project demonstrates **multi-PC, multi-robot control in ROS 2** using **TurtleBot3** simulated in **Gazebo**.

Two TurtleBot3 robots run on a **Host PC** in separate namespaces, while **two Client PCs** independently control each robot over a local network.

This setup is **production-grade** and directly scales to **Isaac Sim / Omniverse**.

---

## 📋 Prerequisites

### Hardware / OS
- Ubuntu 22.04 (recommended)
- All PCs connected to the **same local network** (Wi-Fi or Ethernet)

### Software (ALL PCs)
- ROS 2 **Humble** (or newer)
- TurtleBot3 packages

### Software (Host PC only)
- Gazebo (installed with TurtleBot3 packages)

---

## 🛠 Step 1: Install TurtleBot3 (ALL PCs)

```bash
sudo apt update
sudo apt install ros-humble-turtlebot3*
```

Set TurtleBot3 model:
```bash
export TURTLEBOT3_MODEL=burger
```

(Optional – make permanent)
```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
```

---

## 🌐 Step 2: Network Configuration

### 1️⃣ Find Host PC IP
On the **Host PC**:
```bash
hostname -I
```

Example:
```
192.168.1.50
```

---

### 2️⃣ Ping Test
From each **Client PC**:
```bash
ping 192.168.1.50
```

Ping replies confirm basic connectivity ✅

---

### 3️⃣ Set ROS Domain ID (ALL PCs)

```bash
export ROS_DOMAIN_ID=42
```

(Optional – permanent)
```bash
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
```

---

### 4️⃣ DDS Recommendation (Highly Recommended)

Use **CycloneDDS** for stability (especially Wi-Fi & VMs):

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

(Optional – permanent)
```bash
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```

---

## 🚀 Step 3: Launch Multi-TurtleBot3 Simulation (Host PC)

Open **two terminals** on the **Host PC**.

### 🤖 Terminal 1 — TurtleBot Alpha (`/tb1`)
```bash
ROS_NAMESPACE=tb1 ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 🤖 Terminal 2 — TurtleBot Bravo (`/tb2`)
```bash
ROS_NAMESPACE=tb2 ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

---

## 🎮 Step 4: Remote Control from Client PCs

### 🕹 Client PC #1 — Control TurtleBot Alpha
```bash
ros2 run turtlebot3_teleop teleop_keyboard \
  --ros-args -r cmd_vel:=/tb1/cmd_vel
```

### 🕹 Client PC #2 — Control TurtleBot Bravo
```bash
ros2 run turtlebot3_teleop teleop_keyboard \
  --ros-args -r cmd_vel:=/tb2/cmd_vel
```

---

## 🔍 Verification & Debugging

```bash
ros2 topic list
```

Expected topics:
```
/tb1/cmd_vel
/tb1/odom
/tb1/scan
/tb2/cmd_vel
/tb2/odom
/tb2/scan
```

---

## 📜 License
MIT License

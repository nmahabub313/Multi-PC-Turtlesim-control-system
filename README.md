This README is designed to be a complete, copy-pasteable guide for your project. It covers the environment setup, the specific namespacing commands, and the network verification steps we discussed.
Multi-PC ROS 2 Turtlesim Control
This project demonstrates how to run multiple turtlesim instances in different namespaces on a single Host PC and control them individually from separate Client PCs over a local network.
📋 Prerequisites
 * ROS 2 Installed (Humble, Iron, or Jazzy) on all machines.
 * All PCs must be connected to the same local network.
 * Basic understanding of the Linux terminal.
🛠 Step 1: Network Configuration
Before launching ROS 2, ensure the machines can communicate.
 * Find the Host IP:
   On the Host PC, run hostname -I. Let's assume it is 192.168.1.50.
 * Ping Test:
   From a Client PC, run: ping 192.168.1.50.
 * Set Domain ID:
   To ensure all nodes find each other, set the same Domain ID on all terminals on all PCs:
   export ROS_DOMAIN_ID=42

   (Add this to your ~/.bashrc to make it permanent).
🚀 Step 2: Launch Turtles on the Host PC
Open two separate terminals on the Host PC to launch the simulators in their own namespaces.
Terminal 1: Turtle Alpha
ros2 run turtlesim turtlesim_node --ros-args -r __node:=turtlesim_a -r __ns:=/t1

Terminal 2: Turtle Bravo
ros2 run turtlesim turtlesim_node --ros-args -r __node:=turtlesim_b -r __ns:=/t2

🎮 Step 3: Remote Control from Clients
From Client PC #1 (Control Turtle Alpha)
Run the teleop node and remap the velocity topic to match the /t1 namespace:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/t1/turtle1/cmd_vel

From Client PC #2 (Control Turtle Bravo)
Run the teleop node and remap the velocity topic to match the /t2 namespace:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/t2/turtle1/cmd_vel

🔍 Verification & Debugging
Verify Topics
On any PC, run the following to see the active namespaced topics:
ros2 topic list

Expected Output:
 * /t1/turtle1/cmd_vel
 * /t1/turtle1/pose
 * /t2/turtle1/cmd_vel
 * /t2/turtle1/pose
Troubleshooting
 * Firewall: If topics don't appear, try disabling the firewall on the Host: sudo ufw disable.
 * Multicast: Ensure your router allows UDP multicast.
 * RMW Implementation: Ensure all PCs are using the same RMW (e.g., rmw_cyclonedds_cpp or rmw_fastrtps_cpp).

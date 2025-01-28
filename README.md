
# **SmartCar Simulation and Teleoperation System**

This project is designed to simulate and control a SmartCar robot in a ROS 2 environment. It includes launching the robot in a Gazebo simulation, visualizing it in RViz, managing the map server, and enabling teleoperation through keyboard input.

---

## **Table of Contents**

- [Overview](#overview)
- [Features](#features)
- [Project Structure](#project-structure)
- [Setup](#setup)
- [Usage](#usage)
  - [1. Launching the Simulation](#1-launching-the-simulation)
  - [2. Visualizing in RViz](#2-visualizing-in-rviz)
  - [3. Teleoperation](#3-teleoperation)
- [Troubleshooting](#troubleshooting)
- [Future Enhancements](#future-enhancements)

---

## **Overview**

This project simulates a SmartCar robot in a small town environment. It supports the following functionalities:
- Visualizing the SmartCar URDF model in RViz.
- Simulating the SmartCar in a Gazebo world.
- Loading and managing a 2D map of the environment using the Nav2 map server.
- Controlling the SmartCar robot using a keyboard with the `teleop_twist_keyboard` package.

---

## **Features**

1. **Gazebo Integration**: Launch a simulated small town environment (`smalltown.world`) and spawn the SmartCar robot within it.
2. **RViz Visualization**: Visualize the SmartCar robot model and its transformations in RViz.
3. **Teleoperation**: Use the keyboard to manually control the robot's movements via the `cmd_vel` topic.
4. **Map Management**: Load a predefined map using the Nav2 map server.
5. **ROS 2 Launch System**: Easily manage all components through a single launch file.

---

## **Project Structure**

```
smartcar/
├── smartcar_description/      # Robot description package
│   ├── model/                 # Contains URDF and Xacro files
│   └── launch/                # RViz configuration files
├── smartcar_bringup/          # Launch files and world configuration
│   └── worlds/                # Gazebo world files
    └── launch/                #launch files for gazebo world
├── smartcar_map/              # Map server package
│   └── config/                # 2D map YAML and PGM files
    └── include/               #car gazebo plugin header
    └── src/                   #car gazebo plugin cpp

```

---

## **Setup**

### **Prerequisites**

1. Install ROS 2 (e.g., Humble, Foxy, or Rolling).
2. Ensure the following ROS 2 packages are installed:
   ```bash
   sudo apt install ros-humble-gazebo-ros \
                    ros-humble-joint-state-publisher-gui \
                    ros-humble-robot-state-publisher \
                    ros-humble-nav2-map-server \
                    ros-humble-teleop-twist-keyboard
   ```
   *(Replace `humble` with your ROS 2 version if necessary.)*

3. Clone this repository into your workspace:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone <repository_url>
   ```

4. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

---

## **Usage**

### **1. Launching the Simulation**

Start the simulation by running the main launch file:
```bash
ros2 launch smartcar_bringup gazebo.launch.py
```

This will:
- Start Gazebo with the `smalltown.world`.
- Load the SmartCar robot into the simulation.
- Start the map server.
- Open RViz for visualization.

### **2. Visualizing in RViz**

- RViz will open automatically with the robot's model loaded.
- If it doesn’t start, you can manually run:
  ```bash
  ros2 run rviz2 rviz2 -d ~/ros2_ws/src/smartcar_description/rviz/smartcar_config.rviz
  ```

### **3. Teleoperation**

To control the SmartCar using your keyboard:
1. Run the teleop node:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
2. Use the following keys to control the robot:
   ```
   Moving around:
           w
      a    s    d
           x

   w/x : increase/decrease linear velocity
   a/d : increase/decrease angular velocity
   space key, s : force stop
   ```

---

## **Troubleshooting**

1. **Robot Not Spawning in Gazebo:**
   - Check the terminal logs for errors related to the `spawn_entity.py` node.
   - Verify the URDF file path in the launch file.

2. **Robot Not Moving:**
   - Ensure the robot control node subscribes to the `cmd_vel` topic:
     ```bash
     ros2 topic echo /cmd_vel
     ```
   - If not, remap the topic in the teleop node:
     ```bash
     ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=<robot_cmd_vel_topic>
     ```

3. **RViz Not Showing the Robot:**
   - Verify the `robot_description` parameter in the `robot_state_publisher` node.
   - Ensure the `xacro` file generates a valid URDF.

---

## **Future Enhancements**

1. **Navigation Integration**: Add autonomous navigation capabilities with the Nav2 stack.
2. **Collision Avoidance**: Implement obstacle detection and avoidance.
3. **Sensor Integration**: Add LiDAR and camera sensors to enhance simulation realism.
4. **Multi-Robot Support**: Expand the system to support multiple robots in the environment.

---

## **License**

This project is licensed under the [MIT License](LICENSE).



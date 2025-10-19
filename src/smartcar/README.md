# SmartCAR ROS2 Package

This package contains the robot description, launch files, and configurations for the SmartCAR autonomous vehicle simulation based on the Robot Architecture assignment specifications.

## Package Structure

```
smartcar/
├── CMakeLists.txt
├── package.xml
├── smartcar/               # Python package
│   └── __init__.py
├── launch/
│   ├── smartcar.launch.py  # Basic robot state publisher launch
│   └── gazebo.launch.py    # Gazebo simulation launch (requires gazebo_ros)
├── urdf/
│   └── smartcar.urdf.xacro # Robot URDF description
├── rviz/
│   └── smartcar.rviz       # RViz configuration
└── worlds/
    └── empty.world         # Gazebo world file
```

## Robot Specifications

The SmartCAR robot follows exact specifications from the assignment:

### Physical Dimensions
- **Wheel diameter**: 0.064m 
- **Wheel width**: 0.025m
- **Wheelbase length**: 0.257m (distance between front and rear axles)
- **Wheelbase width**: 0.17m (distance between left and right wheels)

### Joint Configuration (Ackermann Steering)
- `back_left_wheel_joint`: Rear left wheel (continuous rotation)
- `back_right_wheel_joint`: Rear right wheel (continuous rotation)
- `front_left_wheel_steer_joint`: Front left steering joint (revolute)
- `front_left_wheel_joint`: Front left wheel (continuous rotation)
- `front_right_wheel_steer_joint`: Front right steering joint (revolute)
- `front_right_wheel_joint`: Front right wheel (continuous rotation)

### Sensors
- **IMU**: Located at robot center, publishes to `/imu_data` topic
- **LIDAR**: Located at front of robot with 0.05m elevation, publishes to `/scan` topic

### Frame Hierarchy
```
base_footprint (ground projection)
└── base_link (robot center)
    └── chassis_link (main body)
        ├── imu_link (IMU sensor)
        ├── slidar_base_link (LIDAR sensor)
        ├── front_left_steer_link
        │   └── front_left_wheel_link
        ├── front_right_steer_link
        │   └── front_right_wheel_link
        ├── back_left_wheel_link
        └── back_right_wheel_link
```

## Usage

### Basic Robot Visualization

1. **Build the packages**:
   ```bash
   cd /path/to/smartcar/workspace
   colcon build --packages-select smartcar_msgs smartcar
   source install/setup.bash
   ```

2. **Launch complete visualization system**:
   ```bash
   # Launch everything (default): Robot state publisher + Joint GUI + RViz + Foxglove bridge
   ros2 launch smartcar smartcar.launch.py
   
   # Launch with custom options:
   ros2 launch smartcar smartcar.launch.py use_rviz:=false  # Without RViz
   ros2 launch smartcar smartcar.launch.py use_foxglove:=false  # Without Foxglove
   ros2 launch smartcar smartcar.launch.py use_gui:=false  # Without joint GUI
   ```

### Published Topics

When running, the robot publishes the following topics:
- `/robot_description`: Robot URDF description
- `/joint_states`: Current joint positions (from joint_state_publisher)
- `/tf` and `/tf_static`: Transform tree
- `/imu_data`: IMU sensor data (when in Gazebo simulation)
- `/scan`: LIDAR scan data (when in Gazebo simulation)

### Interactive Control

The launch file includes `joint_state_publisher_gui` which allows you to:
- Manually control steering angles for front wheels
- Rotate all wheels to test the robot model
- Visualize different configurations in RViz

## Dependencies

This package depends on:
- `rclcpp`, `rclpy`: ROS2 C++ and Python client libraries
- `sensor_msgs`, `geometry_msgs`, `nav_msgs`: ROS2 message types
- `tf2`, `tf2_ros`: Transform handling
- `urdf`, `xacro`: Robot description processing
- `robot_state_publisher`: Publishes robot transforms
- `joint_state_publisher`: Provides joint states
- `smartcar_msgs`: Custom message definitions

## Custom Messages

The `smartcar_msgs` package defines:
- `Status.msg`: Robot telemetry with fields:
  - `battery_voltage_mv` (int32): Battery voltage in millivolts
  - `battery_current_ma` (int32): Battery current in milliamps  
  - `battery_percentage` (float32): Battery charge percentage (0-100)
  - `steering_angle_rad` (float32): Current steering angle in radians
  - `engine_speed_rpm` (float32): Engine speed in RPM

## Gazebo Simulation

For full Gazebo simulation (requires `gazebo_ros` packages):
```bash
ros2 launch smartcar gazebo.launch.py
```

This will:
- Start Gazebo with the empty world
- Spawn the SmartCAR robot
- Enable IMU and LIDAR sensor plugins
- Publish sensor data to appropriate topics

## Troubleshooting

### Common Issues

1. **"Could not find gazebo_ros"**: Install gazebo_ros packages or use the basic launch file
2. **"No transform from base_link"**: Ensure robot_state_publisher is running
3. **RViz not showing robot**: Check that robot_description topic is being published

### Verification Commands

```bash
# Check if robot description is published
ros2 topic echo /robot_description --once

# Verify joint states
ros2 topic echo /joint_states

# Check TF tree
ros2 run tf2_tools view_frames

# List all topics
ros2 topic list
```

## Development Notes

The robot model follows the assignment specifications exactly:
- All dimensions match the required measurements
- Joint names follow the specified convention
- Sensor placements meet the assignment requirements
- Frame hierarchy follows ROS2 best practices
- URDF includes proper inertial properties and collision geometries

For further development, additional components can be added:
- Wheel odometry calculation nodes
- EKF localization integration  
- Nav2 navigation stack configuration
- Custom control interfaces
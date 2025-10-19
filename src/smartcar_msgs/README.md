# SmartCAR Messages Package

This package defines custom ROS2 message types for the SmartCAR autonomous vehicle project.

## Package Contents

### Messages

#### Status.msg
Robot telemetry and status information message with the following fields:

```
int32 battery_voltage_mv     # Battery voltage in millivolts
int32 battery_current_ma     # Battery current in milliamps  
float32 battery_percentage   # Battery charge percentage (0-100)
float32 steering_angle_rad   # Current steering angle in radians
float32 engine_speed_rpm     # Engine speed in RPM
```

### Usage

To use these messages in your ROS2 nodes:

**C++ Example:**
```cpp
#include "smartcar_msgs/msg/status.hpp"

// Create and publish status message
auto status_msg = smartcar_msgs::msg::Status();
status_msg.battery_voltage_mv = 12000;  // 12.0V
status_msg.battery_current_ma = 2500;   // 2.5A
status_msg.battery_percentage = 85.5;   // 85.5%
status_msg.steering_angle_rad = 0.2;    // 0.2 rad
status_msg.engine_speed_rpm = 1500.0;   // 1500 RPM
```

**Python Example:**
```python
from smartcar_msgs.msg import Status

# Create and publish status message
status_msg = Status()
status_msg.battery_voltage_mv = 12000  # 12.0V
status_msg.battery_current_ma = 2500   # 2.5A
status_msg.battery_percentage = 85.5   # 85.5%
status_msg.steering_angle_rad = 0.2    # 0.2 rad
status_msg.engine_speed_rpm = 1500.0   # 1500 RPM
```

## Building

This package is built using `colcon`:

```bash
colcon build --packages-select smartcar_msgs
```

After building, source the workspace to use the messages:

```bash
source install/setup.bash
```

## Dependencies

- `rosidl_default_generators`: For message generation
- `rosidl_default_runtime`: For message runtime support
- `std_msgs`: Standard ROS2 message types

## Integration

This package is designed to work with the `smartcar` package and provides the message definitions needed for:
- Battery monitoring and management
- Steering angle feedback
- Engine/motor speed telemetry
- System status reporting
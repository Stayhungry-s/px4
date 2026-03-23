# px4

## offboard_control_ros2

This repository now includes a ROS2-based offboard module (`offboard_control_ros2`) for publishing PX4 offboard control messages.

### Features

- Periodically publishes `/fmu/in/offboard_control_mode`
- Periodically publishes `/fmu/in/trajectory_setpoint`
- Sends `/fmu/in/vehicle_command` to switch to OFFBOARD and arm after a configurable number of setpoints
- Subscribes to `/fmu/out/vehicle_status` to monitor flight controller state

### Files

- `offboard_control_ros2/offboard_control_ros2/offboard_node.py`
- `offboard_control_ros2/launch/offboard_control.launch.py`
- `offboard_control_ros2/config/offboard_params.yaml`

### Usage

```bash
cd <workspace_root>
colcon build --packages-select offboard_control_ros2
source install/setup.bash
ros2 launch offboard_control_ros2 offboard_control.launch.py
```

# Precision Landing Main Implementation

This document describes the main implementation in
`offboard_control_ros2/offboard_control_ros2/precision_landing_node.py`.

## 1. Node objective

`precision_landing_node` converts visual target offsets into PX4 OFFBOARD
trajectory setpoints and runs a simple **align + descend** precision-landing flow.

## 2. Inputs and outputs

### Input (vision side)

- Topic: `/vision/landing_target_offset`
- Type: `geometry_msgs/msg/PointStamped`
- Convention:
  - `point.x`: x offset from the vehicle to the landing target (meters)
  - `point.y`: y offset from the vehicle to the landing target (meters)

### Outputs (PX4 side)

- `/fmu/in/offboard_control_mode` (`px4_msgs/msg/OffboardControlMode`)
- `/fmu/in/trajectory_setpoint` (`px4_msgs/msg/TrajectorySetpoint`)
- `/fmu/in/vehicle_command` (`px4_msgs/msg/VehicleCommand`)

## 3. Control flow

1. Periodically publish OFFBOARD control mode and trajectory setpoints.
2. After a configured warmup count, send OFFBOARD mode switch and arm commands.
3. Based on visual offsets, run the following phases:
   - `ALIGN`: correct horizontal offset.
   - `DESCEND`: move altitude setpoint step-by-step toward landing height after alignment.
   - `HOLD`: keep current setpoint when target timeout occurs.

## 4. Key parameters

Parameter file:
`offboard_control_ros2/config/precision_landing_params.yaml`

- `align_threshold_m`: horizontal alignment threshold before descend.
- `descend_step_m`: altitude step size per control cycle.
- `max_horizontal_step_m`: max horizontal correction per cycle.
- `target_timeout_sec`: target freshness timeout.
- `max_setpoint_deviation_m`: max horizontal drift from hold origin.
- `approach_height_m` / `landing_height_m`: approach and landing target heights.

## 5. Run

```bash
cd <workspace_root>
colcon build --packages-select offboard_control_ros2
source install/setup.bash
ros2 launch offboard_control_ros2 precision_landing.launch.py
```

## 6. Integration notes

- The vision node only needs to publish stable target offsets.
- Tune parameters in simulation before flight tests.
- If the visual frame is not aligned with the control frame, apply a frame transform
  in the vision side or an intermediate ROS2 node first.

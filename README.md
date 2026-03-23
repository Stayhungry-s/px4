# px4

## offboard_control_ros2

本仓库新增了一个基于 ROS2 架构的 offboard 模块（`offboard_control_ros2`），用于通过 PX4 ROS2 接口发布 offboard 控制消息。

### 功能

- 周期发布 `/fmu/in/offboard_control_mode`
- 周期发布 `/fmu/in/trajectory_setpoint`
- 达到设定次数后发送 `/fmu/in/vehicle_command` 进入 OFFBOARD 并解锁
- 订阅 `/fmu/out/vehicle_status` 获取飞控状态

### 目录

- `/home/runner/work/px4/px4/offboard_control_ros2/offboard_control_ros2/offboard_node.py`
- `/home/runner/work/px4/px4/offboard_control_ros2/launch/offboard_control.launch.py`
- `/home/runner/work/px4/px4/offboard_control_ros2/config/offboard_params.yaml`

### 使用

```bash
cd /home/runner/work/px4/px4
colcon build --packages-select offboard_control_ros2
source install/setup.bash
ros2 launch offboard_control_ros2 offboard_control.launch.py
```

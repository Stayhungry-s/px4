# 视觉精准降落主实现说明

本文档对应主实现代码：`offboard_control_ros2/offboard_control_ros2/precision_landing_node.py`。

## 1. 节点目标

`precision_landing_node` 负责将视觉目标偏移量转换为 PX4 可直接使用的 OFFBOARD 轨迹设定点，实现“对准 + 下降”的精准降落控制流程。

## 2. 输入与输出

### 输入（视觉侧）

- 话题：`/vision/landing_target_offset`
- 类型：`geometry_msgs/msg/PointStamped`
- 约定：
  - `point.x`：目标相对机体的 x 偏移（米）
  - `point.y`：目标相对机体的 y 偏移（米）

### 输出（PX4 侧）

- `/fmu/in/offboard_control_mode` (`px4_msgs/msg/OffboardControlMode`)
- `/fmu/in/trajectory_setpoint` (`px4_msgs/msg/TrajectorySetpoint`)
- `/fmu/in/vehicle_command` (`px4_msgs/msg/VehicleCommand`)

## 3. 控制流程

1. 周期发布 OFFBOARD 控制模式和轨迹设定点；
2. 在预设次数后发送切换 OFFBOARD 与解锁命令；
3. 接收视觉偏移后进入：
   - `ALIGN`：先修正水平误差；
   - `DESCEND`：误差进入阈值后逐步下降；
   - `HOLD`：目标丢失超时则保持当前位置。

## 4. 关键参数

参数文件：`offboard_control_ros2/config/precision_landing_params.yaml`

- `align_threshold_m`：进入下降阶段的对准阈值
- `descend_step_m`：每次控制周期下降步长
- `max_horizontal_step_m`：水平每周期最大移动量
- `target_timeout_sec`：视觉目标超时判定
- `approach_height_m` / `landing_height_m`：接近高度与最低下降高度

## 5. 启动方式

```bash
cd <workspace_root>
colcon build --packages-select offboard_control_ros2
source install/setup.bash
ros2 launch offboard_control_ros2 precision_landing.launch.py
```

## 6. 对接建议

- 视觉节点只需稳定输出目标偏移即可；
- 建议先在仿真调参，再上实机；
- 若视觉坐标系与机体系不一致，请在视觉侧或中间转换节点先做坐标变换。

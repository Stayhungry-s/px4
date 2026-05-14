from __future__ import annotations

from typing import List, Optional

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand

MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
ARM_ACTION = 1


class PrecisionLandingNode(Node):
    def __init__(self) -> None:
        super().__init__('precision_landing_node')

        self.declare_parameter('timer_period', 0.05)
        self.declare_parameter('system_id', 1)
        self.declare_parameter('component_id', 1)
        self.declare_parameter('arm_after_setpoints', 10)
        self.declare_parameter('target_timeout_sec', 0.4)
        self.declare_parameter('target_topic', '/vision/landing_target_offset')
        self.declare_parameter('hold_x', 0.0)
        self.declare_parameter('hold_y', 0.0)
        self.declare_parameter('approach_height_m', -5.0)
        self.declare_parameter('landing_height_m', -0.6)
        self.declare_parameter('align_threshold_m', 0.15)
        self.declare_parameter('max_horizontal_step_m', 0.5)
        self.declare_parameter('descend_step_m', 0.15)
        self.declare_parameter('max_setpoint_deviation_m', 10.0)
        self.declare_parameter('target_yaw_rad', 0.0)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos
        )
        self._trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos
        )
        self._vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos
        )

        self._target_sub = self.create_subscription(
            PointStamped,
            str(self.get_parameter('target_topic').value),
            self._target_callback,
            qos,
        )

        self._target_x: Optional[float] = None
        self._target_y: Optional[float] = None
        self._target_time_s: Optional[float] = None

        self._sp_x = float(self.get_parameter('hold_x').value)
        self._sp_y = float(self.get_parameter('hold_y').value)
        self._sp_z = float(self.get_parameter('approach_height_m').value)
        self._hold_x = self._sp_x
        self._hold_y = self._sp_y
        self._max_setpoint_deviation_m = float(
            self.get_parameter('max_setpoint_deviation_m').value
        )
        self._descend_step_m = float(self.get_parameter('descend_step_m').value)
        if self._descend_step_m < 0.0:
            self.get_logger().warning('descend_step_m is negative; using 0.0.')
            self._descend_step_m = 0.0

        self._setpoint_counter = 0
        self._arm_and_mode_sent = False
        self._last_phase: Optional[str] = None

        timer_period = float(self.get_parameter('timer_period').value)
        self._timer = self.create_timer(timer_period, self._timer_callback)
        self.get_logger().info('PrecisionLandingNode started.')

    def _timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds // 1000)

    def _now_s(self) -> float:
        return float(self.get_clock().now().nanoseconds) / 1e9

    def _target_callback(self, msg: PointStamped) -> None:
        self._target_x = float(msg.point.x)
        self._target_y = float(msg.point.y)
        self._target_time_s = self._now_s()

    def _publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp = self._timestamp_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self._offboard_control_mode_pub.publish(msg)

    def _publish_trajectory_setpoint(self) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp = self._timestamp_us()
        msg.position = [self._sp_x, self._sp_y, self._sp_z]
        msg.yaw = float(self.get_parameter('target_yaw_rad').value)
        self._trajectory_setpoint_pub.publish(msg)

    def _publish_vehicle_command(
        self, command: int, param1: float = 0.0, param2: float = 0.0
    ) -> None:
        msg = VehicleCommand()
        msg.timestamp = self._timestamp_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = int(self.get_parameter('system_id').value)
        msg.target_component = int(self.get_parameter('component_id').value)
        msg.source_system = int(self.get_parameter('system_id').value)
        msg.source_component = int(self.get_parameter('component_id').value)
        msg.from_external = True
        self._vehicle_command_pub.publish(msg)

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min(value, max_value), min_value)

    @staticmethod
    def _step_towards(current: float, target: float, step: float) -> float:
        if step <= 0.0:
            return current
        delta = target - current
        if abs(delta) <= step:
            return target
        return current + step if delta > 0.0 else current - step

    def _target_is_fresh(self) -> bool:
        if self._target_time_s is None:
            return False
        target_timeout_sec = float(self.get_parameter('target_timeout_sec').value)
        return (self._now_s() - self._target_time_s) <= target_timeout_sec

    def _update_setpoint_from_target(self) -> str:
        if not self._target_is_fresh() or self._target_x is None or self._target_y is None:
            return 'HOLD'

        max_horizontal_step_m = float(self.get_parameter('max_horizontal_step_m').value)
        align_threshold_m = float(self.get_parameter('align_threshold_m').value)
        landing_height_m = float(self.get_parameter('landing_height_m').value)

        correction_x = self._clamp(
            self._target_x, -max_horizontal_step_m, max_horizontal_step_m
        )
        correction_y = self._clamp(
            self._target_y, -max_horizontal_step_m, max_horizontal_step_m
        )
        self._sp_x += correction_x
        self._sp_y += correction_y
        self._sp_x = self._clamp(
            self._sp_x,
            self._hold_x - self._max_setpoint_deviation_m,
            self._hold_x + self._max_setpoint_deviation_m,
        )
        self._sp_y = self._clamp(
            self._sp_y,
            self._hold_y - self._max_setpoint_deviation_m,
            self._hold_y + self._max_setpoint_deviation_m,
        )
        if abs(self._target_x) <= align_threshold_m and abs(self._target_y) <= align_threshold_m:
            self._sp_z = self._step_towards(
                self._sp_z, landing_height_m, self._descend_step_m
            )
            return 'DESCEND'
        return 'ALIGN'

    def _arm_and_switch_offboard_if_ready(self) -> None:
        self._setpoint_counter += 1
        arm_after_setpoints = int(self.get_parameter('arm_after_setpoints').value)
        if self._arm_and_mode_sent or self._setpoint_counter < arm_after_setpoints:
            return
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_CUSTOM_MAIN_MODE_OFFBOARD,
        )
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            ARM_ACTION,
        )
        self._arm_and_mode_sent = True
        self.get_logger().info('Sent OFFBOARD mode request and arm command.')

    def _timer_callback(self) -> None:
        phase = self._update_setpoint_from_target()
        if phase != self._last_phase:
            self.get_logger().info(
                f'phase={phase}, sp=({self._sp_x:.2f}, {self._sp_y:.2f}, {self._sp_z:.2f})'
            )
            self._last_phase = phase
        self._publish_offboard_control_mode()
        self._publish_trajectory_setpoint()
        self._arm_and_switch_offboard_if_ready()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PrecisionLandingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

from __future__ import annotations

import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus

MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
COMPONENT_ARM = 1


class OffboardControlNode(Node):
    def __init__(self) -> None:
        super().__init__('offboard_control_node')

        self.declare_parameter('timer_period', 0.1)
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', -5.0)
        self.declare_parameter('target_yaw_rad', 0.0)
        self.declare_parameter('system_id', 1)
        self.declare_parameter('component_id', 1)
        self.declare_parameter('arm_after_setpoints', 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos,
        )
        self._trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos,
        )
        self._vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos,
        )

        self._vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self._vehicle_status_callback,
            qos,
        )

        self._nav_state: Optional[int] = None
        self._setpoint_counter = 0
        self._arm_and_mode_sent = False

        timer_period = float(self.get_parameter('timer_period').value)
        self._timer = self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info('OffboardControlNode started.')

    def _timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds // 1000)

    def _vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self._nav_state = msg.nav_state

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
        msg.position = [
            float(self.get_parameter('target_x').value),
            float(self.get_parameter('target_y').value),
            float(self.get_parameter('target_z').value),
        ]
        yaw = float(self.get_parameter('target_yaw_rad').value)
        msg.yaw = yaw if math.isfinite(yaw) else 0.0
        self._trajectory_setpoint_pub.publish(msg)

    def _publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
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

    def _timer_callback(self) -> None:
        self._publish_offboard_control_mode()
        self._publish_trajectory_setpoint()

        self._setpoint_counter += 1
        arm_after_setpoints = int(self.get_parameter('arm_after_setpoints').value)

        if not self._arm_and_mode_sent and self._setpoint_counter >= arm_after_setpoints:
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                PX4_CUSTOM_MAIN_MODE_OFFBOARD,
            )
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                COMPONENT_ARM,
            )
            self._arm_and_mode_sent = True
            self.get_logger().info(
                f'Sent OFFBOARD mode request and arm command (nav_state={self._nav_state}).'
            )


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = OffboardControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

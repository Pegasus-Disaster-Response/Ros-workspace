#!/usr/bin/env python3
"""
Pegasus PX4 Offboard Interface Node
-------------------------------------
Bridges MPC trajectory setpoints to PX4 offboard control commands
over XRCE-DDS. This is the final link in the autonomy chain:

    A* → D* Lite 3D → MPC → THIS NODE → PX4 → motors

The node handles:
  1. Publishing OffboardControlMode at >2 Hz (PX4 requires this
     to stay in offboard mode — drops out after 0.5s without it)
  2. Converting MPC PoseStamped + TwistStamped setpoints to
     PX4 TrajectorySetpoint messages (ROS ENU → PX4 NED conversion)
  3. Arming and switching to offboard mode on command
  4. Safety: returns to HOLD mode if MPC stops publishing

Frame conventions:
  - ROS uses ENU (East-North-Up) with FLU body frame
  - PX4 uses NED (North-East-Down) with FRD body frame
  - Conversion: NED.x = ENU.y, NED.y = ENU.x, NED.z = -ENU.z

Publishes (to PX4 via XRCE-DDS):
    /fmu/in/trajectory_setpoint      (px4_msgs/TrajectorySetpoint)
    /fmu/in/offboard_control_mode    (px4_msgs/OffboardControlMode)
    /fmu/in/vehicle_command          (px4_msgs/VehicleCommand)

Subscribes:
    /pegasus/trajectory/setpoint           (PoseStamped — from MPC)
    /pegasus/trajectory/velocity_setpoint  (TwistStamped — from MPC)
    /pegasus/offboard/arm                  (Bool — arm command)
    /pegasus/offboard/engage               (Bool — switch to offboard)

Publishes (status):
    /pegasus/offboard/status         (String — JSON status)

Author: Team Pegasus — Cal Poly Pomona
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool, String

from px4_msgs.msg import (
    TrajectorySetpoint,
    OffboardControlMode,
    VehicleCommand,
    VehicleOdometry,
)


class PX4OffboardNode(Node):

    # PX4 VehicleCommand constants
    VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
    VEHICLE_CMD_DO_SET_MODE = 176

    def __init__(self):
        super().__init__('px4_offboard_node')

        # ── Parameters ──────────────────────────────────────
        self.declare_parameter('offboard_heartbeat_hz', 50.0)
        self.declare_parameter('setpoint_timeout_s', 0.5)
        self.declare_parameter('auto_engage', False)
        self.declare_parameter('auto_arm', False)
        self.declare_parameter('default_altitude_ned_m', -10.0)

        self.heartbeat_hz = self.get_parameter('offboard_heartbeat_hz').value
        self.setpoint_timeout = self.get_parameter('setpoint_timeout_s').value
        self.auto_engage = self.get_parameter('auto_engage').value
        self.auto_arm = self.get_parameter('auto_arm').value
        self.default_alt = self.get_parameter('default_altitude_ned_m').value

        # ── PX4 QoS — must be BEST_EFFORT for XRCE-DDS ─────
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── State ───────────────────────────────────────────
        self.latest_pose = None
        self.latest_twist = None
        self.last_setpoint_time = 0.0
        self.offboard_engaged = False
        self.armed = False
        self.setpoint_count = 0
        self.heartbeat_count = 0

        # ── Subscribers (from MPC) ──────────────────────────
        self.create_subscription(
            PoseStamped, '/pegasus/trajectory/setpoint',
            self._setpoint_cb, 10)
        self.create_subscription(
            TwistStamped, '/pegasus/trajectory/velocity_setpoint',
            self._velocity_cb, 10)
        self.create_subscription(
            Bool, '/pegasus/offboard/arm',
            self._arm_cb, 10)
        self.create_subscription(
            Bool, '/pegasus/offboard/engage',
            self._engage_cb, 10)

        # ── Subscribers (from PX4 — for status monitoring) ──
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self._vehicle_odom_cb, px4_qos)

        # ── Publishers (to PX4 via XRCE-DDS) ────────────────
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint', px4_qos)
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode', px4_qos)
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command', px4_qos)

        # ── Status publisher ────────────────────────────────
        self.status_pub = self.create_publisher(
            String, '/pegasus/offboard/status', 10)

        # ── Timers ──────────────────────────────────────────
        self.create_timer(
            1.0 / self.heartbeat_hz, self._heartbeat_tick)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'PX4 Offboard Interface: heartbeat={self.heartbeat_hz}Hz, '
            f'timeout={self.setpoint_timeout}s, '
            f'auto_engage={self.auto_engage}, auto_arm={self.auto_arm}')

    # ── Callbacks ────────────────────────────────────────────

    def _setpoint_cb(self, msg: PoseStamped):
        self.latest_pose = msg
        self.last_setpoint_time = self.get_clock().now().nanoseconds / 1e9

    def _velocity_cb(self, msg: TwistStamped):
        self.latest_twist = msg

    def _vehicle_odom_cb(self, msg: VehicleOdometry):
        # Could use this for monitoring — currently just confirms PX4 link
        pass

    def _arm_cb(self, msg: Bool):
        if msg.data:
            self.arm()
        else:
            self.disarm()

    def _engage_cb(self, msg: Bool):
        if msg.data:
            self.engage_offboard()
        else:
            self.offboard_engaged = False
            self.get_logger().info('Offboard mode disengaged')

    # ── Main heartbeat loop ──────────────────────────────────

    def _heartbeat_tick(self):
        """
        Runs at offboard_heartbeat_hz (~50 Hz).

        PX4 requires OffboardControlMode at >2 Hz to stay in offboard
        mode. We publish it every tick alongside the trajectory setpoint.
        If MPC stops publishing setpoints for longer than setpoint_timeout,
        we stop sending trajectory commands (PX4 will fall back to HOLD).
        """
        now = self.get_clock().now().nanoseconds / 1e9

        # Always publish OffboardControlMode (PX4 heartbeat)
        self._publish_offboard_control_mode()
        self.heartbeat_count += 1

        # Auto-engage sequence: arm then engage after sending enough heartbeats
        if self.auto_engage and not self.offboard_engaged:
            if self.heartbeat_count > 50:  # ~1 second of heartbeats
                if self.auto_arm and not self.armed:
                    self.arm()
                self.engage_offboard()

        # Only publish trajectory if we have a recent setpoint
        if self.latest_pose is None:
            return

        age = now - self.last_setpoint_time
        if age > self.setpoint_timeout:
            if self.setpoint_count > 0:
                self.get_logger().warn(
                    f'MPC setpoint stale ({age:.1f}s) — '
                    f'holding last command',
                    throttle_duration_sec=2.0)
            return

        # Convert ROS ENU setpoint to PX4 NED and publish
        self._publish_trajectory_setpoint()
        self.setpoint_count += 1

    # ── ENU → NED conversion + publishing ────────────────────

    def _publish_trajectory_setpoint(self):
        """
        Convert MPC PoseStamped (ENU) + TwistStamped (ENU) to
        PX4 TrajectorySetpoint (NED) and publish.

        Frame conversion:
            NED.x = ENU.y    (North = East-frame Y)
            NED.y = ENU.x    (East = East-frame X)
            NED.z = -ENU.z   (Down = -Up)
        """
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        if self.latest_pose is not None:
            p = self.latest_pose.pose.position
            # ENU → NED position
            msg.position[0] = p.y     # NED X = ENU Y (North)
            msg.position[1] = p.x     # NED Y = ENU X (East)
            msg.position[2] = -p.z    # NED Z = -ENU Z (Down)

            # Yaw: ENU yaw (from East, CCW) → NED yaw (from North, CW)
            # Extract yaw from quaternion
            q = self.latest_pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            enu_yaw = math.atan2(siny_cosp, cosy_cosp)
            # NED yaw = pi/2 - ENU yaw (convert reference from East to North)
            ned_yaw = math.pi / 2.0 - enu_yaw
            # Normalize to [-pi, pi]
            ned_yaw = (ned_yaw + math.pi) % (2 * math.pi) - math.pi
            msg.yaw = ned_yaw
        else:
            msg.position[0] = float('nan')
            msg.position[1] = float('nan')
            msg.position[2] = self.default_alt
            msg.yaw = float('nan')

        if self.latest_twist is not None:
            v = self.latest_twist.twist.linear
            # ENU → NED velocity
            msg.velocity[0] = v.y     # NED Vx = ENU Vy
            msg.velocity[1] = v.x     # NED Vy = ENU Vx
            msg.velocity[2] = -v.z    # NED Vz = -ENU Vz
        else:
            msg.velocity[0] = float('nan')
            msg.velocity[1] = float('nan')
            msg.velocity[2] = float('nan')

        # Acceleration: not commanded (let PX4 PID handle it)
        msg.acceleration[0] = float('nan')
        msg.acceleration[1] = float('nan')
        msg.acceleration[2] = float('nan')
        msg.jerk[0] = float('nan')
        msg.jerk[1] = float('nan')
        msg.jerk[2] = float('nan')
        msg.yawspeed = float('nan')

        self.traj_pub.publish(msg)

    def _publish_offboard_control_mode(self):
        """
        Tell PX4 which axes we're commanding.
        Position + velocity = PX4 uses position controller with
        velocity feedforward.
        """
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_mode_pub.publish(msg)

    # ── Arm / Disarm / Engage ────────────────────────────────

    def arm(self):
        """Send arm command to PX4."""
        self._send_vehicle_command(
            self.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0)
        self.armed = True
        self.get_logger().info('ARM command sent')

    def disarm(self):
        """Send disarm command to PX4."""
        self._send_vehicle_command(
            self.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0)
        self.armed = False
        self.get_logger().info('DISARM command sent')

    def engage_offboard(self):
        """Switch PX4 to offboard mode."""
        # PX4 custom mode for OFFBOARD: main_mode=6, sub_mode=0
        # Encoded as: base_mode=1 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
        # custom_mode = main_mode << 16 | sub_mode << 24
        # For DO_SET_MODE: param1=base_mode, param2=custom_main_mode
        self._send_vehicle_command(
            self.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,    # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            param2=6.0)    # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        self.offboard_engaged = True
        self.get_logger().info('OFFBOARD mode command sent')

    def _send_vehicle_command(self, command, param1=0.0, param2=0.0,
                               param3=0.0, param4=0.0, param5=0.0,
                               param6=0.0, param7=0.0):
        """Send a VehicleCommand to PX4."""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_cmd_pub.publish(msg)

    # ── Status ───────────────────────────────────────────────

    def _publish_status(self):
        now = self.get_clock().now().nanoseconds / 1e9
        age = now - self.last_setpoint_time if self.last_setpoint_time > 0 else -1.0

        status = {
            "armed": self.armed,
            "offboard_engaged": self.offboard_engaged,
            "has_setpoint": self.latest_pose is not None,
            "setpoint_age_s": round(age, 2),
            "setpoints_sent": self.setpoint_count,
            "heartbeats_sent": self.heartbeat_count,
            "auto_engage": self.auto_engage,
            "auto_arm": self.auto_arm,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PX4OffboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Offboard interface shutting down')
        node.disarm()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
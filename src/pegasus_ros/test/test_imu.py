#!/usr/bin/env python3
"""
IMU Sensor Unit Test — Pegasus Disaster Response UAV

Validates the PX4 IMU bridge output on /pegasus/imu/data before SLAM.

Checks:
  1. Topic is publishing within timeout
  2. frame_id is 'imu_link'
  3. Linear acceleration magnitude ≈ 9.81 m/s² (gravity vector, ±2 m/s²)
  4. Angular velocity values are finite
  5. Orientation is valid (covariance[0] != -1)
  6. Publish rate is ≥ 40 Hz (configured for 50 Hz)

Usage:
  python3 test/test_imu.py
  # or after building:
  ros2 run pegasus_ros test_imu
"""

import sys
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

TOPIC = '/pegasus/imu/data'
EXPECTED_FRAME = 'imu_link'
COLLECT_SECS = 3.0
TIMEOUT_SECS = 5.0
MIN_RATE_HZ = 40.0
GRAVITY = 9.81
GRAVITY_TOLERANCE = 2.0   # m/s²


class ImuTester(Node):
    def __init__(self):
        super().__init__('imu_tester')
        self.messages = []
        self.first_stamp = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )
        self.sub = self.create_subscription(Imu, TOPIC, self._cb, qos)

    def _cb(self, msg):
        if self.first_stamp is None:
            self.first_stamp = time.monotonic()
        self.messages.append(msg)


def _ok(text):
    return f'\033[92m  PASS\033[0m  {text}'

def _fail(text):
    return f'\033[91m  FAIL\033[0m  {text}'


def run():
    rclpy.init()
    node = ImuTester()
    print(f'\n=== IMU Test  ({TOPIC}) ===')
    print(f'Waiting up to {TIMEOUT_SECS}s for first message...')

    deadline = time.monotonic() + TIMEOUT_SECS
    while time.monotonic() < deadline and not node.messages:
        rclpy.spin_once(node, timeout_sec=0.1)

    results = []

    # ── Check 1: topic publishing ───────────────────────────────
    if not node.messages:
        results.append(_fail(f'No message received on {TOPIC} within {TIMEOUT_SECS}s'))
        _print_and_exit(results, node)

    results.append(_ok(f'Topic publishing — first message received'))

    # Collect for COLLECT_SECS to measure rate
    collect_end = time.monotonic() + COLLECT_SECS
    while time.monotonic() < collect_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    msgs = node.messages
    msg = msgs[-1]

    # ── Check 2: frame_id ───────────────────────────────────────
    fid = msg.header.frame_id
    if fid == EXPECTED_FRAME:
        results.append(_ok(f'frame_id = "{fid}"'))
    else:
        results.append(_fail(f'frame_id = "{fid}"  (expected "{EXPECTED_FRAME}")'))

    # ── Check 3: gravity magnitude ──────────────────────────────
    ax = msg.linear_acceleration.x
    ay = msg.linear_acceleration.y
    az = msg.linear_acceleration.z
    mag = math.sqrt(ax**2 + ay**2 + az**2)
    if abs(mag - GRAVITY) <= GRAVITY_TOLERANCE:
        results.append(_ok(
            f'Linear accel magnitude = {mag:.3f} m/s²  (gravity ✓)'
        ))
    else:
        results.append(_fail(
            f'Linear accel magnitude = {mag:.3f} m/s²  '
            f'(expected {GRAVITY} ± {GRAVITY_TOLERANCE} m/s²)'
        ))

    # ── Check 4: angular velocity finite ───────────────────────
    gx = msg.angular_velocity.x
    gy = msg.angular_velocity.y
    gz = msg.angular_velocity.z
    if all(math.isfinite(v) for v in (gx, gy, gz)):
        results.append(_ok(
            f'Angular velocity finite  [{gx:.4f}, {gy:.4f}, {gz:.4f}] rad/s'
        ))
    else:
        results.append(_fail(f'Angular velocity contains non-finite values'))

    # ── Check 5: valid orientation ──────────────────────────────
    if msg.orientation_covariance[0] != -1.0:
        q = msg.orientation
        results.append(_ok(
            f'Orientation valid  w={q.w:.3f} x={q.x:.3f} y={q.y:.3f} z={q.z:.3f}'
        ))
    else:
        results.append(_fail(
            'Orientation unknown (covariance[0] == -1). '
            'VehicleAttitude not received from Pixhawk.'
        ))

    # ── Check 6: publish rate ───────────────────────────────────
    elapsed = time.monotonic() - node.first_stamp
    rate = len(msgs) / elapsed if elapsed > 0 else 0.0
    if rate >= MIN_RATE_HZ:
        results.append(_ok(f'Publish rate = {rate:.1f} Hz  (≥ {MIN_RATE_HZ} Hz)'))
    else:
        results.append(_fail(
            f'Publish rate = {rate:.1f} Hz  (expected ≥ {MIN_RATE_HZ} Hz). '
            'Check XRCE-DDS agent connection.'
        ))

    _print_and_exit(results, node)


def _print_and_exit(results, node):
    for line in results:
        print(line)
    failures = sum(1 for r in results if 'FAIL' in r)
    print(f'\n{"─"*45}')
    if failures == 0:
        print(f'\033[92mAll {len(results)} IMU checks passed.\033[0m\n')
    else:
        print(f'\033[91m{failures}/{len(results)} IMU checks FAILED.\033[0m\n')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if failures == 0 else 1)


if __name__ == '__main__':
    run()

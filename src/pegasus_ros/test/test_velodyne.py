#!/usr/bin/env python3
"""
Velodyne VLP-16 Sensor Unit Test — Pegasus Disaster Response UAV

Validates the Velodyne point cloud output on /velodyne_points before SLAM.

Checks:
  1. Topic is publishing within timeout
  2. frame_id is 'velodyne'
  3. Point cloud is non-empty
  4. Point count is in a realistic range for VLP-16 (≥ 10 000 pts/scan)
  5. Publish rate is ≥ 8 Hz (configured for 10 Hz at 600 RPM)
  6. Point fields include x, y, z, intensity, ring

Usage:
  python3 test/test_velodyne.py
  # or after building:
  ros2 run pegasus_ros test_velodyne
"""

import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

TOPIC = '/velodyne_points'
EXPECTED_FRAME = 'velodyne'
COLLECT_SECS = 3.0
TIMEOUT_SECS = 5.0
MIN_RATE_HZ = 8.0
MIN_POINTS = 10_000    # VLP-16: 16 rings × ~1800 pts/ring per scan at 600 RPM
EXPECTED_FIELDS = {'x', 'y', 'z', 'intensity', 'ring'}


class VelodyneTester(Node):
    def __init__(self):
        super().__init__('velodyne_tester')
        self.messages = []
        self.first_stamp = None
        self.sub = self.create_subscription(
            PointCloud2, TOPIC, self._cb, 10
        )

    def _cb(self, msg):
        if self.first_stamp is None:
            self.first_stamp = time.monotonic()
        self.messages.append(msg)


def _ok(text):
    return f'\033[92m  PASS\033[0m  {text}'

def _fail(text):
    return f'\033[91m  FAIL\033[0m  {text}'


def _point_count(msg: PointCloud2) -> int:
    return msg.width * msg.height


def run():
    rclpy.init()
    node = VelodyneTester()
    print(f'\n=== Velodyne VLP-16 Test  ({TOPIC}) ===')
    print(f'Waiting up to {TIMEOUT_SECS}s for first message...')

    deadline = time.monotonic() + TIMEOUT_SECS
    while time.monotonic() < deadline and not node.messages:
        rclpy.spin_once(node, timeout_sec=0.1)

    results = []

    # ── Check 1: topic publishing ───────────────────────────────
    if not node.messages:
        results.append(_fail(
            f'No message received on {TOPIC} within {TIMEOUT_SECS}s. '
            'Check velodyne_driver and velodyne_convert are running, '
            'and VLP-16 IP is reachable at 192.168.1.201.'
        ))
        _print_and_exit(results, node)

    results.append(_ok('Topic publishing — first message received'))

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
        results.append(_fail(
            f'frame_id = "{fid}"  (expected "{EXPECTED_FRAME}"). '
            'Check velodyne_driver frame_id parameter.'
        ))

    # ── Check 3 & 4: point count ────────────────────────────────
    n_pts = _point_count(msg)
    if n_pts == 0:
        results.append(_fail('Point cloud is empty (width × height = 0)'))
    elif n_pts < MIN_POINTS:
        results.append(_fail(
            f'Point count = {n_pts}  (expected ≥ {MIN_POINTS}). '
            'Possible partial scan or driver issue.'
        ))
    else:
        results.append(_ok(f'Point count = {n_pts} pts'))

    # ── Check 5: publish rate ───────────────────────────────────
    elapsed = time.monotonic() - node.first_stamp
    rate = len(msgs) / elapsed if elapsed > 0 else 0.0
    if rate >= MIN_RATE_HZ:
        results.append(_ok(f'Publish rate = {rate:.1f} Hz  (≥ {MIN_RATE_HZ} Hz)'))
    else:
        results.append(_fail(
            f'Publish rate = {rate:.1f} Hz  (expected ≥ {MIN_RATE_HZ} Hz). '
            'Check VLP-16 RPM setting (should be 600 RPM = 10 Hz).'
        ))

    # ── Check 6: expected point fields ──────────────────────────
    field_names = {f.name for f in msg.fields}
    missing = EXPECTED_FIELDS - field_names
    if not missing:
        results.append(_ok(f'Point fields present: {sorted(field_names)}'))
    else:
        results.append(_fail(f'Missing point fields: {sorted(missing)}'))

    _print_and_exit(results, node)


def _print_and_exit(results, node):
    for line in results:
        print(line)
    failures = sum(1 for r in results if 'FAIL' in r)
    print(f'\n{"─"*45}')
    if failures == 0:
        print(f'\033[92mAll {len(results)} Velodyne checks passed.\033[0m\n')
    else:
        print(f'\033[91m{failures}/{len(results)} Velodyne checks FAILED.\033[0m\n')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if failures == 0 else 1)


if __name__ == '__main__':
    run()

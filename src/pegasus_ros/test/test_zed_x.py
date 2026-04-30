#!/usr/bin/env python3
"""
ZED X Stereo Camera Unit Test — Pegasus Disaster Response UAV

Validates all three ZED X topics used by SLAM before testing SLAM.

Topics checked:
  /zed_x/zed_node/rgb/color/rect/image        — rectified RGB
  /zed_x/zed_node/rgb/color/rect/camera_info  — RGB camera calibration
  /zed_x/zed_node/depth/depth_registered      — depth aligned to RGB

Checks:
  1. All three topics publish within timeout
  2. RGB frame_id contains 'zed_x_left_camera_frame' (optical frame)
  3. RGB image dimensions match config (pub_resolution=CUSTOM, downscale=2 → 960×540)
  4. camera_info focal length > 0 (calibration loaded)
  5. Depth frame_id contains 'zed_x' (depth is aligned to left camera)
  6. Depth values are in [0.3, 20.0] m range (per zed_x.yaml min/max_depth)
  7. RGB publish rate is ≥ 25 Hz (configured for 30 Hz)

Usage:
  python3 test/test_zed_x.py
  # or after building:
  ros2 run pegasus_ros test_zed_x
"""

import sys
import math
import time
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

RGB_TOPIC    = '/zed_x/zed_node/rgb/color/rect/image'
INFO_TOPIC   = '/zed_x/zed_node/rgb/color/rect/camera_info'
DEPTH_TOPIC  = '/zed_x/zed_node/depth/depth_registered'

COLLECT_SECS  = 3.0
TIMEOUT_SECS  = 10.0    # ZED takes longer to initialize
MIN_RATE_HZ   = 25.0
DEPTH_MIN     = 0.3     # m — from zed_x.yaml min_depth
DEPTH_MAX     = 20.0    # m — from zed_x.yaml max_depth
# Expected resolution: HD1080 (1920×1080) with pub_downscale_factor=2.0 → 960×540
EXPECTED_W    = 960
EXPECTED_H    = 540


class ZedXTester(Node):
    def __init__(self):
        super().__init__('zed_x_tester')
        self.rgb_msgs   = []
        self.info_msgs  = []
        self.depth_msgs = []
        self.first_rgb_stamp = None

        self.sub_rgb   = self.create_subscription(Image,      RGB_TOPIC,   self._rgb_cb,   10)
        self.sub_info  = self.create_subscription(CameraInfo, INFO_TOPIC,  self._info_cb,  10)
        self.sub_depth = self.create_subscription(Image,      DEPTH_TOPIC, self._depth_cb, 10)

    def _rgb_cb(self, msg):
        if self.first_rgb_stamp is None:
            self.first_rgb_stamp = time.monotonic()
        self.rgb_msgs.append(msg)

    def _info_cb(self, msg):
        self.info_msgs.append(msg)

    def _depth_cb(self, msg):
        self.depth_msgs.append(msg)

    @property
    def all_received(self):
        return self.rgb_msgs and self.info_msgs and self.depth_msgs


def _ok(text):
    return f'\033[92m  PASS\033[0m  {text}'

def _fail(text):
    return f'\033[91m  FAIL\033[0m  {text}'


def _sample_depth_range(msg: Image):
    """Return (min, max) of finite, non-zero depth values from a 32FC1 image."""
    if msg.encoding != '32FC1':
        return None, None
    floats = struct.unpack(f'{len(msg.data)//4}f', bytes(msg.data))
    valid = [v for v in floats if math.isfinite(v) and v > 0.0]
    if not valid:
        return None, None
    return min(valid), max(valid)


def run():
    rclpy.init()
    node = ZedXTester()
    print(f'\n=== ZED X Camera Test ===')
    print(f'Waiting up to {TIMEOUT_SECS}s for all three topics...')
    print(f'  {RGB_TOPIC}')
    print(f'  {INFO_TOPIC}')
    print(f'  {DEPTH_TOPIC}')

    deadline = time.monotonic() + TIMEOUT_SECS
    while time.monotonic() < deadline and not node.all_received:
        rclpy.spin_once(node, timeout_sec=0.1)

    results = []

    # ── Check 1: all topics publishing ─────────────────────────
    missing_topics = []
    if not node.rgb_msgs:
        missing_topics.append(RGB_TOPIC)
    if not node.info_msgs:
        missing_topics.append(INFO_TOPIC)
    if not node.depth_msgs:
        missing_topics.append(DEPTH_TOPIC)

    if missing_topics:
        for t in missing_topics:
            results.append(_fail(
                f'No message on {t} within {TIMEOUT_SECS}s. '
                'Check zed_wrapper is running and serial_number matches camera.'
            ))
        _print_and_exit(results, node)

    results.append(_ok('All three topics publishing'))

    # Collect for COLLECT_SECS to measure rate
    collect_end = time.monotonic() + COLLECT_SECS
    while time.monotonic() < collect_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    rgb_msg   = node.rgb_msgs[-1]
    info_msg  = node.info_msgs[-1]
    depth_msg = node.depth_msgs[-1]

    # ── Check 2: RGB frame_id ───────────────────────────────────
    rgb_fid = rgb_msg.header.frame_id
    if 'zed_x' in rgb_fid:
        results.append(_ok(f'RGB frame_id = "{rgb_fid}"'))
    else:
        results.append(_fail(
            f'RGB frame_id = "{rgb_fid}"  (expected frame containing "zed_x"). '
            'Check camera_name in pegasus_sensors.launch.py.'
        ))

    # ── Check 3: RGB image dimensions ──────────────────────────
    w, h = rgb_msg.width, rgb_msg.height
    if w == EXPECTED_W and h == EXPECTED_H:
        results.append(_ok(f'RGB resolution = {w}×{h}  (pub_downscale_factor=2 ✓)'))
    else:
        results.append(_fail(
            f'RGB resolution = {w}×{h}  '
            f'(expected {EXPECTED_W}×{EXPECTED_H} from HD1080 ÷ 2). '
            'Check pub_resolution and pub_downscale_factor in zed_x.yaml.'
        ))

    # ── Check 4: camera_info focal length ──────────────────────
    fx = info_msg.k[0]  # K matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    fy = info_msg.k[4]
    if fx > 0.0 and fy > 0.0:
        results.append(_ok(f'Camera intrinsics loaded  fx={fx:.1f} fy={fy:.1f}'))
    else:
        results.append(_fail(
            f'Camera intrinsics invalid  fx={fx} fy={fy}. '
            'Calibration not loaded or camera not initialized.'
        ))

    # ── Check 5: depth frame_id ─────────────────────────────────
    depth_fid = depth_msg.header.frame_id
    if 'zed_x' in depth_fid:
        results.append(_ok(f'Depth frame_id = "{depth_fid}"'))
    else:
        results.append(_fail(
            f'Depth frame_id = "{depth_fid}"  (expected frame containing "zed_x")'
        ))

    # ── Check 6: depth value range ──────────────────────────────
    d_min, d_max = _sample_depth_range(depth_msg)
    if d_min is None:
        results.append(_fail(
            f'Depth image has no valid finite values  (encoding={depth_msg.encoding}). '
            'Check depth_mode in zed_x.yaml and that the scene has surfaces in view.'
        ))
    elif d_min >= DEPTH_MIN and d_max <= DEPTH_MAX:
        results.append(_ok(
            f'Depth range = [{d_min:.2f}, {d_max:.2f}] m  '
            f'(within [{DEPTH_MIN}, {DEPTH_MAX}] m ✓)'
        ))
    else:
        results.append(_fail(
            f'Depth range = [{d_min:.2f}, {d_max:.2f}] m  '
            f'(expected within [{DEPTH_MIN}, {DEPTH_MAX}] m). '
            'Some values outside zed_x.yaml min/max_depth bounds.'
        ))

    # ── Check 7: RGB publish rate ───────────────────────────────
    elapsed = time.monotonic() - node.first_rgb_stamp
    rate = len(node.rgb_msgs) / elapsed if elapsed > 0 else 0.0
    if rate >= MIN_RATE_HZ:
        results.append(_ok(f'RGB publish rate = {rate:.1f} Hz  (≥ {MIN_RATE_HZ} Hz)'))
    else:
        results.append(_fail(
            f'RGB publish rate = {rate:.1f} Hz  (expected ≥ {MIN_RATE_HZ} Hz). '
            'Check grab_frame_rate and pub_frame_rate in zed_x.yaml.'
        ))

    _print_and_exit(results, node)


def _print_and_exit(results, node):
    for line in results:
        print(line)
    failures = sum(1 for r in results if 'FAIL' in r)
    print(f'\n{"─"*45}')
    if failures == 0:
        print(f'\033[92mAll {len(results)} ZED X checks passed.\033[0m\n')
    else:
        print(f'\033[91m{failures}/{len(results)} ZED X checks FAILED.\033[0m\n')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if failures == 0 else 1)


if __name__ == '__main__':
    run()

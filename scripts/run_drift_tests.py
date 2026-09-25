#!/usr/bin/env python3
"""
Drift-free tests for SO-101 IK (rviz2-docker).

Measures joint drift after closed-loop Cartesian trajectories.
Per `IK_LITERATURE.md` §3.2.C (Hassan, El-Habrouk, Deghedie 2020, Robotica
38(8):1495–1512): without drift-free criterion in the cost function
(`min ‖q̇ + λ·(q − q_start)‖²`), closed-loop Cartesian paths accumulate
joint drift. With the criterion active, drift ≈ 0.

This script quantifies that drift. It does NOT modify `so101-ik-node.cpp` —
it only measures. To make the test pass without the criterion, run with
`--baseline` (verdict inverts).

Usage (inside container, after `ros2 launch so101_ik so101_ik.launch.py`):
    python3 scripts/run_drift_tests.py [options]

See also:
    scripts/run_all_tests.sh  — general smoke test (27 cases, A/B/C/D)
    explore-ik/IK_LITERATURE.md §3 — theoretical background
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


# Number of arm joints to capture (excluding gripper, matching rviz2-docker
# convention in `cartesian_control.cpp` which keeps the gripper fixed).
ARM_JOINTS = 5

# Default loop parameters (rviz2-docker reachable workspace, around 0.25 m
# from base, well inside the 0.48 m r_max reported in README).
DEFAULT_CENTER = (0.25, 0.0, 0.15)
CIRCLE_RADIUS = 0.05      # m
SQUARE_HALF = 0.04        # m


def circle_xz_loop(center, radius, n_points):
    """Closed loop on a horizontal circle in XZ plane.

    Starts at (cx + r, cy, cz), walks counterclockwise, returns to start.
    n_points is the number of distinct waypoints; the closing point is
    appended automatically so the loop is truly closed.
    """
    cx, cy, cz = center
    pts = [
        (cx + radius * math.cos(2 * math.pi * i / n_points),
         cy,
         cz + radius * math.sin(2 * math.pi * i / n_points))
        for i in range(n_points)
    ]
    pts.append(pts[0])
    return pts


def square_xy_loop(center, half_size, n_per_side):
    """Closed loop on a square in XY plane, walked clockwise.

    Starts at top-right corner (cx + h, cy + h), continues through 4 sides.
    Last point equals first.
    """
    cx, cy, cz = center
    h = half_size
    corners = [
        (cx + h, cy + h),   # top-right (start)
        (cx - h, cy + h),   # top-left
        (cx - h, cy - h),   # bottom-left
        (cx + h, cy - h),   # bottom-right
    ]
    points = []
    denom = max(n_per_side - 1, 1)
    for i in range(4):
        a = corners[i]
        b = corners[(i + 1) % 4]
        for j in range(n_per_side):
            t = j / denom
            points.append((
                a[0] + (b[0] - a[0]) * t,
                a[1] + (b[1] - a[1]) * t,
                cz,
            ))
    return points


class DriftTester(Node):
    def __init__(self):
        super().__init__('drift_tester')
        self.sub = self.create_subscription(
            JointState, '/joint_states', self._cb, 10)
        self.pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.latest_q = None
        self.joint_names = None

    def _cb(self, msg):
        if not msg.position:
            return
        self.latest_q = list(msg.position[:ARM_JOINTS])
        if self.joint_names is None:
            self.joint_names = list(msg.name[:ARM_JOINTS])

    def spin_for(self, timeout_sec):
        """Pump callbacks for the given duration."""
        end = time.time() + timeout_sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for_first_state(self, timeout_sec=5.0):
        """Block until /joint_states fires at least once."""
        end = time.time() + timeout_sec
        while self.latest_q is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.latest_q is None:
            raise RuntimeError(
                f"No /joint_states received within {timeout_sec:.1f}s — "
                "is the IK node running? (ros2 launch so101_ik ...)"
            )

    def publish_point(self, x, y, z):
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)
        self.spin_for(0.02)

    def publish_loop(self, points, dt=2.5):
        for p in points:
            self.publish_point(*p)
            time.sleep(dt)

    def run_shape(self, name, points, threshold, baseline, publish_dt=2.5):
        print(f"=== drift-free test: shape={name}, "
              f"n={len(points)}, threshold={threshold:.3f} rad ===")

        # 1. Wait for any first joint state to arrive.
        self.wait_for_first_state()

        # 2. Settle at the loop start for 1.5 s.
        first = points[0]
        self.publish_point(*first)
        self.spin_for(1.5)

        # 3. Capture start_q.
        start = list(self.latest_q)

        # 4. Walk the loop.
        self.publish_loop(points, dt=publish_dt)

        # 5. Re-publish start point to make sure EE returned there, settle.
        self.publish_point(*first)
        self.spin_for(2.0)

        # 6. Capture end_q.
        end = list(self.latest_q)

        # 7. Compute drift per joint.
        drift = [abs(e - s) for s, e in zip(start, end)]
        max_drift = max(drift)

        # 8. Print.
        names = self.joint_names or [f'j{i}' for i in range(ARM_JOINTS)]
        label_width = max(len(n) for n in names)
        print('start:')
        for n, q in zip(names, start):
            print(f'  {n:<{label_width}}  {q:+.4f}')
        print('end:')
        for n, q in zip(names, end):
            print(f'  {n:<{label_width}}  {q:+.4f}')
        print('drift per joint (rad):')
        for n, d in zip(names, drift):
            print(f'  {n:<{label_width}}  {d:.4f}')
        print(f'max_drift: {max_drift:.4f} rad  (threshold {threshold:.3f})')

        # 9. Verdict.
        if baseline:
            ok = max_drift > threshold
            verdict = 'BASELINE_OK' if ok else 'BASELINE_FAIL'
            note = ('criterion NOT active — raw drift recorded'
                    if ok else
                    'drift already small — criterion might already be active?')
        else:
            ok = max_drift < threshold
            verdict = 'PASS' if ok else 'FAIL'
            note = ('drift_free criterion is active (or path is short enough)'
                    if ok else
                    'drift_free criterion NOT active or insufficient')
        print(f'verdict: {verdict}  ({note})')
        return ok


def main():
    parser = argparse.ArgumentParser(
        description='Drift-free tests for SO-101 IK (rviz2-docker).')
    parser.add_argument('--shape', default='all',
                        choices=['circle-xz', 'square-xy', 'all'])
    parser.add_argument('--n-points', type=int, default=16,
                        help='Number of points for circle-xz (default 16)')
    parser.add_argument('--n-per-side', type=int, default=5,
                        help='Number of points per side for square-xy (default 5)')
    parser.add_argument('--threshold', type=float, default=0.05,
                        help='Drift threshold in radians (default 0.05 ≈ 3°)')
    parser.add_argument('--baseline', action='store_true',
                        help='Baseline mode: invert verdict — PASS if drift '
                             'EXCEEDS threshold (criterion not yet active)')
    parser.add_argument('--center', nargs=3, type=float,
                        default=list(DEFAULT_CENTER), metavar=('X', 'Y', 'Z'),
                        help=f'Loop center in base_link frame '
                             f'(default {" ".join(str(c) for c in DEFAULT_CENTER)})')
    parser.add_argument('--settle-sec', type=float, default=1.5,
                        help='Seconds to settle at start pose (default 1.5)')
    parser.add_argument('--publish-dt', type=float, default=2.5,
                        help='Seconds between target publishes during loop walk '
                             '(default 2.5 — must exceed robot interp_steps/publish_rate)')
    args = parser.parse_args()

    rclpy.init()
    tester = DriftTester()
    all_ok = True
    try:
        center = tuple(args.center)
        shapes = (['circle-xz', 'square-xy']
                  if args.shape == 'all' else [args.shape])
        for shape in shapes:
            if shape == 'circle-xz':
                points = circle_xz_loop(center, CIRCLE_RADIUS, args.n_points)
            else:
                points = square_xy_loop(center, SQUARE_HALF, args.n_per_side)
            ok = tester.run_shape(shape, points, args.threshold, args.baseline,
                                 args.publish_dt)
            all_ok = all_ok and ok
            print()
    except RuntimeError as e:
        print(f'ERROR: {e}', file=sys.stderr)
        sys.exit(2)
    finally:
        tester.destroy_node()
        rclpy.shutdown()

    print('=== summary ===')
    print(f'all_ok={all_ok}  mode={"baseline" if args.baseline else "verify"}')
    sys.exit(0 if all_ok else 1)


if __name__ == '__main__':
    main()

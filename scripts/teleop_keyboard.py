#!/usr/bin/env python3
"""Standalone keyboard teleop node for SO-101 (rviz2-docker).

Publishes 6D PoseStamped targets to /target_pose. Reads keyboard in a
separate thread with non-blocking select() so ROS 2 spin is never blocked.

Keys:
  Translation (step via +/-):
    T/G : +/- X      A/D : +/- Y      W/S : +/- Z
  Rotation (step via [/]):
    J/L : yaw   +/-    I/K : pitch +/-    U/O : roll  +/-
  Step:
    + / - : translation step (1mm <-> 10mm)
    [ / ] : rotation step (1° <-> 10°)
  Other:
    R : reset to home pose (identity quat)
    H : help (reprint controls)
    Q : quit

Usage (inside container, after ros2 launch):
    python3 scripts/teleop_keyboard.py
"""
import math
import rclpy
import select
import sys
import termios
import threading
import tty
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


HELP_TEXT = """
SO-101 Teleop (6D PoseStamped):
  Translation (step via +/-):
    T/G : +/- X      A/D : +/- Y      W/S : +/- Z
  Rotation (step via [/]):
    J/L : yaw   +/-    I/K : pitch +/-    U/O : roll  +/-
  Step:
    +/- : translation step (1mm <-> 10mm)
    [/] : rotation step (1° <-> 10°)
  Other:
    R : reset to home pose (identity quaternion)
    H : help (reprint this)
    Q : quit
"""

HOME_POS = (0.20, 0.0, 0.15)
HOME_QUAT = (0.0, 0.0, 0.0, 1.0)  # identity (x, y, z, w)


class TeleopNode(Node):
    def __init__(self):
        super().__init__('so101_teleop_keyboard')
        self.pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.target_pos = list(HOME_POS)
        self.target_quat = list(HOME_QUAT)
        self.trans_step = 0.01      # 1 cm
        self.rot_step = math.radians(1.0)  # 1 degree
        self.running = True
        print(HELP_TEXT)
        self.kb_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.kb_thread.start()

    def publish(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = self.target_pos[0]
        msg.pose.position.y = self.target_pos[1]
        msg.pose.position.z = self.target_pos[2]
        msg.pose.orientation.x = self.target_quat[0]
        msg.pose.orientation.y = self.target_quat[1]
        msg.pose.orientation.z = self.target_quat[2]
        msg.pose.orientation.w = self.target_quat[3]
        self.pub.publish(msg)
        print(
            f"target: pos=({self.target_pos[0]:+.3f}, {self.target_pos[1]:+.3f}, "
            f"{self.target_pos[2]:+.3f}) quat=({self.target_quat[0]:+.3f}, "
            f"{self.target_quat[1]:+.3f}, {self.target_quat[2]:+.3f}, "
            f"{self.target_quat[3]:+.3f})"
        )

    def keyboard_loop(self):
        while self.running and rclpy.ok():
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if rlist:
                try:
                    ch = sys.stdin.read(1)
                except Exception:
                    continue
                self.handle_key(ch)

    def apply_rotation(self, axis, angle):
        """Multiply current quat by small rotation around world axis."""
        half = angle / 2.0
        s = math.sin(half)
        dx, dy, dz, dw = 0.0, 0.0, 0.0, math.cos(half)
        if axis == 'x':
            dx = s
        elif axis == 'y':
            dy = s
        elif axis == 'z':
            dz = s
        qx, qy, qz, qw = self.target_quat
        # q * R = (qw*d.xyz + dw*q.xyz + q.xyz × d.xyz, qw*dw - q.xyz · d.xyz)
        cx = qy * dz - qz * dy
        cy = qz * dx - qx * dz
        cz = qx * dy - qy * dx
        nx = qw * dx + dw * qx + cx
        ny = qw * dy + dw * qy + cy
        nz = qw * dz + dw * qz + cz
        nw = qw * dw - (qx * dx + qy * dy + qz * dz)
        norm = math.sqrt(nx * nx + ny * ny + nz * nz + nw * nw)
        self.target_quat = [nx / norm, ny / norm, nz / norm, nw / norm]

    def handle_key(self, ch):
        if ch in ('q', 'Q'):
            self.running = False
            return
        if ch in ('h', 'H'):
            print(HELP_TEXT)
            return
        if ch in ('r', 'R'):
            self.target_pos = list(HOME_POS)
            self.target_quat = list(HOME_QUAT)
        elif ch in ('t', 'T'):
            self.target_pos[0] += self.trans_step
        elif ch in ('g', 'G'):
            self.target_pos[0] -= self.trans_step
        elif ch in ('a', 'A'):
            self.target_pos[1] += self.trans_step
        elif ch in ('d', 'D'):
            self.target_pos[1] -= self.trans_step
        elif ch in ('w', 'W'):
            self.target_pos[2] += self.trans_step
        elif ch in ('s', 'S'):
            self.target_pos[2] -= self.trans_step
        elif ch in ('+', '='):
            self.trans_step = 0.10  # 10 cm
            print(f"trans step = {self.trans_step * 1000:.0f} mm")
            return
        elif ch in ('-', '_'):
            self.trans_step = 0.01  # 1 cm
            print(f"trans step = {self.trans_step * 1000:.0f} mm")
            return
        elif ch in ('j', 'J'):
            self.apply_rotation('z', self.rot_step)
        elif ch in ('l', 'L'):
            self.apply_rotation('z', -self.rot_step)
        elif ch in ('i', 'I'):
            self.apply_rotation('y', self.rot_step)
        elif ch in ('k', 'K'):
            self.apply_rotation('y', -self.rot_step)
        elif ch in ('u', 'U'):
            self.apply_rotation('x', self.rot_step)
        elif ch in ('o', 'O'):
            self.apply_rotation('x', -self.rot_step)
        elif ch in ('[', '{'):
            self.rot_step = math.radians(10.0)
            print(f"rot step = {math.degrees(self.rot_step):.0f}°")
            return
        elif ch in (']', '}'):
            self.rot_step = math.radians(1.0)
            print(f"rot step = {math.degrees(self.rot_step):.0f}°")
            return
        else:
            return  # unknown key — don't publish
        self.publish()


def main():
    rclpy.init()
    node = TeleopNode()
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

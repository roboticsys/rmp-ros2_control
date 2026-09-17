#!/usr/bin/env python3
# Copyright 2026 Robotic Systems Integration, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Phase-2 phantom verification: jog joint1 via the online topic, stop, jog again.

A continuous producer publishes short JointTrajectory windows (positions+velocities,
quadratic-friendly) at 50 Hz along its own commanded timeline. Asserts, from
/joint_states: (1) the joint moves and reaches the commanded speed, (2) it comes to
rest PROMPTLY after the producer goes silent (bounded lookahead: committed depth <=
committed_horizon + stop-tail, so the settle must be fast), (3) a second jog reopens.
Prints PASS/FAIL lines; exit 0 only if all pass.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = [f"elfin_joint{i}" for i in range(1, 7)]
TOPIC = "/rapidcode_passthrough_trajectory_controller/joint_trajectory"
JOG_VEL = 0.2          # rad/s on joint1
PUBLISH_PERIOD = 0.02  # 50 Hz producer
WINDOW_POINTS = 6      # points per window
POINT_DT = 0.02        # window spans 120 ms
SETTLE_LIMIT = 0.60    # s from last publish to |vel| < eps (prompt-stop bound)
VEL_EPS = 1e-3


class JogDemo(Node):
    def __init__(self):
        super().__init__("jog_gate_demo")
        self.pub = self.create_publisher(JointTrajectory, TOPIC, 10)
        self.sub = self.create_subscription(JointState, "/joint_states", self.on_state, 50)
        self.state = None          # latest (t, pos[], vel[]) keyed by JOINTS order
        self.samples = []          # (t, pos_j1, vel_j1) history

    def on_state(self, msg):
        idx = {name: k for k, name in enumerate(msg.name)}
        try:
            pos = [msg.position[idx[j]] for j in JOINTS]
            vel = [msg.velocity[idx[j]] for j in JOINTS]
        except (KeyError, IndexError):
            return
        now = time.monotonic()
        self.state = (now, pos, vel)
        self.samples.append((now, pos[0], vel[0]))

    def wait_state(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while self.state is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.state is not None

    def jog(self, duration, vel):
        """Publish windows for `duration` s along a self-tracked commanded timeline."""
        base = [p for p in self.state[1]]
        start = time.monotonic()
        last_pub = start
        while time.monotonic() - start < duration:
            rclpy.spin_once(self, timeout_sec=0.0)
            now = time.monotonic()
            if now - last_pub < PUBLISH_PERIOD:
                time.sleep(0.001)
                continue
            last_pub = now
            elapsed = now - start
            msg = JointTrajectory()
            msg.joint_names = JOINTS
            for k in range(WINDOW_POINTS):
                t = k * POINT_DT
                pt = JointTrajectoryPoint()
                pt.positions = [base[0] + vel * (elapsed + t)] + base[1:]
                pt.velocities = [vel] + [0.0] * (len(JOINTS) - 1)
                pt.time_from_start.sec = int(t)
                pt.time_from_start.nanosec = int((t % 1.0) * 1e9)
                msg.points.append(pt)
            self.pub.publish(msg)
        return time.monotonic()  # time of last publish (approx)

    def watch(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)


def analyze(samples, t_last_pub, label):
    moved = max(p for _, p, _ in samples) - min(p for _, p, _ in samples)
    peak_vel = max(abs(v) for _, v, _ in [(t, v, 0) for t, _, v in samples])
    settle_t = None
    for t, _, v in samples:
        if t <= t_last_pub:
            continue
        if abs(v) < VEL_EPS:
            settle_t = t - t_last_pub
            break
    ok = True
    if moved < 0.05:
        print(f"FAIL [{label}] joint1 barely moved ({moved:.4f} rad)")
        ok = False
    else:
        print(f"PASS [{label}] joint1 moved {moved:.4f} rad")
    if abs(peak_vel - JOG_VEL) > 0.08:
        print(f"FAIL [{label}] peak |vel| {peak_vel:.3f} not near commanded {JOG_VEL}")
        ok = False
    else:
        print(f"PASS [{label}] peak |vel| {peak_vel:.3f} ~ commanded {JOG_VEL}")
    if settle_t is None:
        print(f"FAIL [{label}] never settled to rest after producer silence")
        ok = False
    elif settle_t > SETTLE_LIMIT:
        print(f"FAIL [{label}] settle took {settle_t:.3f}s > {SETTLE_LIMIT}s (depth unbounded?)")
        ok = False
    else:
        print(f"PASS [{label}] settled {settle_t*1000:.0f}ms after last publish (prompt stop)")
    return ok


def main():
    rclpy.init()
    node = JogDemo()
    if not node.wait_state():
        print("FAIL no /joint_states")
        return 1
    # Wait for the controller's subscription to match, else the first windows publish
    # into the void and, once matched, the producer timeline is already ahead of the
    # robot -> every window fails the first-point continuity check and is dropped.
    deadline = time.monotonic() + 10.0
    while node.pub.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.pub.get_subscription_count() == 0:
        print("FAIL jog topic never matched a subscriber")
        return 1
    time.sleep(0.3)  # small grace so the match is bidirectional
    all_ok = True

    node.samples.clear()
    t_last = node.jog(2.0, JOG_VEL)
    node.watch(1.5)
    all_ok &= analyze(node.samples, t_last, "jog1")

    # Second jog: the stop-tail closed the move; a fresh window must reopen cleanly.
    node.samples.clear()
    t_last = node.jog(1.5, -JOG_VEL)
    node.watch(1.5)
    all_ok &= analyze(node.samples, t_last, "jog2-reopen")

    node.destroy_node()
    rclpy.shutdown()
    print("RESULT", "PASS" if all_ok else "FAIL")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())

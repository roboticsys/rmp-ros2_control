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

"""Phase-3 verification: MoveIt Servo jog through the passthrough controller.

Pre-phase-3 symptom: joint-jog mostly HELD with sporadic twitches and ~zero net motion
(windows dropped by the continuity gate / kinked by the crude replace). This script
drives Servo in JOINT_JOG then TWIST mode and scores smoothness from /joint_states:
  - net displacement (was ~0 before)
  - stall fraction while commanding (was ~1 before: the "hold" symptom)
  - wrong-direction fraction (the "twitch backward" symptom)
  - settle time after command silence (stop-tail promptness)
Exit 0 only if all checks pass.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = [f"elfin_joint{i}" for i in range(1, 7)]
RATE = 0.02          # 50 Hz command stream
# Servo owns most of the stop: 0.1s stale timeout + its Ruckig decel (~1.5s from 0.48
# rad/s at the ~0.32 rad/s^2 filter limit), then our stop-tail closes on silence.
SETTLE_LIMIT = 2.5
VEL_EPS = 1e-3


class ServoSmoke(Node):
    def __init__(self):
        super().__init__("servo_smoke_phase3")
        self.jog_pub = self.create_publisher(JointJog, "/servo_node/delta_joint_cmds", 10)
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.raw_pub = self.create_publisher(JointTrajectory,
            "/rapidcode_passthrough_trajectory_controller/joint_trajectory", 10)
        self.sub = self.create_subscription(JointState, "/joint_states", self.on_state, 50)
        self.mode_cli = self.create_client(ServoCommandType, "/servo_node/switch_command_type")
        self.state = None
        self.samples = []  # (t, {joint: (pos, vel)})

    def on_state(self, msg):
        idx = {name: k for k, name in enumerate(msg.name)}
        try:
            snap = {j: (msg.position[idx[j]], msg.velocity[idx[j]]) for j in JOINTS}
        except (KeyError, IndexError):
            return
        self.state = snap
        self.samples.append((time.monotonic(), snap))

    def wait_ready(self):
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.state is not None and self.mode_cli.service_is_ready():
                return True
        return False

    def switch_mode(self, mode):
        req = ServoCommandType.Request()
        req.command_type = mode
        future = self.mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.done() and future.result() is not None and future.result().success

    def stream(self, duration, make_msg, publisher):
        start = time.monotonic()
        last_pub = 0.0
        while time.monotonic() - start < duration:
            rclpy.spin_once(self, timeout_sec=0.0)
            now = time.monotonic()
            if now - last_pub >= RATE:
                last_pub = now
                publisher.publish(make_msg())
            time.sleep(0.001)
        return time.monotonic()

    def watch(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def joint_jog_msg(self, joint_velocity):
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(joint_velocity.keys())
        msg.velocities = list(joint_velocity.values())
        return msg

    def twist_msg(self, linear_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "elfin_base_link"
        msg.twist.linear.z = linear_z
        return msg


def joint_series(samples, joint, t_from, t_to):
    return [(t, s[joint][0], s[joint][1]) for t, s in samples if t_from <= t <= t_to]


def settle_time(samples, joint_list, t_after):
    for t, snap in samples:
        if t <= t_after:
            continue
        if all(abs(snap[j][1]) < VEL_EPS for j in joint_list):
            return t - t_after
    return None


def check(condition, label, detail):
    print(("PASS" if condition else "FAIL"), label, detail)
    return condition


def analyze_jog(samples, joint, t_start, t_last_pub):
    # Steady window: skip the first 0.5s (Servo ramp) and the trailing settle.
    series = joint_series(samples, joint, t_start + 0.5, t_last_pub)
    if len(series) < 20:
        print("FAIL jog: too few samples")
        return False
    moved = series[-1][1] - series[0][1]
    stall = sum(1 for _, _, v in series if abs(v) < 0.02) / len(series)
    wrong = sum(1 for _, _, v in series if v < -0.01) / len(series)
    settle = settle_time(samples, [joint], t_last_pub)
    ok = check(moved > 0.25, "jog net displacement", f"{moved:.3f} rad in steady window")
    ok &= check(stall < 0.10, "jog stall fraction", f"{stall:.2%} (was ~100% pre-splice)")
    ok &= check(wrong < 0.02, "jog wrong-direction fraction", f"{wrong:.2%}")
    ok &= check(settle is not None and settle < SETTLE_LIMIT, "jog settle",
        f"{settle if settle is None else round(settle, 3)}s after last command")
    return bool(ok)


def analyze_twist(samples, t_start, t_last_pub):
    # A base-frame -Z twist must produce sustained coordinated joint motion.
    total = {j: 0.0 for j in JOINTS}
    count = 0
    for t, snap in samples:
        if t_start + 0.5 <= t <= t_last_pub:
            count += 1
            for j in JOINTS:
                total[j] += abs(snap[j][1])
    if count == 0:
        print("FAIL twist: no samples")
        return False
    mean_speed = sum(total.values()) / count
    settle = settle_time(samples, JOINTS, t_last_pub)
    ok = check(mean_speed > 0.02, "twist joints moving", f"mean sum|qdot| {mean_speed:.3f} rad/s")
    ok &= check(settle is not None and settle < SETTLE_LIMIT, "twist settle",
        f"{settle if settle is None else round(settle, 3)}s after last command")
    return bool(ok)


def main():
    rclpy.init()
    node = ServoSmoke()
    if not node.wait_ready():
        print("FAIL servo/joint_states not ready")
        return 1
    all_ok = True

    # --- JOINT_JOG: the smoothness benchmark ------------------------------------
    if not node.switch_mode(ServoCommandType.Request.JOINT_JOG):
        print("FAIL switch to JOINT_JOG")
        return 1
    node.samples.clear()
    t_start = time.monotonic()
    t_last = node.stream(3.0, lambda: node.joint_jog_msg({"elfin_joint1": 0.3}), node.jog_pub)
    node.watch(2.0)
    all_ok &= analyze_jog(node.samples, "elfin_joint1", t_start, t_last)

    # --- bend away from the upright singularity before the Cartesian test -------
    # Direct jog windows to the controller topic (bypassing Servo's slow Ruckig ramp):
    # a self-tracked constant-velocity timeline, like jog_gate_demo.
    node.watch(1.0)
    # Bend elbow AND wrist: joint5 = 0 is the elfin wrist singularity, so a twist from
    # an unbent wrist trips Servo's singularity e-stop no matter the elbow.
    bend_vel = {"elfin_joint2": 0.25, "elfin_joint3": -0.30, "elfin_joint5": -0.30}
    base = {j: node.state[j][0] for j in JOINTS}
    start = time.monotonic()
    last_pub = 0.0
    while time.monotonic() - start < 3.0:
        rclpy.spin_once(node, timeout_sec=0.0)
        now = time.monotonic()
        if now - last_pub >= RATE:
            last_pub = now
            elapsed = now - start
            msg = JointTrajectory()
            msg.joint_names = JOINTS
            for k in range(6):
                t = k * 0.02
                pt = JointTrajectoryPoint()
                pt.positions = [base[j] + bend_vel.get(j, 0.0) * (elapsed + t) for j in JOINTS]
                pt.velocities = [bend_vel.get(j, 0.0) for j in JOINTS]
                pt.time_from_start.sec = 0
                pt.time_from_start.nanosec = int(t * 1e9)
                msg.points.append(pt)
            node.raw_pub.publish(msg)
        time.sleep(0.001)
    node.watch(2.0)
    print("bend pose:", {j: round(node.state[j][0], 3) for j in JOINTS})

    # --- TWIST: Cartesian jog (the end goal) ------------------------------------
    if not node.switch_mode(ServoCommandType.Request.TWIST):
        print("FAIL switch to TWIST")
        return 1
    node.samples.clear()
    t_start = time.monotonic()
    t_last = node.stream(2.0, lambda: node.twist_msg(-0.05), node.twist_pub)
    node.watch(2.0)
    all_ok &= analyze_twist(node.samples, t_start, t_last)

    node.destroy_node()
    rclpy.shutdown()
    print("RESULT", "PASS" if all_ok else "FAIL")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())

// Copyright 2026 Robotic Systems Integration, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Integration test: drive RapidCodeSystemHardware through its FULL lifecycle against
// the (phantom) RMP and confirm a real MovePVT stream actually moves the axes.
//
// Unlike test_rapidcode_system_hardware.cpp (hermetic, no rmp), this test calls
// on_configure/on_activate -- so it needs a live phantom RMP and must be the SOLE
// rmp owner (stop any running bringup first). It feeds a simple 200 ms / 200-point
// quintic move from 0.0 -> 1.0 on both joints, in 4 chunks (64,64,64,8), then polls
// read() while the firmware executes and checks the phantom axes reach 1.0 + settle.
//
// Built but NOT auto-run by `colcon test` (CMake SKIP_TEST). Run it directly:
//   ./build/rapidcode_system/test_rapidcode_system_motion
//
// HOLD / RapidSetupX mode -- keep the rmp UP so you can scope/record the move in
// RapidSetupX (rapidserver runs on the host; container uses host networking):
//   RAPIDCODE_MOTION_HOLD=1 ./build/rapidcode_system/test_rapidcode_system_motion
// It brings the rmp up, then PAUSES so you can connect RapidSetupX (axis 0/1), arm a
// Command Position recording, and trigger moves on <Enter> (each press ping-pongs
// 0<->1). Type q<Enter> to stop and tear the rmp down. Run it from an interactive
// shell so stdin is a TTY:  docker compose exec ros2_rapidcode bash

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

#include "rapidcode_system/rapidcode_system_hardware.hpp"
#include "rapidcode_trajectory_transfer/protocol.hpp"

namespace proto = rapidcode_trajectory_transfer;

namespace rapidcode_system
{

namespace
{
constexpr std::size_t kNumJoints = 2;
constexpr int kNumPoints = 200;
constexpr double kDt = 0.001;            // s per point
constexpr double kT = kDt * kNumPoints;  // 0.2 s total

// Quintic smoothstep position fraction f(tau) = 10t^3 - 15t^4 + 6t^5 and its
// derivatives (per unit displacement). Starts/ends at rest (f'=f''=0 at 0 and 1).
// `from`/`to`: start/end joint value; `point_index`: sample number in 1..kNumPoints.
struct Sample { double pos, vel, acc, jrk; };
Sample QuinticSample(int point_index, double from, double to)
{
  const double amp = to - from;
  const double tau = (static_cast<double>(point_index) * kDt) / kT;  // point_index in 1..kNumPoints -> tau in (0,1]
  const double tau2 = tau * tau, tau3 = tau2 * tau, tau4 = tau3 * tau, tau5 = tau4 * tau;
  Sample sample;
  sample.pos = from + amp * (10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5);
  sample.vel = (amp / kT) * (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4);
  sample.acc = (amp / (kT * kT)) * (60.0 * tau - 180.0 * tau2 + 120.0 * tau3);
  sample.jrk = (amp / (kT * kT * kT)) * (60.0 - 360.0 * tau + 360.0 * tau2);
  return sample;
}
}  // namespace

// MoveResult: p0/p1 = final joint 0/1 positions; flags + counters read from GPIO state.
struct MoveResult { double p0, p1; bool seen_moving, settled; int accepted, error; };

class MotionTest : public ::testing::Test
{
protected:
  const std::string gpio_ = proto::DefaultGpioName;
  RapidCodeSystemHardware hw_;
  std::vector<hardware_interface::CommandInterface> cmd_;
  std::vector<hardware_interface::StateInterface> state_;
  std::map<std::string, std::size_t> cmd_idx_;
  std::map<std::string, std::size_t> state_idx_;
  bool active_ = false;
  std::uint64_t seq_ = 0;  // monotonic command_sequence across all moves

  void SetUp() override
  {
    hardware_interface::HardwareInfo info;
    info.name = "RapidCodeSystem";
    info.type = "system";
    info.hardware_parameters["use_hardware"] = "false";
    info.hardware_parameters["cpu_affinity"] = "1";  // phantom: any core but 0
    info.hardware_parameters["sample_rate"] = "1000.0";
    for (std::size_t joint_index = 0; joint_index < kNumJoints; ++joint_index)
    {
      hardware_interface::ComponentInfo joint;
      joint.name = "joint" + std::to_string(joint_index + 1);
      info.joints.push_back(joint);
    }

    ASSERT_EQ(hw_.on_init(info), hardware_interface::CallbackReturn::SUCCESS);

    const rclcpp_lifecycle::State unconfigured;
    if (hw_.on_configure(unconfigured) != hardware_interface::CallbackReturn::SUCCESS)
    {
      GTEST_SKIP() << "on_configure failed: phantom RMP unavailable or the EtherCAT "
                      "network is live (a bringup still owns it). Stop the bringup and re-run.";
    }
    if (hw_.on_activate(unconfigured) != hardware_interface::CallbackReturn::SUCCESS)
    {
      (void)hw_.on_cleanup(unconfigured);
      GTEST_SKIP() << "on_activate failed; cannot exercise MovePVT.";
    }
    active_ = true;

    cmd_ = hw_.export_command_interfaces();
    state_ = hw_.export_state_interfaces();
    for (std::size_t idx = 0; idx < cmd_.size(); ++idx) { cmd_idx_[cmd_[idx].get_name()] = idx; }
    for (std::size_t idx = 0; idx < state_.size(); ++idx) { state_idx_[state_[idx].get_name()] = idx; }
  }

  void TearDown() override
  {
    const rclcpp_lifecycle::State active;
    if (active_)
    {
      (void)hw_.on_deactivate(active);
      (void)hw_.on_cleanup(active);
    }
  }

  void SetCmd(const std::string & suffix, double value)
  {
    auto iter = cmd_idx_.find(proto::FullName(gpio_, suffix));
    ASSERT_NE(iter, cmd_idx_.end()) << "no command interface '" << suffix << "'";
    (void)cmd_[iter->second].set_value(value);
  }
  double GetGpioState(const std::string & suffix)
  {
    auto iter = state_idx_.find(proto::FullName(gpio_, suffix));
    EXPECT_NE(iter, state_idx_.end()) << "no state interface '" << suffix << "'";
    return iter == state_idx_.end() ? 0.0 : state_[iter->second].get_optional().value();
  }
  double GetJointPos(std::size_t joint)
  {
    const std::string full = "joint" + std::to_string(joint + 1) + "/position";
    auto iter = state_idx_.find(full);
    EXPECT_NE(iter, state_idx_.end()) << "no state interface '" << full << "'";
    return iter == state_idx_.end() ? 0.0 : state_[iter->second].get_optional().value();
  }
  void Read() { (void)hw_.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(kDt)); }
  void Write() { (void)hw_.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(kDt)); }

  // Feed a 200-point quintic `from`->`to` on both joints (BEGIN + 4 chunks) and poll
  // read() while the firmware executes, printing the position/is_moving timeline.
  MoveResult FeedAndRun(double from, double to, double horizon_ms = 1200.0)
  {
    std::vector<Sample> traj(kNumPoints);
    for (int point_index = 0; point_index < kNumPoints; ++point_index) { traj[point_index] = QuinticSample(point_index + 1, from, to); }

    // --- BEGIN ---
    SetCmd(proto::interface_names::TrajectorySize, proto::Encode(kNumPoints));
    SetCmd(proto::interface_names::ValidFields,
      proto::Encode(proto::FieldMaskPosition | proto::FieldMaskVelocity |
        proto::FieldMaskAcceleration | proto::FieldMaskJerk));
    SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::Begin));
    SetCmd(proto::interface_names::CommandSequence, proto::Encode(++seq_));
    Write();

    // --- 4 AppendChunks: 64, 64, 64, 8 ---
    const int kCap = proto::ChunkCapacity;
    for (int base = 0; base < kNumPoints; base += kCap)
    {
      const int len = std::min(kCap, kNumPoints - base);
      const bool final = (base + len == kNumPoints);
      for (int slot = 0; slot < len; ++slot)
      {
        const Sample & sample = traj[base + slot];
        SetCmd(proto::SlotDuration(slot), kDt);
        for (std::size_t joint_index = 0; joint_index < kNumJoints; ++joint_index)
        {
          SetCmd(proto::JointSlotPosition(joint_index, slot), sample.pos);
          SetCmd(proto::JointSlotVelocity(joint_index, slot), sample.vel);
          SetCmd(proto::JointSlotAcceleration(joint_index, slot), sample.acc);
          SetCmd(proto::JointSlotJerk(joint_index, slot), sample.jrk);
        }
      }
      SetCmd(proto::interface_names::ChunkBaseIndex, proto::Encode(base));
      SetCmd(proto::interface_names::ChunkLen, proto::Encode(len));
      SetCmd(proto::interface_names::ChunkFinal, proto::Encode(final ? 1 : 0));
      SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::AppendChunk));
      SetCmd(proto::interface_names::CommandSequence, proto::Encode(++seq_));
      Write();
    }

    MoveResult result{};
    result.accepted = proto::Decode<int>(GetGpioState(proto::interface_names::AcceptedPointIndex));
    result.error = proto::Decode<int>(GetGpioState(proto::interface_names::ErrorCode));

    // --- run the firmware out ---
    std::printf("  move %.4f -> %.4f   accepted=%d error=%d\n", from, to, result.accepted, result.error);
    std::printf("    t(ms)  is_moving   joint1     joint2\n");
    bool seen_moving = false, settled = false;
    double last_log = -1.0, t_ms = 0.0;
    while (t_ms < horizon_ms)
    {
      Read();
      const bool moving = proto::Decode<int>(GetGpioState(proto::interface_names::IsMoving)) != 0;
      seen_moving = seen_moving || moving;
      if (last_log < 0.0 || t_ms - last_log >= 25.0)
      {
        std::printf("    %5.0f     %d        %8.5f   %8.5f\n",
          t_ms, moving ? 1 : 0, GetJointPos(0), GetJointPos(1));
        last_log = t_ms;
      }
      if (seen_moving && !moving) { settled = true; break; }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      t_ms += 2.0;
    }
    Read();
    result.p0 = GetJointPos(0); result.p1 = GetJointPos(1);
    result.seen_moving = seen_moving; result.settled = settled;
    result.error = proto::Decode<int>(GetGpioState(proto::interface_names::ErrorCode));
    std::printf("    FINAL  is_moving=%d  joint1=%.6f joint2=%.6f  seen_moving=%d settled=%d\n",
      proto::Decode<int>(GetGpioState(proto::interface_names::IsMoving)),
      result.p0, result.p1, result.seen_moving ? 1 : 0, result.settled ? 1 : 0);
    return result;
  }
};

// Single 0->1 quintic move; asserts both phantom axes reach 1.0. This is the
// automatic "does the hardware stream MovePVT correctly" check.
TEST_F(MotionTest, StreamsQuinticMoveToOnePointZero)
{
  // RapidSetupX HOLD mode: keep the rmp up and trigger moves interactively so the
  // motion can be scoped/recorded in RapidSetupX. No assertions in this mode.
  const char * hold = std::getenv("RAPIDCODE_MOTION_HOLD");
  if (hold != nullptr && std::string(hold) != "0")
  {
    std::printf("\n=== RapidSetupX HOLD mode =========================================\n");
    std::printf("The phantom rmp is UP. Connect RapidSetupX to this host's rapidserver,\n");
    std::printf("select axis 0 (joint1) and axis 1 (joint2), and arm a Command Position\n");
    std::printf("recording. Each <Enter> runs a 200 ms quintic move (ping-ponging 0<->1).\n");
    std::printf("Type q then <Enter> to stop and release the rmp.\n");
    std::printf("===================================================================\n");
    std::string line;
    double from = GetJointPos(0);
    double to = (from < 0.5) ? 1.0 : 0.0;
    while (true)
    {
      std::printf("\n[Enter] run move %.3f -> %.3f   |   q[Enter] quit: ", from, to);
      std::fflush(stdout);
      if (!std::getline(std::cin, line)) { break; }      // EOF
      if (line == "q" || line == "Q") { break; }
      FeedAndRun(from, to);
      from = GetJointPos(0);                              // continue from where it ended
      to = (to >= 0.5) ? 0.0 : 1.0;                       // ping-pong
    }
    std::printf("HOLD mode done; releasing rmp.\n");
    return;
  }

  const MoveResult result = FeedAndRun(0.0, 1.0);
  EXPECT_EQ(result.error, 0) << "hardware reported an error code during execution";
  EXPECT_TRUE(result.seen_moving) << "is_moving never went true -- the MovePVT stream never started";
  EXPECT_NEAR(result.p0, 1.0, 0.02) << "joint1 did not reach 1.0";
  EXPECT_NEAR(result.p1, 1.0, 0.02) << "joint2 did not reach 1.0";
}

}  // namespace rapidcode_system

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

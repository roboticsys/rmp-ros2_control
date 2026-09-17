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

// Integration test: connect PassthroughTrajectoryController output directly to
// RapidCodeSystemHardware and confirm the controller-generated chunks drive a
// real MultiAxis::MovePVT() move on the phantom RMP.
//
// This is intentionally a skipped/manual test. It calls hardware on_configure()
// and on_activate(), so it needs exclusive ownership of the phantom RMP. Stop any
// running bringup first, then run:
//   ./build/rapidcode_system/test_rapidcode_system_passthrough_controller_motion
//
// Motion profile under test:
//   two-axis quintic smoothstep PVAJ, zero endpoint velocity/acceleration,
//   sampled by PassthroughTrajectoryController at 1 ms and streamed as chunks.
// The 200 ms profile produces 201 dense PVAJ frames, so it spans multiple chunks:
//   64, 64, 64, 9
//
// HOLD / RapidSetupX mode -- keep the rmp UP so you can scope/record the move:
//   RAPIDCODE_PASSTHROUGH_MOTION_HOLD=1 ./build/rapidcode_system/test_rapidcode_system_passthrough_controller_motion
// It pauses after the phantom RMP is active so you can connect RapidSetupX, arm a
// Command Position recording, then press <Enter> to run each passthrough move.
// If stdin is closed (for example, `docker exec` without `-it`), HOLD mode
// auto-runs repeated moves after a short delay instead of exiting immediately.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rapidcode_passthrough_trajectory_controller/passthrough_trajectory_controller.hpp"
#include "rapidcode_system/rapidcode_system_hardware.hpp"
#include "rapidcode_trajectory_transfer/protocol.hpp"

namespace proto = rapidcode_trajectory_transfer;

namespace rapidcode_passthrough_trajectory_controller
{
namespace
{
constexpr std::size_t kNumJoints = 2;
constexpr double kDt = 0.001;
constexpr std::int64_t kMoveDurationNs = 200000000;  // 200 ms
constexpr double kJoint1Offset = 1.0;
constexpr double kJoint2Offset = 0.5;
constexpr double kGoalTolerance = 0.02;
constexpr int kAutoRunInitialDelayMs = 10000;
constexpr int kAutoRunIntervalMs = 5000;

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

std::atomic<bool> g_stop_requested{false};

void HandleStopSignal(int)
{
  g_stop_requested.store(true);
}

class ScopedStopSignalHandlers
{
public:
  ScopedStopSignalHandlers()
  {
    previous_int_ = std::signal(SIGINT, HandleStopSignal);
    previous_term_ = std::signal(SIGTERM, HandleStopSignal);
  }

  ~ScopedStopSignalHandlers()
  {
    std::signal(SIGINT, previous_int_);
    std::signal(SIGTERM, previous_term_);
  }

private:
  using SignalHandler = void (*)(int);
  SignalHandler previous_int_ = SIG_DFL;
  SignalHandler previous_term_ = SIG_DFL;
};

void SetTimeFromStart(trajectory_msgs::msg::JointTrajectoryPoint & point, std::int64_t nanoseconds)
{
  point.time_from_start.sec = static_cast<std::int32_t>(nanoseconds / 1000000000);
  point.time_from_start.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000);
}

bool EnvVarEnabled(const char * name)
{
  const char * value = std::getenv(name);
  return value != nullptr && std::string(value) != "0";
}

bool HoldModeRequested()
{
  return EnvVarEnabled("RAPIDCODE_PASSTHROUGH_MOTION_HOLD") ||
    EnvVarEnabled("RAPIDCODE_MOTION_HOLD");
}

int EnvVarInt(const char * name, int fallback)
{
  const char * value = std::getenv(name);
  if (value == nullptr) { return fallback; }
  try
  {
    return std::max(0, std::stoi(value));
  }
  catch (const std::exception &)
  {
    return fallback;
  }
}

void SleepInterruptible(std::chrono::milliseconds duration)
{
  constexpr auto step = std::chrono::milliseconds(100);
  auto remaining = duration;
  while (!g_stop_requested.load() && remaining.count() > 0)
  {
    const auto nap = std::min(step, remaining);
    std::this_thread::sleep_for(nap);
    remaining -= nap;
  }
}

std::vector<int> ExpectedChunkSizes(int point_count)
{
  std::vector<int> chunks;
  for (int base = 0; base < point_count; base += proto::ChunkCapacity)
  {
    chunks.push_back(std::min(proto::ChunkCapacity, point_count - base));
  }
  return chunks;
}

void PrintProfileSummary(
  const GoalTrajectory & traj, double from0, double to0, double from1, double to1)
{
  const std::vector<int> chunks = ExpectedChunkSizes(static_cast<int>(traj.num_points));
  std::printf("\n=== Passthrough controller -> MovePVT profile =====================\n");
  std::printf("Profile: two-axis quintic smoothstep PVAJ\n");
  std::printf("Endpoint constraints: zero velocity, zero acceleration\n");
  std::printf("Sample period: %.3f ms\n", kDt * 1000.0);
  std::printf("Dense frames: %u\n", traj.num_points);
  std::printf("Targets: joint1 %.6f -> %.6f, joint2 %.6f -> %.6f\n",
    from0, to0, from1, to1);
  std::printf("Chunk sizes:");
  for (int chunk : chunks) { std::printf(" %d", chunk); }
  std::printf("\n====================================================================\n");
}
}  // namespace

struct ControllerMotionResult
{
  std::vector<int> chunk_bases;
  std::vector<int> chunk_lengths;
  int trajectory_size = 0;
  int valid_fields = 0;
  int accepted_index = -1;
  int error_code = 0;
  bool saw_begin = false;
  bool saw_final_chunk = false;
  bool seen_moving = false;
  bool settled = false;
  double joint0 = 0.0;
  double joint1 = 0.0;
};

class PassthroughTrajectoryControllerMotionTest : public ::testing::Test
{
protected:
  const std::string gpio_ = proto::DefaultGpioName;
  const std::vector<std::string> joints_ = {"joint1", "joint2"};

  rapidcode_system::RapidCodeSystemHardware hw_;
  PassthroughTrajectoryController controller_;

  std::vector<hardware_interface::CommandInterface> command_interfaces_;
  std::vector<hardware_interface::StateInterface> state_interfaces_;
  std::map<std::string, std::size_t> command_index_;
  std::map<std::string, std::size_t> state_index_;

  bool hardware_active_ = false;
  bool controller_assigned_ = false;

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
      joint.name = joints_[joint_index];
      info.joints.push_back(joint);
    }

    ASSERT_EQ(hw_.on_init(info), hardware_interface::CallbackReturn::SUCCESS);

    const rclcpp_lifecycle::State unconfigured;
    if (hw_.on_configure(unconfigured) != hardware_interface::CallbackReturn::SUCCESS)
    {
      GTEST_SKIP() << "on_configure failed: phantom RMP unavailable or another "
                      "bringup owns it. Stop the bringup and re-run this test.";
    }
    if (hw_.on_activate(unconfigured) != hardware_interface::CallbackReturn::SUCCESS)
    {
      (void)hw_.on_cleanup(unconfigured);
      GTEST_SKIP() << "on_activate failed; cannot exercise MovePVT.";
    }
    hardware_active_ = true;

    command_interfaces_ = hw_.export_command_interfaces();
    state_interfaces_ = hw_.export_state_interfaces();
    for (std::size_t idx = 0; idx < command_interfaces_.size(); ++idx)
    {
      command_index_[command_interfaces_[idx].get_name()] = idx;
    }
    for (std::size_t idx = 0; idx < state_interfaces_.size(); ++idx)
    {
      state_index_[state_interfaces_[idx].get_name()] = idx;
    }

    ConfigureController();
    LoanHardwareInterfacesToController();
    CacheControllerInterfaceIndices();
  }

  void TearDown() override
  {
    if (controller_assigned_)
    {
      controller_.release_interfaces();
      controller_assigned_ = false;
    }
    const rclcpp_lifecycle::State active;
    if (hardware_active_)
    {
      (void)hw_.on_deactivate(active);
      (void)hw_.on_cleanup(active);
      hardware_active_ = false;
    }
  }

  void ConfigureController()
  {
    controller_.joint_names_ = joints_;
    controller_.gpio_name_ = gpio_;
    controller_.chunk_size_ = proto::ChunkCapacity;
    controller_.sample_period_ = kDt;
    controller_.first_point_tolerance_ = 1.0e-6;
    controller_.goal_position_tolerance_ = kGoalTolerance;
    controller_.command_sequence_ = 0;

    const std::size_t kcap = static_cast<std::size_t>(proto::ChunkCapacity);
    controller_.ci_slot_time_.assign(kcap, -1);
    controller_.ci_slot_joint_position_.assign(kcap * joints_.size(), -1);
    controller_.ci_slot_joint_velocity_.assign(kcap * joints_.size(), -1);
    controller_.ci_slot_joint_acceleration_.assign(kcap * joints_.size(), -1);
    controller_.ci_slot_joint_jerk_.assign(kcap * joints_.size(), -1);
    controller_.si_joint_position_.assign(joints_.size(), -1);
    controller_.si_joint_velocity_.assign(joints_.size(), -1);
  }

  void LoanHardwareInterfacesToController()
  {
    std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
    std::vector<hardware_interface::LoanedStateInterface> loaned_states;
    loaned_commands.reserve(command_interfaces_.size());
    loaned_states.reserve(state_interfaces_.size());
    for (auto & command : command_interfaces_) { loaned_commands.emplace_back(command); }
    for (auto & state : state_interfaces_) { loaned_states.emplace_back(state); }

    controller_.assign_interfaces(std::move(loaned_commands), std::move(loaned_states));
    controller_assigned_ = true;
  }

  int CommandIndex(const std::string & suffix) const
  {
    return CommandIndexFull(proto::FullName(gpio_, suffix));
  }

  int CommandIndexFull(const std::string & full_name) const
  {
    const auto iter = command_index_.find(full_name);
    if (iter == command_index_.end()) { throw std::runtime_error("missing command " + full_name); }
    return static_cast<int>(iter->second);
  }

  int StateIndex(const std::string & suffix) const
  {
    return StateIndexFull(proto::FullName(gpio_, suffix));
  }

  int StateIndexFull(const std::string & full_name) const
  {
    const auto iter = state_index_.find(full_name);
    if (iter == state_index_.end()) { throw std::runtime_error("missing state " + full_name); }
    return static_cast<int>(iter->second);
  }

  void CacheControllerInterfaceIndices()
  {
    controller_.ci_command_ = CommandIndex(proto::interface_names::CommandToken);
    controller_.ci_command_sequence_ = CommandIndex(proto::interface_names::CommandSequence);
    controller_.ci_trajectory_id_ = CommandIndex(proto::interface_names::TrajectoryId);
    controller_.ci_trajectory_size_ = CommandIndex(proto::interface_names::TrajectorySize);
    controller_.ci_valid_fields_ = CommandIndex(proto::interface_names::ValidFields);
    controller_.ci_chunk_base_ = CommandIndex(proto::interface_names::ChunkBaseIndex);
    controller_.ci_chunk_len_ = CommandIndex(proto::interface_names::ChunkLen);
    controller_.ci_chunk_final_ = CommandIndex(proto::interface_names::ChunkFinal);

    controller_.si_is_moving_ = StateIndex(proto::interface_names::IsMoving);
    controller_.si_ack_sequence_ = StateIndex(proto::interface_names::AckSequence);
    controller_.si_accepted_index_ = StateIndex(proto::interface_names::AcceptedPointIndex);
    controller_.si_error_code_ = StateIndex(proto::interface_names::ErrorCode);
    controller_.si_completed_trajectory_id_ =
      StateIndex(proto::interface_names::CompletedTrajectoryId);
    // online_enabled_ defaults true, so update() reads the committed depth every cycle
    // even though this fixture never stages a jog -- the index must be valid.
    controller_.si_committed_depth_ =
      StateIndex(proto::interface_names::CommittedDepthPoints);

    for (std::size_t slot = 0; slot < static_cast<std::size_t>(proto::ChunkCapacity); ++slot)
    {
      controller_.ci_slot_time_[slot] = CommandIndex(proto::SlotDuration(slot));
      for (std::size_t joint = 0; joint < joints_.size(); ++joint)
      {
        const std::size_t idx = slot * joints_.size() + joint;
        controller_.ci_slot_joint_position_[idx] =
          CommandIndex(proto::JointSlotPosition(joint, slot));
        controller_.ci_slot_joint_velocity_[idx] =
          CommandIndex(proto::JointSlotVelocity(joint, slot));
        controller_.ci_slot_joint_acceleration_[idx] =
          CommandIndex(proto::JointSlotAcceleration(joint, slot));
        controller_.ci_slot_joint_jerk_[idx] =
          CommandIndex(proto::JointSlotJerk(joint, slot));
      }
    }

    for (std::size_t joint = 0; joint < joints_.size(); ++joint)
    {
      controller_.si_joint_position_[joint] = StateIndexFull(joints_[joint] + "/position");
      controller_.si_joint_velocity_[joint] = StateIndexFull(joints_[joint] + "/velocity");
    }
  }

  double CommandValue(const std::string & suffix) const
  {
    return command_interfaces_[static_cast<std::size_t>(CommandIndex(suffix))]
      .get_optional().value_or(0.0);
  }

  double StateValue(const std::string & suffix) const
  {
    return state_interfaces_[static_cast<std::size_t>(StateIndex(suffix))]
      .get_optional().value_or(0.0);
  }

  double JointPosition(std::size_t joint) const
  {
    return state_interfaces_[static_cast<std::size_t>(
      StateIndexFull(joints_[joint] + "/position"))].get_optional().value_or(0.0);
  }

  int CommandToken() const
  {
    return proto::Decode<int>(CommandValue(proto::interface_names::CommandToken));
  }

  int CommandSequence() const
  {
    return proto::Decode<int>(CommandValue(proto::interface_names::CommandSequence));
  }

  void Read()
  {
    (void)hw_.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(kDt));
  }

  void Write()
  {
    (void)hw_.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(kDt));
  }

  void UpdateController()
  {
    ASSERT_EQ(controller_.update(
      rclcpp::Time(0), rclcpp::Duration::from_seconds(kDt)),
      controller_interface::return_type::OK);
  }

  FollowJointTrajectory::Goal MakeGoal(double start0, double start1) const
  {
    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = joints_;
    goal.trajectory.points.resize(2);

    auto & start = goal.trajectory.points[0];
    start.positions = {start0, start1};
    start.velocities = {0.0, 0.0};
    start.accelerations = {0.0, 0.0};
    SetTimeFromStart(start, 0);

    auto & end = goal.trajectory.points[1];
    end.positions = {start0 + kJoint1Offset, start1 + kJoint2Offset};
    end.velocities = {0.0, 0.0};
    end.accelerations = {0.0, 0.0};
    SetTimeFromStart(end, kMoveDurationNs);

    return goal;
  }

  std::shared_ptr<GoalTrajectory> Remap(const FollowJointTrajectory::Goal & goal) const
  {
    return controller_.RemapTrajectory(goal);
  }

  void InjectPendingTrajectory(const std::shared_ptr<GoalTrajectory> & traj)
  {
    // Reset the multi-goal pipeline (single-threaded test; no executor spins, so the
    // monitor timer never fires -- direct member access is safe here).
    controller_.feeding_.reset();
    controller_.pending_.clear();
    controller_.inflight_.clear();
    controller_.next_trajectory_id_ = 1;
    controller_.have_last_end_ = false;
    controller_.cancel_requested_.store(false);
    controller_.incoming_.clear();
    controller_.monitored_goals_.clear();

    // Enqueue with no action handle -- this test drives command output + hardware state
    // directly, and update()'s Finish/Abort helpers null-check the goal handle.
    PassthroughTrajectoryController::GoalEntry entry;
    entry.traj = traj;
    controller_.pending_.push_back(std::move(entry));
  }

  // ~/stop and ~/reset_fault plumbing (friend access): mirror what the executor-side
  // services do -- latch first, then request -- so the RT round trip is driven exactly
  // as in production.
  void RequestOperatorStop()
  {
    controller_.stop_latched_.store(true);
    controller_.stop_requested_.store(true);
  }
  void RequestResetFault() { controller_.reset_requested_.store(true); }
  bool StopLatched() const { return controller_.stop_latched_.load(); }
  bool PipelineEmpty() const
  {
    return !controller_.feeding_.has_value() && controller_.pending_.empty() &&
      controller_.inflight_.empty();
  }

  // Queue an additional (handle-less) goal behind whatever is already pending. A fixture
  // method because friendship (private GoalEntry/pending_ access) is not inherited by the
  // TEST_F-generated subclasses.
  void EnqueueGoalBehind(const std::shared_ptr<GoalTrajectory> & traj)
  {
    PassthroughTrajectoryController::GoalEntry entry;
    entry.traj = traj;
    controller_.pending_.push_back(std::move(entry));
  }

  ControllerMotionResult RunControllerToHardware(const GoalTrajectory & traj)
  {
    ControllerMotionResult result;
    bool final_chunk_acked = false;

    for (int cycle = 0; cycle < 2000; ++cycle)
    {
      Read();
      const bool moving =
        proto::Decode<int>(StateValue(proto::interface_names::IsMoving)) != 0;
      result.seen_moving = result.seen_moving || moving;
      if (final_chunk_acked && result.seen_moving && !moving)
      {
        result.settled = true;
        break;
      }

      UpdateController();
      const int token = CommandToken();
      const int sequence = CommandSequence();

      if (token == static_cast<int>(proto::CommandToken::Begin))
      {
        result.saw_begin = true;
        result.trajectory_size =
          proto::Decode<int>(CommandValue(proto::interface_names::TrajectorySize));
        result.valid_fields =
          proto::Decode<int>(CommandValue(proto::interface_names::ValidFields));
      }
      else if (token == static_cast<int>(proto::CommandToken::AppendChunk))
      {
        const int base =
          proto::Decode<int>(CommandValue(proto::interface_names::ChunkBaseIndex));
        const int len = proto::Decode<int>(CommandValue(proto::interface_names::ChunkLen));
        const bool final =
          proto::Decode<int>(CommandValue(proto::interface_names::ChunkFinal)) != 0;
        result.chunk_bases.push_back(base);
        result.chunk_lengths.push_back(len);
        result.saw_final_chunk = result.saw_final_chunk || final;
      }

      Write();
      result.accepted_index =
        proto::Decode<int>(StateValue(proto::interface_names::AcceptedPointIndex));
      result.error_code = proto::Decode<int>(StateValue(proto::interface_names::ErrorCode));

      const int ack_sequence =
        proto::Decode<int>(StateValue(proto::interface_names::AckSequence));
      final_chunk_acked = result.saw_final_chunk && ack_sequence == sequence &&
        result.accepted_index == static_cast<int>(traj.num_points) - 1;

      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    Read();
    result.joint0 = JointPosition(0);
    result.joint1 = JointPosition(1);
    result.accepted_index =
      proto::Decode<int>(StateValue(proto::interface_names::AcceptedPointIndex));
    result.error_code = proto::Decode<int>(StateValue(proto::interface_names::ErrorCode));
    return result;
  }

  ControllerMotionResult RunHoldMove(
    double start0, double start1, double target0, double target1)
  {
    auto goal = MakeGoal(start0, start1);
    goal.trajectory.points.back().positions = {target0, target1};
    const auto traj = Remap(goal);
    if (!traj)
    {
      ADD_FAILURE() << "controller failed to remap passthrough hold-mode trajectory";
      return {};
    }

    PrintProfileSummary(*traj, start0, target0, start1, target1);
    InjectPendingTrajectory(traj);
    const ControllerMotionResult result = RunControllerToHardware(*traj);
    std::printf("  result: accepted=%d error=%d seen_moving=%d settled=%d"
                " joint1=%.6f joint2=%.6f\n",
      result.accepted_index, result.error_code, result.seen_moving ? 1 : 0, result.settled ? 1 : 0,
      result.joint0, result.joint1);
    return result;
  }

  void RunAutoRepeatHoldMode()
  {
    const int initial_delay_ms = EnvVarInt(
      "RAPIDCODE_PASSTHROUGH_MOTION_AUTORUN_INITIAL_DELAY_MS", kAutoRunInitialDelayMs);
    const int interval_ms = EnvVarInt(
      "RAPIDCODE_PASSTHROUGH_MOTION_AUTORUN_INTERVAL_MS", kAutoRunIntervalMs);

    std::printf("\nstdin is closed, so interactive <Enter> control is unavailable.\n");
    std::printf("Auto-running repeated passthrough moves every %.3f s after an initial %.3f s delay.\n",
      static_cast<double>(interval_ms) / 1000.0, static_cast<double>(initial_delay_ms) / 1000.0);
    std::printf("Use `docker exec -it ...` for manual <Enter>/q control, or Ctrl+C to stop this run.\n");
    std::fflush(stdout);

    SleepInterruptible(std::chrono::milliseconds(initial_delay_ms));
    while (!g_stop_requested.load())
    {
      Read();
      const double start0 = JointPosition(0);
      const double start1 = JointPosition(1);
      const double direction = start0 < 0.5 ? 1.0 : -1.0;
      RunHoldMove(
        start0, start1, start0 + direction * kJoint1Offset, start1 + direction * kJoint2Offset);
      SleepInterruptible(std::chrono::milliseconds(interval_ms));
    }
  }

  void RunRapidSetupXHoldMode()
  {
    ScopedStopSignalHandlers signal_handlers;
    g_stop_requested.store(false);

    std::printf("\n=== RapidSetupX HOLD mode: passthrough controller ==================\n");
    std::printf("The phantom RMP is UP. Connect RapidSetupX to this host's rapidserver,\n");
    std::printf("select axis 0 (joint1) and axis 1 (joint2), and arm a Command Position\n");
    std::printf("recording. Each <Enter> runs a controller-generated 200 ms quintic\n");
    std::printf("PVAJ passthrough move. Type q then <Enter> to stop and release the RMP.\n");
    std::printf("Accepted env vars: RAPIDCODE_PASSTHROUGH_MOTION_HOLD=1 or RAPIDCODE_MOTION_HOLD=1\n");
    std::printf("====================================================================\n");

    std::string line;
    while (!g_stop_requested.load())
    {
      Read();
      const double start0 = JointPosition(0);
      const double start1 = JointPosition(1);
      const double direction = start0 < 0.5 ? 1.0 : -1.0;
      const double target0 = start0 + direction * kJoint1Offset;
      const double target1 = start1 + direction * kJoint2Offset;

      std::printf("\n[Enter] run passthrough move joint1 %.3f -> %.3f, joint2 %.3f -> %.3f"
                  "   |   q[Enter] quit: ",
        start0, target0, start1, target1);
      std::fflush(stdout);
      if (!std::getline(std::cin, line))
      {
        RunAutoRepeatHoldMode();
        break;
      }
      if (line == "q" || line == "Q") { break; }

      (void)RunHoldMove(start0, start1, target0, target1);
    }

    std::printf("HOLD mode done; releasing RMP.\n");
  }
};

TEST_F(PassthroughTrajectoryControllerMotionTest, ControllerChunksDriveMultiChunkMovePvt)
{
  if (HoldModeRequested())
  {
    RunRapidSetupXHoldMode();
    return;
  }

  Read();
  const double start0 = JointPosition(0);
  const double start1 = JointPosition(1);
  const double target0 = start0 + kJoint1Offset;
  const double target1 = start1 + kJoint2Offset;

  const auto goal = MakeGoal(start0, start1);
  const auto traj = Remap(goal);
  ASSERT_TRUE(traj);
  ASSERT_GT(traj->num_points, static_cast<uint32_t>(proto::ChunkCapacity))
    << "test must command at least one profile spanning multiple chunks";

  PrintProfileSummary(*traj, start0, target0, start1, target1);
  InjectPendingTrajectory(traj);

  const ControllerMotionResult result = RunControllerToHardware(*traj);

  EXPECT_TRUE(result.saw_begin) << "controller never emitted BEGIN";
  EXPECT_EQ(result.trajectory_size, static_cast<int>(traj->num_points));
  EXPECT_EQ(result.valid_fields,
    proto::FieldMaskPosition | proto::FieldMaskVelocity |
      proto::FieldMaskAcceleration | proto::FieldMaskJerk);

  const std::vector<int> expected_chunks = ExpectedChunkSizes(static_cast<int>(traj->num_points));
  EXPECT_EQ(result.chunk_lengths, expected_chunks);
  ASSERT_EQ(result.chunk_bases.size(), expected_chunks.size());
  for (std::size_t idx = 0, base = 0; idx < expected_chunks.size(); ++idx)
  {
    EXPECT_EQ(result.chunk_bases[idx], static_cast<int>(base));
    base += static_cast<std::size_t>(expected_chunks[idx]);
  }
  EXPECT_TRUE(result.saw_final_chunk) << "controller never emitted final chunk";
  EXPECT_EQ(result.accepted_index, static_cast<int>(traj->num_points) - 1);
  EXPECT_EQ(result.error_code, 0) << "hardware reported an error while consuming controller chunks";
  EXPECT_TRUE(result.seen_moving) << "is_moving never went true; MovePVT did not start";
  EXPECT_TRUE(result.settled) << "motion did not settle before the test horizon";
  EXPECT_NEAR(result.joint0, target0, kGoalTolerance) << "joint1 did not reach target";
  EXPECT_NEAR(result.joint1, target1, kGoalTolerance) << "joint2 did not reach target";
}

// Operator stop (~/stop) against the real hardware plugin + phantom RMP, end to end:
// mid-motion the controller aborts the pipeline and emits Cmd::Stop -> HandleStop runs
// MultiAxis::Stop() -- the group decelerates to rest with NO error latch (a commanded
// stop is not a fault). The latch then holds until ~/reset_fault emits Cmd::Reset ->
// HandleReset ClearFaults the commanded STOPPED group back to IDLE, and a fresh goal
// runs to completion. Proves the whole stop -> latch -> re-arm -> resume round trip.
TEST_F(PassthroughTrajectoryControllerMotionTest, OperatorStopDeceleratesAndResetRearms)
{
  if (HoldModeRequested()) { GTEST_SKIP() << "interactive hold mode requested"; }

  Read();
  const double start0 = JointPosition(0);
  const double start1 = JointPosition(1);
  const double target0 = start0 + kJoint1Offset;

  // Stretch the profile to 1 s so the stop lands well inside the motion.
  auto goal = MakeGoal(start0, start1);
  SetTimeFromStart(goal.trajectory.points.back(), 1000000000);
  const auto traj = Remap(goal);
  ASSERT_TRUE(traj);
  InjectPendingTrajectory(traj);

  // Phase 1: drive until the arm is measurably mid-motion.
  bool mid_motion = false;
  for (int cycle = 0; cycle < 2000 && !mid_motion; ++cycle)
  {
    Read();
    UpdateController();
    Write();
    mid_motion = std::fabs(JointPosition(0) - start0) > 0.05 &&
      proto::Decode<int>(StateValue(proto::interface_names::IsMoving)) != 0;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_TRUE(mid_motion) << "move never got underway; cannot exercise a mid-motion stop";

  // Phase 2: stop. Cycle until the group settles at rest; error_code must stay 0.
  RequestOperatorStop();
  bool saw_stop_token = false;
  bool settled = false;
  int error_code = 0;
  for (int cycle = 0; cycle < 2000 && !settled; ++cycle)
  {
    Read();
    UpdateController();
    saw_stop_token = saw_stop_token ||
      CommandToken() == static_cast<int>(proto::CommandToken::Stop);
    Write();
    error_code = proto::Decode<int>(StateValue(proto::interface_names::ErrorCode));
    if (error_code != 0) { break; }
    settled = saw_stop_token &&
      proto::Decode<int>(StateValue(proto::interface_names::IsMoving)) == 0;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  Read();
  const double stop0 = JointPosition(0);
  std::printf("  stop: joint1 held at %.4f (target was %.4f), error=%d\n",
    stop0, target0, error_code);
  EXPECT_TRUE(saw_stop_token) << "controller never emitted Cmd::Stop";
  EXPECT_TRUE(settled) << "group did not settle at rest after the stop";
  EXPECT_EQ(error_code, 0) << "a commanded stop must not latch an error";
  EXPECT_TRUE(PipelineEmpty()) << "the stop must abort the whole goal pipeline";
  EXPECT_LT(std::fabs(stop0 - start0), kJoint1Offset - 0.1)
    << "the arm reached the target anyway; the stop did nothing";
  EXPECT_TRUE(StopLatched());

  // Phase 3: release. Cmd::Reset must re-arm the hardware (ClearFaults the commanded
  // STOPPED group back to IDLE) and drop the latch.
  RequestResetFault();
  for (int cycle = 0; cycle < 200 && StopLatched(); ++cycle)
  {
    Read();
    UpdateController();
    Write();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_FALSE(StopLatched()) << "~/reset_fault did not release the stop latch";

  // Phase 4: a fresh goal from the stopped pose must run to completion.
  Read();
  const double resume0 = JointPosition(0);
  const double resume1 = JointPosition(1);
  const ControllerMotionResult result =
    RunHoldMove(resume0, resume1, resume0 + 0.3, resume1 + 0.15);
  EXPECT_EQ(result.error_code, 0) << "post-release move errored; the re-arm is broken";
  EXPECT_TRUE(result.settled) << "post-release move did not settle";
  EXPECT_NEAR(result.joint0, resume0 + 0.3, kGoalTolerance)
    << "post-release move missed its target";
}

// Two self-contained (finalize) goals PIPELINED against the real (phantom) hardware: both
// are queued up front, so goal 2 is pending/streamed while goal 1 is still executing. The
// hardware must SERIALIZE the reopen -- DrainQueue's reopen gate holds goal 2's fresh
// MovePVT until goal 1's finalized move drains and the group returns to IDLE -- so there
// is no firmware path error (Error 3856). completed_trajectory_id must advance to 2 and
// the arm must reach goal 2's target. Proves per-trajectory completion + the gated reopen
// end-to-end (no wedge, no path error, no OUT_OF_FRAMES).
TEST_F(PassthroughTrajectoryControllerMotionTest, TwoGoalsPipelineCompletion)
{
  if (HoldModeRequested()) { GTEST_SKIP() << "interactive hold mode requested"; }

  Read();
  const double s0 = JointPosition(0);  // goal 1 start (joint 0) == current pose
  const double s1 = JointPosition(1);  // goal 1 start (joint 1) == current pose
  const double m0 = s0 + kJoint1Offset;  // goal 1 target == goal 2 start (joint 0)
  const double m1 = s1 + kJoint2Offset;  // goal 1 target == goal 2 start (joint 1)
  const double e0 = m0 + kJoint1Offset;  // goal 2 target (joint 0)
  const double e1 = m1 + kJoint2Offset;  // goal 2 target (joint 1)

  const auto t1 = Remap(MakeGoal(s0, s1));  // s -> m
  const auto t2 = Remap(MakeGoal(m0, m1));  // m -> e
  ASSERT_TRUE(t1);
  ASSERT_TRUE(t2);

  // Queue both up front: goal 2 sits pending while goal 1 streams/executes.
  InjectPendingTrajectory(t1);
  EnqueueGoalBehind(t2);

  std::vector<std::uint64_t> begin_ids;
  int last_final_id1 = -1;
  int last_final_id2 = -1;
  std::uint64_t last_begin_id = 0;
  std::uint64_t max_completed = 0;
  int error_code = 0;
  bool settled = false;

  for (int cycle = 0; cycle < 8000; ++cycle)
  {
    Read();
    max_completed = std::max<std::uint64_t>(max_completed,
      proto::Decode<std::uint64_t>(StateValue(proto::interface_names::CompletedTrajectoryId)));
    error_code = proto::Decode<int>(StateValue(proto::interface_names::ErrorCode));
    const bool moving = proto::Decode<int>(StateValue(proto::interface_names::IsMoving)) != 0;
    if (error_code != 0) { break; }
    if (max_completed >= 2 && !moving) { settled = true; break; }

    UpdateController();
    const int token = CommandToken();
    if (token == static_cast<int>(proto::CommandToken::Begin))
    {
      last_begin_id =
        proto::Decode<std::uint64_t>(CommandValue(proto::interface_names::TrajectoryId));
      begin_ids.push_back(last_begin_id);
    }
    else if (token == static_cast<int>(proto::CommandToken::AppendChunk))
    {
      const int final = proto::Decode<int>(CommandValue(proto::interface_names::ChunkFinal));
      if (last_begin_id == 1) { last_final_id1 = final; }
      else if (last_begin_id == 2) { last_final_id2 = final; }
    }
    Write();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  Read();
  const double j0 = JointPosition(0);  // final pose (joint 0) after both goals
  const double j1 = JointPosition(1);  // final pose (joint 1) after both goals
  std::printf("  two-goal(pipelined): begins=%zu completed=%llu error=%d settled=%d j0=%.4f j1=%.4f\n",
    begin_ids.size(), static_cast<unsigned long long>(max_completed), error_code,
    settled ? 1 : 0, j0, j1);

  EXPECT_EQ(error_code, 0) << "hardware reported an error during the pipelined move";
  ASSERT_EQ(begin_ids.size(), 2u) << "expected exactly two BEGINs (one per goal)";
  EXPECT_EQ(begin_ids[0], 1u);
  EXPECT_EQ(begin_ids[1], 2u);
  EXPECT_TRUE(settled) << "pipeline did not complete both goals and settle";
  EXPECT_GE(max_completed, 2u) << "completed_trajectory_id never reached goal 2";
  EXPECT_EQ(last_final_id1, 1) << "goal 1 self-contained (finalize) -> trailing chunk is final";
  EXPECT_EQ(last_final_id2, 1) << "goal 2 self-contained (finalize) -> trailing chunk is final";
  EXPECT_NEAR(j0, e0, kGoalTolerance) << "joint1 did not reach goal 2 target";
  EXPECT_NEAR(j1, e1, kGoalTolerance) << "joint2 did not reach goal 2 target";
}

}  // namespace rapidcode_passthrough_trajectory_controller

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

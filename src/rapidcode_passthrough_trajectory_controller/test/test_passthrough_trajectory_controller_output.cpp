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

// Unit tests for the PassthroughTrajectoryController trajectory_transfer output.
//
// SCOPE: verify the raw doubles written to the controller's ros2_control command
// interfaces before RapidCodeSystemHardware or MovePVT enter the loop. The test
// injects fake command/state interfaces, feeds a remapped dense trajectory through
// update(), and checks BEGIN/chunk sequencing plus slot-major P/V/A/J payloads.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rapidcode_passthrough_trajectory_controller/passthrough_trajectory_controller.hpp"
#include "rapidcode_trajectory_transfer/protocol.hpp"

namespace proto = rapidcode_trajectory_transfer;

namespace rapidcode_passthrough_trajectory_controller
{
namespace
{
constexpr double kSamplePeriod = 0.001;
constexpr std::int64_t kMoveDurationNs = 130000000;  // 130 ms -> 130 samples + final knot
constexpr double kMoveDuration = static_cast<double>(kMoveDurationNs) / 1.0e9;
constexpr double kJoint1Start = 0.125;
constexpr double kJoint1End = 1.625;
constexpr double kJoint2Start = -0.250;
constexpr double kJoint2End = 0.750;

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

struct QuinticSample
{
  double position = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  double jerk = 0.0;
};

std::pair<std::string, std::string> SplitInterfaceName(const std::string & full_name)
{
  const std::size_t slash = full_name.find('/');
  if (slash == std::string::npos)
  {
    throw std::runtime_error("interface name has no slash: " + full_name);
  }
  return {full_name.substr(0, slash), full_name.substr(slash + 1)};
}

void SetTimeFromStart(trajectory_msgs::msg::JointTrajectoryPoint & point, std::int64_t nanoseconds)
{
  point.time_from_start.sec = static_cast<std::int32_t>(nanoseconds / 1000000000);
  point.time_from_start.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000);
}

FollowJointTrajectory::Goal MakeTwoJointGoal(
  bool with_velocities = true, bool with_accelerations = true)
{
  FollowJointTrajectory::Goal goal;

  // Reversed from the controller order on purpose; RemapTrajectory must put the
  // output back into controller/hardware order: joint1, then joint2.
  goal.trajectory.joint_names = {"joint2", "joint1"};
  goal.trajectory.points.resize(2);

  auto & start = goal.trajectory.points[0];
  start.positions = {kJoint2Start, kJoint1Start};
  if (with_velocities) { start.velocities = {0.0, 0.0}; }
  if (with_accelerations) { start.accelerations = {0.0, 0.0}; }
  SetTimeFromStart(start, 0);

  auto & end = goal.trajectory.points[1];
  end.positions = {kJoint2End, kJoint1End};
  if (with_velocities) { end.velocities = {0.0, 0.0}; }
  if (with_accelerations) { end.accelerations = {0.0, 0.0}; }
  SetTimeFromStart(end, kMoveDurationNs);

  return goal;
}

QuinticSample ExpectedQuintic(std::size_t point_index, std::size_t num_points,
  double start, double end)
{
  if (point_index + 1 == num_points)
  {
    return {end, 0.0, 0.0, 0.0};
  }

  const double time_s = static_cast<double>(point_index) * kSamplePeriod;
  const double tau = time_s / kMoveDuration;
  const double tau2 = tau * tau;
  const double tau3 = tau2 * tau;
  const double tau4 = tau3 * tau;
  const double tau5 = tau4 * tau;
  const double amp = end - start;

  QuinticSample sample;
  sample.position = start + amp * (10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5);
  sample.velocity = (amp / kMoveDuration) *
    (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4);
  sample.acceleration = (amp / (kMoveDuration * kMoveDuration)) *
    (60.0 * tau - 180.0 * tau2 + 120.0 * tau3);
  sample.jerk = (amp / (kMoveDuration * kMoveDuration * kMoveDuration)) *
    (60.0 - 360.0 * tau + 360.0 * tau2);
  return sample;
}

// Cubic Hermite for the same rest-to-rest move (V=A=0 both ends): position 3tau^2-2tau^3,
// linear acceleration, constant jerk. The final knot is emitted separately at rest.
QuinticSample ExpectedCubic(std::size_t point_index, std::size_t num_points,
  double start, double end)
{
  if (point_index + 1 == num_points)
  {
    return {end, 0.0, 0.0, 0.0};
  }

  const double time_s = static_cast<double>(point_index) * kSamplePeriod;
  const double tau = time_s / kMoveDuration;
  const double tau2 = tau * tau;
  const double amp = end - start;

  QuinticSample sample;
  sample.position = start + amp * (3.0 * tau2 - 2.0 * tau2 * tau);
  sample.velocity = (amp / kMoveDuration) * (6.0 * tau - 6.0 * tau2);
  sample.acceleration = (amp / (kMoveDuration * kMoveDuration)) * (6.0 - 12.0 * tau);
  sample.jerk = (amp / (kMoveDuration * kMoveDuration * kMoveDuration)) * (-12.0);
  return sample;
}

// Linear fit for a position-only single-segment move: constant secant velocity, zero
// acceleration and jerk. The final knot is emitted separately (velocity absent -> 0).
QuinticSample ExpectedLinear(std::size_t point_index, std::size_t num_points,
  double start, double end)
{
  if (point_index + 1 == num_points)
  {
    return {end, 0.0, 0.0, 0.0};
  }

  const double time_s = static_cast<double>(point_index) * kSamplePeriod;
  const double tau = time_s / kMoveDuration;
  const double amp = end - start;

  QuinticSample sample;
  sample.position = start + amp * tau;
  sample.velocity = amp / kMoveDuration;
  sample.acceleration = 0.0;
  sample.jerk = 0.0;
  return sample;
}

void ExpectNearRelative(double actual, double expected, const char * label)
{
  const double tolerance = std::max(1.0e-9, std::abs(expected) * 1.0e-9);
  EXPECT_NEAR(actual, expected, tolerance) << label;
}

std::vector<std::string> SplitCsvFields(const std::string & line)
{
  std::vector<std::string> fields;
  std::size_t start = 0;
  while (true)
  {
    const std::size_t comma = line.find(',', start);
    if (comma == std::string::npos) { fields.push_back(line.substr(start)); break; }
    fields.push_back(line.substr(start, comma - start));
    start = comma + 1;
  }
  return fields;
}

std::vector<std::string> SplitNonEmptyLines(const std::string & text)
{
  std::vector<std::string> lines;
  std::size_t start = 0;
  while (start < text.size())
  {
    const std::size_t newline = text.find('\n', start);
    const std::size_t end = (newline == std::string::npos) ? text.size() : newline;
    if (end > start) { lines.push_back(text.substr(start, end - start)); }
    if (newline == std::string::npos) { break; }
    start = newline + 1;
  }
  return lines;
}

}  // namespace

class PassthroughTrajectoryControllerTest : public ::testing::Test
{
protected:
  const std::string gpio_ = proto::DefaultGpioName;
  const std::vector<std::string> joints_ = {"joint1", "joint2"};

  std::vector<double> command_values_;
  std::vector<double> state_values_;
  std::vector<hardware_interface::CommandInterface> command_handles_;
  std::vector<hardware_interface::StateInterface> state_handles_;
  std::map<std::string, std::size_t> command_index_;
  std::map<std::string, std::size_t> state_index_;
  PassthroughTrajectoryController controller_;

  void SetUp() override
  {
    controller_.joint_names_ = joints_;
    controller_.gpio_name_ = gpio_;
    controller_.chunk_size_ = proto::ChunkCapacity;
    controller_.sample_period_ = kSamplePeriod;
    controller_.first_point_tolerance_ = 1.0e-9;
    controller_.goal_position_tolerance_ = 1.0e-9;
    controller_.command_sequence_ = 0;

    // Online (jog) working set, mirroring on_activate (the test bypasses the lifecycle).
    // Limits are generous so BlendDuration is driven by the velocity delta, not clamping.
    controller_.online_enabled_ = true;
    controller_.online_producer_timeout_ = 0.05;
    controller_.online_committed_horizon_ = 0.08;
    controller_.online_low_water_ = 0.03;
    // Feed-gate band in points (the fixture bypasses on_configure's conversion):
    // 30/80 points at the 1ms sample grid = the 0.03/0.08s parameters above. Tests
    // leave committed_depth_points at 0 unless they exercise the gate, so the gate
    // stays wide open (refilling) for the pre-gate online tests.
    controller_.online_low_water_points_ = 30;
    controller_.online_high_water_points_ = 80;
    controller_.online_max_velocity_.assign(joints_.size(), 2.0);
    controller_.online_max_acceleration_.assign(joints_.size(), 5.0);
    controller_.online_max_jerk_.assign(joints_.size(), 50.0);
    controller_.online_committed_pos_.assign(joints_.size(), 0.0);
    controller_.online_committed_vel_.assign(joints_.size(), 0.0);
    controller_.online_stop_traj_ = std::make_shared<GoalTrajectory>(joints_.size(), 4096);
    controller_.online_splice_traj_ = std::make_shared<GoalTrajectory>(joints_.size(), 4096);
    controller_.online_splice_vel_.assign(joints_.size(), 0.0);
    controller_.online_rest_vel_.assign(joints_.size(), 0.0);
    controller_.online_identity_mapping_.resize(joints_.size());
    for (std::size_t joint_index = 0; joint_index < joints_.size(); ++joint_index)
    {
      controller_.online_identity_mapping_[joint_index] = static_cast<int>(joint_index);
    }

    const std::size_t kcap = static_cast<std::size_t>(proto::ChunkCapacity);
    controller_.ci_slot_time_.assign(kcap, -1);
    controller_.ci_slot_joint_position_.assign(kcap * joints_.size(), -1);
    controller_.ci_slot_joint_velocity_.assign(kcap * joints_.size(), -1);
    controller_.ci_slot_joint_acceleration_.assign(kcap * joints_.size(), -1);
    controller_.ci_slot_joint_jerk_.assign(kcap * joints_.size(), -1);
    controller_.si_joint_position_.assign(joints_.size(), -1);
    controller_.si_joint_velocity_.assign(joints_.size(), -1);

    BuildFakeInterfaces();
    CacheControllerInterfaceIndices();
  }

  void TearDown() override
  {
    controller_.release_interfaces();
  }

  std::shared_ptr<GoalTrajectory> Remap(const FollowJointTrajectory::Goal & goal)
  {
    return controller_.RemapTrajectory(goal);
  }

  // interpolation_ is private; reach it through the fixture (a friend) rather than the
  // TEST_F body (friendship isn't inherited by the generated test subclass).
  void SetInterpolation(Interpolation method) { controller_.interpolation_ = method; }
  Interpolation GetInterpolation() const { return controller_.interpolation_; }

  // ~/reset_fault plumbing (friend access). The executor-side service only sets this
  // atomic, so the RT-side behavior is driven by poking it directly.
  void RequestResetFault() { controller_.reset_requested_.store(true); }
  bool ResetRequested() const { return controller_.reset_requested_.load(); }

  // ~/stop plumbing (friend access). Mirrors what the executor-side service does:
  // latch first (intakes reject from that instant), then request the RT teardown.
  void RequestOperatorStop()
  {
    controller_.stop_latched_.store(true);
    controller_.stop_requested_.store(true);
  }
  bool StopLatched() const { return controller_.stop_latched_.load(); }
  // Admission rule 1 as both intakes see it (the fixture has no node, so HandleGoal and
  // the topic callback themselves cannot run here). nullptr means "admit".
  const char * IntakeLatchReason() const { return controller_.IntakeLatchReason(); }
  // on_configure's cycle-period check, as a pure function.
  static bool CycleSustainable(double update_rate_hz, double sample_period, std::string & why)
  {
    return PassthroughTrajectoryController::CyclePeriodSustainable(
      update_rate_hz > 0.0 ? 1.0 / update_rate_hz : 0.0, sample_period, why);
  }
  // What the topic callback does with an empty JointTrajectory (the soft-stop request).
  void RequestOnlineStop() { controller_.online_stop_requested_.store(true); }
  bool OnlineStopRequested() const { return controller_.online_stop_requested_.load(); }
  // on_activate's RT-state reset (the fixture bypasses the lifecycle).
  void Reactivate() { controller_.ResetActionPipeline(); }

  std::vector<int> Mapping(const FollowJointTrajectory::Goal & goal)
  {
    return controller_.ResolveJointMapping(goal);
  }

  // Drive LogInputTrajectory into an in-memory temp file and return its text, so a test
  // can assert the CSV without leaving a file on disk (tmpfile is removed on close).
  std::string CaptureInputLog(const FollowJointTrajectory::Goal & goal)
  {
    controller_.input_csv_ = std::tmpfile();
    EXPECT_NE(controller_.input_csv_, nullptr);
    controller_.input_seq_ = 0;
    controller_.input_goal_id_ = 0;
    controller_.WriteInputTraceHeader();  // on_activate writes this in the real path
    controller_.LogInputTrajectory(goal);
    std::rewind(controller_.input_csv_);
    std::string text;
    char buffer[512];
    while (std::fgets(buffer, sizeof(buffer), controller_.input_csv_) != nullptr) { text += buffer; }
    std::fclose(controller_.input_csv_);
    controller_.input_csv_ = nullptr;
    return text;
  }

  void BuildFakeInterfaces()
  {
    const auto command_config = controller_.command_interface_configuration();
    const auto state_config = controller_.state_interface_configuration();

    command_values_.assign(command_config.names.size(), 0.0);
    state_values_.assign(state_config.names.size(), 0.0);
    command_handles_.reserve(command_config.names.size());
    state_handles_.reserve(state_config.names.size());

    for (std::size_t idx = 0; idx < command_config.names.size(); ++idx)
    {
      const auto names = SplitInterfaceName(command_config.names[idx]);
      command_index_[command_config.names[idx]] = idx;
      command_handles_.emplace_back(names.first, names.second, &command_values_[idx]);
    }
    for (std::size_t idx = 0; idx < state_config.names.size(); ++idx)
    {
      const auto names = SplitInterfaceName(state_config.names[idx]);
      state_index_[state_config.names[idx]] = idx;
      state_handles_.emplace_back(names.first, names.second, &state_values_[idx]);
    }

    std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
    std::vector<hardware_interface::LoanedStateInterface> loaned_states;
    loaned_commands.reserve(command_handles_.size());
    loaned_states.reserve(state_handles_.size());
    for (auto & command : command_handles_) { loaned_commands.emplace_back(command); }
    for (auto & state : state_handles_) { loaned_states.emplace_back(state); }

    controller_.assign_interfaces(std::move(loaned_commands), std::move(loaned_states));
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

  void SetState(const std::string & suffix, double value)
  {
    state_values_[static_cast<std::size_t>(StateIndex(suffix))] = value;
  }

  void SetJointPositionState(std::size_t joint, double value)
  {
    state_values_[static_cast<std::size_t>(StateIndexFull(joints_[joint] + "/position"))] = value;
  }

  double CommandValue(const std::string & suffix) const
  {
    return command_values_[static_cast<std::size_t>(CommandIndex(suffix))];
  }

  void Update()
  {
    ASSERT_EQ(controller_.update(
      rclcpp::Time(0), rclcpp::Duration::from_seconds(kSamplePeriod)),
      controller_interface::return_type::OK);
  }

  void InjectPendingTrajectory(const std::shared_ptr<GoalTrajectory> & traj)
  {
    ASSERT_TRUE(traj);
    ASSERT_GE(traj->positions.size(), joints_.size());

    // Reset the multi-goal pipeline to a clean idle state (single-threaded test: safe to
    // touch the RT-owned members directly).
    controller_.feeding_.reset();
    controller_.pending_.clear();
    controller_.inflight_.clear();
    controller_.next_trajectory_id_ = 1;
    controller_.command_sequence_ = 0;  // paired with AckSequence=0 below so re-inject re-arms prev_acked
    controller_.have_last_end_ = false;
    controller_.cancel_requested_.store(false);
    { std::lock_guard<std::mutex> lock(controller_.incoming_mutex_); controller_.incoming_.clear(); }
    { std::lock_guard<std::mutex> lock(controller_.monitored_mutex_); controller_.monitored_goals_.clear(); }

    SetState(proto::interface_names::IsMoving, proto::Encode(0));
    SetState(proto::interface_names::AckSequence, proto::Encode<std::uint64_t>(0));
    SetState(proto::interface_names::AcceptedPointIndex, proto::Encode(-1));
    SetState(proto::interface_names::ErrorCode, proto::Encode(0));
    SetState(proto::interface_names::CompletedTrajectoryId, proto::Encode<std::uint64_t>(0));
    for (std::size_t joint = 0; joint < joints_.size(); ++joint)
    {
      SetJointPositionState(joint, traj->positions[joint]);
    }

    // Enqueue directly into pending_ with no action handle -- this test asserts on the
    // command-interface output only, and update()'s Finish/Abort helpers null-check the
    // goal, so a handle-less entry drives the FSM without an action server.
    PassthroughTrajectoryController::GoalEntry entry;
    entry.traj = traj;
    controller_.pending_.push_back(std::move(entry));
  }

  int CurrentCommandToken() const
  {
    return proto::Decode<int>(CommandValue(proto::interface_names::CommandToken));
  }

  std::uint64_t CurrentCommandSequence() const
  {
    return proto::Decode<std::uint64_t>(
      CommandValue(proto::interface_names::CommandSequence));
  }

  int NextIndex() const
  {
    // Chunk cursor of the goal currently being fed (0 when not feeding).
    return controller_.feeding_ ? controller_.feeding_->next_index : 0;
  }

  void AckCurrentCommand()
  {
    SetState(proto::interface_names::AckSequence,
      proto::Encode<std::uint64_t>(CurrentCommandSequence()));
  }

  // --- multi-goal helpers (Phase 2) -------------------------------------------
  // A minimal dense trajectory: `npts` points, both joints held at (p0,p1). First and
  // last points coincide, so continuity holds against either live state or the seam.
  std::shared_ptr<GoalTrajectory> TinyTraj(double p0, double p1, std::uint32_t npts = 3) const
  {
    auto traj = std::make_shared<GoalTrajectory>(joints_.size(), npts);
    traj->valid_fields = proto::FieldMaskPosition | proto::FieldMaskVelocity |
      proto::FieldMaskAcceleration | proto::FieldMaskJerk;
    for (std::uint32_t point_index = 0; point_index < npts; ++point_index)
    {
      traj->durations.push_back(kSamplePeriod);
      for (std::size_t joint_index = 0; joint_index < joints_.size(); ++joint_index)
      {
        traj->positions.push_back(joint_index == 0 ? p0 : p1);
        traj->velocities.push_back(0.0);
        traj->accelerations.push_back(0.0);
        traj->jerks.push_back(0.0);
      }
    }
    traj->num_points = npts;
    return traj;
  }

  // Queue an extra goal behind whatever is already pending (no state reset).
  void EnqueueGoal(const std::shared_ptr<GoalTrajectory> & traj)
  {
    PassthroughTrajectoryController::GoalEntry entry;
    entry.traj = traj;
    controller_.pending_.push_back(std::move(entry));
  }

  int CurrentTrajectoryId() const
  {
    return proto::Decode<int>(CommandValue(proto::interface_names::TrajectoryId));
  }
  int CurrentChunkFinal() const
  {
    return proto::Decode<int>(CommandValue(proto::interface_names::ChunkFinal));
  }
  void SetCompleted(std::uint64_t id)
  {
    SetState(proto::interface_names::CompletedTrajectoryId, proto::Encode(id));
  }
  std::size_t InflightCount() const { return controller_.inflight_.size(); }
  std::size_t PendingCount() const { return controller_.pending_.size(); }
  bool Feeding() const { return controller_.feeding_.has_value(); }

  // Drive update()+ack until the in-flight set reaches `target` (bounded so a bug can't
  // hang the test). Acking a no-command cycle is harmless (re-acks the last sequence).
  void DriveUntilInflight(std::size_t target)
  {
    for (int attempt = 0; attempt < 5000 && InflightCount() < target; ++attempt)
    {
      Update();
      AckCurrentCommand();
    }
  }

  // --- online (jog) helpers ---------------------------------------------------
  // Reset the pipeline + online state + hardware handshake to a clean idle, and park the
  // live joint state at `start` so an online adopt's first-point continuity check passes.
  void PrepareOnline(const std::vector<double> & start)
  {
    controller_.feeding_.reset();
    controller_.pending_.clear();
    controller_.inflight_.clear();
    controller_.next_trajectory_id_ = 1;
    controller_.command_sequence_ = 0;
    controller_.have_last_end_ = false;
    { std::lock_guard<std::mutex> lock(controller_.incoming_mutex_); controller_.incoming_.clear(); }
    { std::lock_guard<std::mutex> lock(controller_.monitored_mutex_); controller_.monitored_goals_.clear(); }
    controller_.ResetOnlineState();
    controller_.action_busy_.store(false);
    controller_.online_busy_.store(false);
    controller_.rt_incoming_online_.writeFromNonRT(std::shared_ptr<GoalTrajectory>());

    SetState(proto::interface_names::IsMoving, proto::Encode(0));
    SetState(proto::interface_names::AckSequence, proto::Encode<std::uint64_t>(0));
    SetState(proto::interface_names::AcceptedPointIndex, proto::Encode(-1));
    SetState(proto::interface_names::ErrorCode, proto::Encode(0));
    SetState(proto::interface_names::CompletedTrajectoryId, proto::Encode<std::uint64_t>(0));
    SetState(proto::interface_names::CommittedDepthPoints, proto::Encode(0));
    for (std::size_t joint = 0; joint < joints_.size(); ++joint)
    {
      SetJointPositionState(joint, start[joint]);
    }
  }

  // Report a hardware committed depth (points accepted but not executed) for the
  // online feed gate to throttle against.
  void SetCommittedDepth(int points)
  {
    SetState(proto::interface_names::CommittedDepthPoints, proto::Encode(points));
  }

  int CurrentChunkLen() const
  {
    return proto::Decode<int>(CommandValue(proto::interface_names::ChunkLen));
  }

  // Stage a snapshot as the executor-thread callback would (finalize=false stream).
  void StageOnlineSnapshot(const std::shared_ptr<GoalTrajectory> & traj)
  {
    traj->finalize = false;
    controller_.rt_incoming_online_.writeFromNonRT(traj);
  }

  // A short online window: `npts` points advancing joint0 at constant velocity `vel0`
  // (joint1 held at start1), each duration kSamplePeriod, so the committed velocity
  // seeds a real decel-to-rest tail. Point 0 sits at (start0, start1) for continuity.
  std::shared_ptr<GoalTrajectory> MovingWindow(
    double start0, double start1, double vel0, std::uint32_t npts) const
  {
    auto traj = std::make_shared<GoalTrajectory>(joints_.size(), npts);
    traj->valid_fields = proto::FieldMaskPosition | proto::FieldMaskVelocity |
      proto::FieldMaskAcceleration | proto::FieldMaskJerk;
    for (std::uint32_t point_index = 0; point_index < npts; ++point_index)
    {
      traj->durations.push_back(kSamplePeriod);
      traj->positions.push_back(start0 + vel0 * kSamplePeriod * static_cast<double>(point_index));
      traj->positions.push_back(start1);
      traj->velocities.push_back(vel0);
      traj->velocities.push_back(0.0);
      traj->accelerations.push_back(0.0);
      traj->accelerations.push_back(0.0);
      traj->jerks.push_back(0.0);
      traj->jerks.push_back(0.0);
    }
    traj->num_points = npts;
    return traj;
  }

  // Remap through the JointTrajectory overload (the one the online callback uses).
  std::shared_ptr<GoalTrajectory> RemapMsg(const trajectory_msgs::msg::JointTrajectory & traj)
  {
    return controller_.RemapTrajectory(traj);
  }

  void SetProducerTimeout(double seconds) { controller_.online_producer_timeout_ = seconds; }
  bool OnlineActive() const { return controller_.online_active_; }
  bool OnlineStopping() const { return controller_.online_stopping_; }
  bool OnlineBusyFlag() const { return controller_.online_busy_.load(); }
  bool ActionBusyFlag() const { return controller_.action_busy_.load(); }
  int OnlineNextIndex() const { return controller_.online_next_index_; }
  int TrajectorySizeCommanded() const
  {
    return proto::Decode<int>(CommandValue(proto::interface_names::TrajectorySize));
  }
  double StopTailAccel(std::size_t point, std::size_t joint) const
  {
    return controller_.online_stop_traj_->accelerations[point * joints_.size() + joint];
  }

  // --- splice (phase 3) inspection helpers -------------------------------------
  const GoalTrajectory & OnlineTraj() const { return *controller_.online_traj_; }
  double CommittedPos(std::size_t joint) const { return controller_.online_committed_pos_[joint]; }
  double CommittedVel(std::size_t joint) const { return controller_.online_committed_vel_[joint]; }
  double OnlinePos(std::size_t point, std::size_t joint) const
  {
    return controller_.online_traj_->positions[point * joints_.size() + joint];
  }
  double OnlineVel(std::size_t point, std::size_t joint) const
  {
    return controller_.online_traj_->velocities[point * joints_.size() + joint];
  }
  double OnlineSpanSeconds() const
  {
    double span = 0.0;
    for (const double dt : controller_.online_traj_->durations) { span += dt; }
    return span;
  }

  void ExpectSampleAt(const GoalTrajectory & traj, std::size_t point_index,
    std::size_t joint, const QuinticSample & expected)
  {
    const std::size_t src = point_index * joints_.size() + joint;
    ExpectNearRelative(traj.positions[src], expected.position, "position");
    ExpectNearRelative(traj.velocities[src], expected.velocity, "velocity");
    ExpectNearRelative(traj.accelerations[src], expected.acceleration, "acceleration");
    ExpectNearRelative(traj.jerks[src], expected.jerk, "jerk");
  }

  void ExpectChunkPayload(const GoalTrajectory & traj, int base, int len) const
  {
    for (int slot = 0; slot < len; ++slot)
    {
      const std::size_t point = static_cast<std::size_t>(base + slot);
      EXPECT_DOUBLE_EQ(CommandValue(proto::SlotDuration(static_cast<std::size_t>(slot))),
        traj.durations[point]) << "duration slot " << slot;
      for (std::size_t joint = 0; joint < joints_.size(); ++joint)
      {
        const std::size_t src = point * joints_.size() + joint;
        EXPECT_DOUBLE_EQ(
          CommandValue(proto::JointSlotPosition(joint, static_cast<std::size_t>(slot))),
          traj.positions[src]) << "position point " << point << " joint " << joint;
        EXPECT_DOUBLE_EQ(
          CommandValue(proto::JointSlotVelocity(joint, static_cast<std::size_t>(slot))),
          traj.velocities[src]) << "velocity point " << point << " joint " << joint;
        EXPECT_DOUBLE_EQ(
          CommandValue(proto::JointSlotAcceleration(joint, static_cast<std::size_t>(slot))),
          traj.accelerations[src]) << "acceleration point " << point << " joint " << joint;
        EXPECT_DOUBLE_EQ(
          CommandValue(proto::JointSlotJerk(joint, static_cast<std::size_t>(slot))),
          traj.jerks[src]) << "jerk point " << point << " joint " << joint;
      }
    }
  }
};

TEST_F(PassthroughTrajectoryControllerTest, RemapTrajectoryProducesDensePvajSoa)
{
  const auto goal = MakeTwoJointGoal();
  const auto traj = Remap(goal);

  ASSERT_TRUE(traj);
  const std::size_t expected_points =
    static_cast<std::size_t>(std::ceil(kMoveDuration / kSamplePeriod)) + 1u;
  EXPECT_EQ(traj->num_joints, joints_.size());
  EXPECT_EQ(traj->num_points, expected_points);
  EXPECT_EQ(traj->valid_fields,
    proto::FieldMaskPosition | proto::FieldMaskVelocity |
      proto::FieldMaskAcceleration | proto::FieldMaskJerk);
  ASSERT_EQ(traj->durations.size(), expected_points);
  ASSERT_EQ(traj->positions.size(), expected_points * joints_.size());
  ASSERT_EQ(traj->velocities.size(), expected_points * joints_.size());
  ASSERT_EQ(traj->accelerations.size(), expected_points * joints_.size());
  ASSERT_EQ(traj->jerks.size(), expected_points * joints_.size());

  for (double duration : traj->durations)
  {
    EXPECT_DOUBLE_EQ(duration, kSamplePeriod);
  }

  const std::vector<std::size_t> sample_indices = {
    0u, 1u, expected_points / 2u, expected_points - 2u, expected_points - 1u};
  for (const std::size_t index : sample_indices)
  {
    ExpectSampleAt(*traj, index, 0,
      ExpectedQuintic(index, expected_points, kJoint1Start, kJoint1End));
    ExpectSampleAt(*traj, index, 1,
      ExpectedQuintic(index, expected_points, kJoint2Start, kJoint2End));
  }
}

TEST_F(PassthroughTrajectoryControllerTest, RemapForcedCubicMatchesHermite)
{
  // Force cubic: the goal carries accelerations, but cubic must ignore them and fit a
  // cubic Hermite through (position, velocity). Constant jerk here would fail against
  // the old quintic-only resampler (whose jerk varies), so this proves the new order.
  SetInterpolation(Interpolation::Cubic);
  const auto traj = Remap(MakeTwoJointGoal());
  ASSERT_TRUE(traj);

  const std::size_t expected_points =
    static_cast<std::size_t>(std::ceil(kMoveDuration / kSamplePeriod)) + 1u;
  ASSERT_EQ(traj->num_points, expected_points);

  const std::vector<std::size_t> sample_indices = {
    0u, 1u, expected_points / 2u, expected_points - 2u, expected_points - 1u};
  for (const std::size_t index : sample_indices)
  {
    ExpectSampleAt(*traj, index, 0,
      ExpectedCubic(index, expected_points, kJoint1Start, kJoint1End));
    ExpectSampleAt(*traj, index, 1,
      ExpectedCubic(index, expected_points, kJoint2Start, kJoint2End));
  }
}

TEST_F(PassthroughTrajectoryControllerTest, RemapAutoSelectsCubicWithoutAccelerations)
{
  // Default interpolation is Auto. A velocity-only goal must resolve to cubic and must
  // not index the (absent) acceleration vectors.
  ASSERT_EQ(GetInterpolation(), Interpolation::Auto);
  const auto traj = Remap(MakeTwoJointGoal(/*with_velocities=*/true, /*with_accelerations=*/false));
  ASSERT_TRUE(traj);

  const std::size_t expected_points =
    static_cast<std::size_t>(std::ceil(kMoveDuration / kSamplePeriod)) + 1u;
  ASSERT_EQ(traj->num_points, expected_points);

  const std::vector<std::size_t> sample_indices = {
    0u, 1u, expected_points / 2u, expected_points - 2u, expected_points - 1u};
  for (const std::size_t index : sample_indices)
  {
    ExpectSampleAt(*traj, index, 0,
      ExpectedCubic(index, expected_points, kJoint1Start, kJoint1End));
    ExpectSampleAt(*traj, index, 1,
      ExpectedCubic(index, expected_points, kJoint2Start, kJoint2End));
  }
}

TEST_F(PassthroughTrajectoryControllerTest, RemapAutoSelectsLinearWithoutVelocities)
{
  // A position-only goal must resolve to linear: constant secant velocity, zero
  // acceleration and jerk, and no indexing of the absent velocity/acceleration vectors.
  const auto traj = Remap(MakeTwoJointGoal(/*with_velocities=*/false, /*with_accelerations=*/false));
  ASSERT_TRUE(traj);

  const std::size_t expected_points =
    static_cast<std::size_t>(std::ceil(kMoveDuration / kSamplePeriod)) + 1u;
  ASSERT_EQ(traj->num_points, expected_points);

  const std::vector<std::size_t> sample_indices = {
    0u, 1u, expected_points / 2u, expected_points - 2u, expected_points - 1u};
  for (const std::size_t index : sample_indices)
  {
    ExpectSampleAt(*traj, index, 0,
      ExpectedLinear(index, expected_points, kJoint1Start, kJoint1End));
    ExpectSampleAt(*traj, index, 1,
      ExpectedLinear(index, expected_points, kJoint2Start, kJoint2End));
  }
}

TEST_F(PassthroughTrajectoryControllerTest, RemapForcedQuadraticGivesConstantAcceleration)
{
  // Quadratic interpolates velocity linearly, so every segment has a single CONSTANT
  // acceleration = (v_end - v_start)/T and zero jerk. It reads only start position + both
  // velocities (not the end position), so distinct endpoint velocities are needed to see
  // motion. Build a goal in controller joint order (joint1, joint2) with nonzero velocities.
  SetInterpolation(Interpolation::Quadratic);

  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = {"joint1", "joint2"};
  goal.trajectory.points.resize(2);
  auto & start = goal.trajectory.points[0];
  start.positions = {0.0, 0.0};
  start.velocities = {0.20, -0.10};
  start.accelerations = {0.0, 0.0};
  SetTimeFromStart(start, 0);
  auto & end = goal.trajectory.points[1];
  end.positions = {1.0, -0.5};  // ignored by quadratic
  end.velocities = {0.50, 0.30};
  end.accelerations = {0.0, 0.0};
  SetTimeFromStart(end, kMoveDurationNs);

  const auto traj = Remap(goal);
  ASSERT_TRUE(traj);

  const double expected_acc_j0 = (0.50 - 0.20) / kMoveDuration;    // joint1: (v1 - v0)/T
  const double expected_acc_j1 = (0.30 - (-0.10)) / kMoveDuration;  // joint2
  const std::size_t joint_count = joints_.size();
  // Every sampled frame except the appended final knot carries the constant accel + 0 jerk.
  for (std::size_t point = 0; point + 1 < traj->num_points; ++point)
  {
    ExpectNearRelative(traj->accelerations[point * joint_count + 0], expected_acc_j0, "j0 constant accel");
    ExpectNearRelative(traj->accelerations[point * joint_count + 1], expected_acc_j1, "j1 constant accel");
    ExpectNearRelative(traj->jerks[point * joint_count + 0], 0.0, "j0 zero jerk");
    ExpectNearRelative(traj->jerks[point * joint_count + 1], 0.0, "j1 zero jerk");
  }
  // Velocity is linear: the first frame is the commanded start velocity.
  ExpectNearRelative(traj->velocities[0], 0.20, "j0 start velocity");
  ExpectNearRelative(traj->velocities[1], -0.10, "j1 start velocity");
}

TEST_F(PassthroughTrajectoryControllerTest, UpdateEmitsWellFormedBeginAndChunks)
{
  const auto traj = Remap(MakeTwoJointGoal());
  ASSERT_TRUE(traj);
  InjectPendingTrajectory(traj);

  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Begin));
  EXPECT_EQ(CurrentCommandSequence(), 1u);
  EXPECT_EQ(proto::Decode<int>(CommandValue(proto::interface_names::TrajectorySize)),
    static_cast<int>(traj->num_points));
  EXPECT_EQ(proto::Decode<int>(CommandValue(proto::interface_names::ValidFields)),
    proto::FieldMaskPosition | proto::FieldMaskVelocity |
      proto::FieldMaskAcceleration | proto::FieldMaskJerk);

  AckCurrentCommand();  // BEGIN acked -> the first loop Update emits chunk 0 (no Opening cycle)

  int expected_base = 0;
  std::uint64_t expected_sequence = 2;
  while (expected_base < static_cast<int>(traj->num_points))
  {
    Update();

    const int expected_len = std::min(
      proto::ChunkCapacity, static_cast<int>(traj->num_points) - expected_base);
    const bool expected_final =
      expected_base + expected_len == static_cast<int>(traj->num_points);

    EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::AppendChunk));
    EXPECT_EQ(CurrentCommandSequence(), expected_sequence);
    EXPECT_EQ(proto::Decode<int>(CommandValue(proto::interface_names::ChunkBaseIndex)),
      expected_base);
    EXPECT_EQ(proto::Decode<int>(CommandValue(proto::interface_names::ChunkLen)),
      expected_len);
    EXPECT_EQ(proto::Decode<int>(CommandValue(proto::interface_names::ChunkFinal)),
      expected_final ? 1 : 0);
    EXPECT_EQ(proto::Decode<int>(CommandValue(proto::interface_names::ValidFields)),
      proto::FieldMaskPosition | proto::FieldMaskVelocity |
        proto::FieldMaskAcceleration | proto::FieldMaskJerk);
    ExpectChunkPayload(*traj, expected_base, expected_len);

    expected_base += expected_len;
    AckCurrentCommand();
    ++expected_sequence;
  }
}

TEST_F(PassthroughTrajectoryControllerTest, AckGateDoesNotAdvanceOrOverwriteNextChunk)
{
  const auto traj = Remap(MakeTwoJointGoal());
  ASSERT_TRUE(traj);
  InjectPendingTrajectory(traj);

  Update();
  ASSERT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Begin));
  ASSERT_EQ(CurrentCommandSequence(), 1u);

  // BEGIN is still unacked. The controller must not advance to chunk streaming.
  Update();
  EXPECT_EQ(CurrentCommandSequence(), 1u);
  EXPECT_EQ(proto::Decode<int>(CommandValue(proto::interface_names::ChunkLen)), 0);
  EXPECT_EQ(NextIndex(), 0);

  AckCurrentCommand();
  Update();  // BEGIN acked -> emit first chunk directly (no separate Opening cycle)
  ASSERT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::AppendChunk));
  ASSERT_EQ(CurrentCommandSequence(), 2u);
  ASSERT_EQ(proto::Decode<int>(CommandValue(proto::interface_names::ChunkBaseIndex)), 0);
  ASSERT_EQ(proto::Decode<int>(CommandValue(proto::interface_names::ChunkLen)),
    proto::ChunkCapacity);
  ASSERT_EQ(NextIndex(), proto::ChunkCapacity);

  // First chunk is still unacked. A later update must not advance to base 64.
  Update();
  EXPECT_EQ(CurrentCommandSequence(), 2u);
  EXPECT_EQ(proto::Decode<int>(CommandValue(proto::interface_names::ChunkBaseIndex)), 0);
  EXPECT_EQ(proto::Decode<int>(CommandValue(proto::interface_names::ChunkLen)),
    proto::ChunkCapacity);
  EXPECT_EQ(NextIndex(), proto::ChunkCapacity);
}

TEST_F(PassthroughTrajectoryControllerTest, ResolveJointMappingReordersGoalToControllerOrder)
{
  // Goal order is {joint2, joint1}; controller order is {joint1, joint2}. The mapping
  // gives each controller joint its column in the goal: joint1 -> goal col 1, joint2 -> 0.
  const auto mapping = Mapping(MakeTwoJointGoal());
  ASSERT_EQ(mapping.size(), joints_.size());
  EXPECT_EQ(mapping[0], 1);  // joint1 is the goal's second column
  EXPECT_EQ(mapping[1], 0);  // joint2 is the goal's first column

  // A goal missing a managed joint resolves to empty (RemapTrajectory returns nullptr).
  FollowJointTrajectory::Goal missing = MakeTwoJointGoal();
  missing.trajectory.joint_names = {"joint2", "elbow"};  // joint1 absent
  EXPECT_TRUE(Mapping(missing).empty());
}

TEST_F(PassthroughTrajectoryControllerTest, LogInputTrajectoryWritesRemappedWaypoints)
{
  const std::string text = CaptureInputLog(MakeTwoJointGoal());
  const auto lines = SplitNonEmptyLines(text);
  ASSERT_EQ(lines.size(), 3u);  // header + 2 waypoints

  // 6 metadata + 3 fields x 2 joints + (seg_method,seg_T) + 6 coeffs x 2 joints = 26 columns.
  EXPECT_EQ(lines[0],
    "seq,goal_id,point,t_from_start,has_vel,has_acc,"
    "j0_pos,j0_vel,j0_acc,j1_pos,j1_vel,j1_acc,"
    "seg_method,seg_T,"
    "j0_c0,j0_c1,j0_c2,j0_c3,j0_c4,j0_c5,"
    "j1_c0,j1_c1,j1_c2,j1_c3,j1_c4,j1_c5");

  const auto row0 = SplitCsvFields(lines[1]);
  ASSERT_EQ(row0.size(), 26u);
  EXPECT_EQ(row0[1], "0");  // goal_id
  EXPECT_EQ(row0[2], "0");  // point
  ExpectNearRelative(std::stod(row0[3]), 0.0, "t_from_start[0]");
  EXPECT_EQ(row0[4], "1");  // has_vel
  EXPECT_EQ(row0[5], "1");  // has_acc
  // Remap proof: j0 is joint1, so column 6 must be joint1's start, not joint2's.
  ExpectNearRelative(std::stod(row0[6]), kJoint1Start, "j0_pos[0]");
  ExpectNearRelative(std::stod(row0[9]), kJoint2Start, "j1_pos[0]");
  // Segment 0 carries accelerations -> Auto resolves to quintic; coefficients logged.
  EXPECT_EQ(row0[12], "quintic");            // seg_method
  ExpectNearRelative(std::stod(row0[13]), kMoveDuration, "seg_T");
  // Quintic c0=start_pos, c1=start_vel(=0), c2=0.5*start_acc(=0) for this rest start.
  ExpectNearRelative(std::stod(row0[14]), kJoint1Start, "j0_c0 == start_pos");
  ExpectNearRelative(std::stod(row0[15]), 0.0, "j0_c1 == start_vel");
  ExpectNearRelative(std::stod(row0[16]), 0.0, "j0_c2 == 0.5*start_acc");

  const auto row1 = SplitCsvFields(lines[2]);
  ASSERT_EQ(row1.size(), 26u);
  EXPECT_EQ(row1[2], "1");
  ExpectNearRelative(std::stod(row1[3]), kMoveDuration, "t_from_start[1]");
  ExpectNearRelative(std::stod(row1[6]), kJoint1End, "j0_pos[1]");
  ExpectNearRelative(std::stod(row1[9]), kJoint2End, "j1_pos[1]");
  // Final point has no segment -> empty method, NaN coefficients.
  EXPECT_EQ(row1[12], "");                    // seg_method empty
  EXPECT_TRUE(std::isnan(std::stod(row1[14])));  // j0_c0 -> NaN
}

TEST_F(PassthroughTrajectoryControllerTest, LogInputTrajectoryMarksAbsentFieldsAsNaN)
{
  // Velocity present, acceleration absent: has_acc=0 and the accel columns log as NaN,
  // never a stray 0.0 that would read as a real commanded acceleration.
  const std::string text =
    CaptureInputLog(MakeTwoJointGoal(/*with_velocities=*/true, /*with_accelerations=*/false));
  const auto lines = SplitNonEmptyLines(text);
  ASSERT_GE(lines.size(), 2u);

  const auto row0 = SplitCsvFields(lines[1]);
  ASSERT_EQ(row0.size(), 26u);
  EXPECT_EQ(row0[4], "1");  // has_vel
  EXPECT_EQ(row0[5], "0");  // has_acc
  EXPECT_TRUE(std::isnan(std::stod(row0[8])));    // j0_acc -> NaN
  EXPECT_TRUE(std::isnan(std::stod(row0[11])));   // j1_acc -> NaN
  EXPECT_FALSE(std::isnan(std::stod(row0[7])));   // j0_vel present -> real number
  // No accelerations -> Auto resolves to cubic; cubic zeroes the degree-4/5 coefficients.
  EXPECT_EQ(row0[12], "cubic");                   // seg_method
  EXPECT_DOUBLE_EQ(std::stod(row0[18]), 0.0);     // j0_c4 == 0 (cubic)
  EXPECT_DOUBLE_EQ(std::stod(row0[19]), 0.0);     // j0_c5 == 0 (cubic)
}

// Multi-goal: BEGIN carries a monotonic trajectory_id, and a goal is finished (removed
// from the in-flight set) when the hardware's completed_trajectory_id reaches its id.
TEST_F(PassthroughTrajectoryControllerTest, MultiGoalTrajectoryIdsAndCompletion)
{
  InjectPendingTrajectory(TinyTraj(0.1, 0.2));  // g1
  EnqueueGoal(TinyTraj(0.1, 0.2));              // g2 queued behind g1

  Update();  // adopt g1 -> BEGIN id 1
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Begin));
  EXPECT_EQ(CurrentTrajectoryId(), 1);
  AckCurrentCommand();
  DriveUntilInflight(1);
  EXPECT_EQ(InflightCount(), 1u);

  DriveUntilInflight(2);  // g2 adopted (BEGIN id 2) and fed
  EXPECT_EQ(CurrentTrajectoryId(), 2);
  EXPECT_EQ(InflightCount(), 2u);
  EXPECT_EQ(PendingCount(), 0u);

  // Hardware finishes id 1 only -> g1 completes, g2 stays in flight.
  SetCompleted(1);
  Update();
  EXPECT_EQ(InflightCount(), 1u);

  // Then id 2 -> g2 completes.
  SetCompleted(2);
  Update();
  EXPECT_EQ(InflightCount(), 0u);
}

// Whether the trailing chunk closes the firmware move (IsFinal) is the trajectory's own
// `finalize` flag -- NOT queue state. A finalize=true goal finalizes; a finalize=false
// goal keeps the move open (the streaming / non-final case). Sequence per goal is
// BEGIN -> (ack) -> chunk; there is no separate Opening cycle.
TEST_F(PassthroughTrajectoryControllerTest, FinalizeFlagControlsTrailingChunkFinal)
{
  // finalize=true (default) -> the sole (last) chunk is IsFinal=1.
  InjectPendingTrajectory(TinyTraj(0.1, 0.2));
  Update();  // adopt -> BEGIN
  ASSERT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Begin));
  AckCurrentCommand();
  Update();  // sole chunk (is_last)
  ASSERT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::AppendChunk));
  EXPECT_EQ(CurrentChunkFinal(), 1) << "finalize=true -> trailing chunk closes the move";

  // finalize=false -> the sole (last) chunk is IsFinal=0 (move stays open).
  auto non_final = TinyTraj(0.1, 0.2);
  non_final->finalize = false;
  InjectPendingTrajectory(non_final);
  Update();  // adopt -> BEGIN
  ASSERT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Begin));
  AckCurrentCommand();
  Update();  // sole chunk (is_last)
  ASSERT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::AppendChunk));
  EXPECT_EQ(CurrentChunkFinal(), 0) << "finalize=false -> trailing chunk keeps the move open";
}

// Multi-goal: a hardware error aborts every in-flight/pending goal and keeps aborting
// goals that arrive while error_code is set; once it clears, goals adopt again. (There is
// no explicit Faulted state anymore -- the behavior is observed via the goal pipeline.)
TEST_F(PassthroughTrajectoryControllerTest, HardwareFaultAbortsAllPipelinedGoals)
{
  InjectPendingTrajectory(TinyTraj(0.1, 0.2));
  Update();  // BEGIN
  AckCurrentCommand();
  DriveUntilInflight(1);
  ASSERT_EQ(InflightCount(), 1u);

  // Fault: the in-flight goal is aborted; nothing is left feeding; no command emitted.
  SetState(proto::interface_names::ErrorCode, proto::Encode(1));
  Update();
  EXPECT_EQ(InflightCount(), 0u) << "fault aborts in-flight goals";
  EXPECT_FALSE(Feeding());
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::None));

  // A goal arriving while faulted is dropped, not adopted.
  EnqueueGoal(TinyTraj(0.1, 0.2));
  Update();
  EXPECT_EQ(PendingCount(), 0u) << "goals are aborted while the hardware is faulted";
  EXPECT_FALSE(Feeding());

  // Error clears -> a fresh goal adopts again.
  SetState(proto::interface_names::ErrorCode, proto::Encode(0));
  EnqueueGoal(TinyTraj(0.1, 0.2));
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Begin))
    << "after the fault clears, a new goal is adopted";
  EXPECT_TRUE(Feeding());
}

// on_configure refuses a cycle the firmware cannot sustain.
// The delivered bringup rates (500 Hz and 200 Hz against a 1 ms sample) pass; the
// firmware rate itself (1:1), a non-integer sample multiple, and degenerate inputs fail.
TEST_F(PassthroughTrajectoryControllerTest, ConfigureRejectsUnsustainableCyclePeriod)
{
  std::string why;
  EXPECT_TRUE(CycleSustainable(500.0, 0.001, why)) << why;   // elfin5 / two-axis bringup
  EXPECT_TRUE(CycleSustainable(200.0, 0.001, why)) << why;   // single-axis bringup
  EXPECT_TRUE(CycleSustainable(250.0, 0.002, why)) << why;   // 2 samples at a 2 ms grid
  EXPECT_TRUE(CycleSustainable(100.0, 0.001, why)) << why;   // 10 samples per point

  EXPECT_FALSE(CycleSustainable(1000.0, 0.001, why)) << "1:1 starves the PVT stream";
  EXPECT_NE(why.find("at least 2"), std::string::npos) << why;
  EXPECT_FALSE(CycleSustainable(2000.0, 0.001, why)) << "faster than the firmware";
  EXPECT_FALSE(CycleSustainable(300.0, 0.001, why)) << "3.33 samples per point";
  EXPECT_NE(why.find("whole multiple"), std::string::npos) << why;
  EXPECT_FALSE(CycleSustainable(0.0, 0.001, why)) << "no update rate";
  EXPECT_FALSE(CycleSustainable(500.0, 0.0, why)) << "no sample period";
  EXPECT_FALSE(CycleSustainable(500.0, -0.001, why)) << "negative sample period";
}

// Admission rule 1: a latched hardware fault refuses new trajectories at
// the intake, not one cycle later in update(). The intakes read the RT mirror of
// error_code, so the refusal follows the first update() that observes the fault and
// lifts on the first update() that observes it cleared. The operator/lifecycle stop
// latch refuses the same way.
TEST_F(PassthroughTrajectoryControllerTest, LatchedFaultRefusesAtAdmission)
{
  InjectPendingTrajectory(TinyTraj(0.1, 0.2));
  Update();  // healthy cycle: mirror reads 0
  EXPECT_EQ(IntakeLatchReason(), nullptr) << "healthy and unlatched: admit";

  SetState(proto::interface_names::ErrorCode, proto::Encode(7));
  Update();  // the fault branch mirrors error_code for the executor-side intakes
  ASSERT_NE(IntakeLatchReason(), nullptr) << "a latched fault refuses at admission";
  EXPECT_NE(std::string(IntakeLatchReason()).find("hardware fault"), std::string::npos);

  SetState(proto::interface_names::ErrorCode, proto::Encode(0));
  Update();
  EXPECT_EQ(IntakeLatchReason(), nullptr) << "admission resumes once the fault clears";

  RequestOperatorStop();
  ASSERT_NE(IntakeLatchReason(), nullptr) << "the stop latch refuses at admission";
  EXPECT_NE(std::string(IntakeLatchReason()).find("operator stop"), std::string::npos);
}

// Operator fault acknowledgement: while faulted, a reset request emits Cmd::Reset
// exactly once through the normal mailbox handshake (sequence bumped), after which the
// hardware clears its latch and normal operation resumes.
TEST_F(PassthroughTrajectoryControllerTest, FaultResetRequestEmitsResetWhenAcked)
{
  InjectPendingTrajectory(TinyTraj(0.1, 0.2));  // clean idle pipeline, mailbox acked
  SetState(proto::interface_names::ErrorCode, proto::Encode(3));
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::None))
    << "fault hold without an operator request emits None";

  RequestResetFault();
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Reset))
    << "an acked mailbox lets the operator request emit Reset";
  EXPECT_EQ(CurrentCommandSequence(), 1u) << "Reset is a normal sequenced command";
  EXPECT_FALSE(ResetRequested()) << "the request is consumed by the emission";

  // Reset consumed but not yet acked: the fault branch must not re-emit (no bump).
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::None));
  EXPECT_EQ(CurrentCommandSequence(), 1u);

  // The hardware consumed Reset and cleared its latch: normal operation resumes.
  AckCurrentCommand();
  SetState(proto::interface_names::ErrorCode, proto::Encode(0));
  EnqueueGoal(TinyTraj(0.1, 0.2));
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Begin))
    << "after the reset round-trip, a fresh goal adopts again";
}

// The reset request must respect the single-slot mailbox: while the previous command
// is un-acked it stays pending (retries next cycle) instead of overwriting the slot.
TEST_F(PassthroughTrajectoryControllerTest, FaultResetRequestWaitsForMailboxAck)
{
  InjectPendingTrajectory(TinyTraj(0.1, 0.2));
  Update();  // BEGIN emitted (sequence 1), deliberately NOT acked

  SetState(proto::interface_names::ErrorCode, proto::Encode(3));
  RequestResetFault();
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::None))
    << "un-acked mailbox: the request must hold, not overwrite the BEGIN slot";
  EXPECT_TRUE(ResetRequested()) << "the pending request survives to retry";

  AckCurrentCommand();  // hardware finally consumed the BEGIN
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Reset));
  EXPECT_EQ(CurrentCommandSequence(), 2u);
  EXPECT_FALSE(ResetRequested());
}

// A request made (or left over) while the hardware is healthy is discarded on the next
// cycle -- it must never emit a Reset later, which would drop a queued trajectory tail.
TEST_F(PassthroughTrajectoryControllerTest, StaleResetRequestClearedOnHealthyCycle)
{
  InjectPendingTrajectory(TinyTraj(0.1, 0.2));
  RequestResetFault();  // stale: no fault is latched
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Begin))
    << "healthy cycle proceeds normally (adopts the pending goal)";
  EXPECT_FALSE(ResetRequested()) << "the stale request is discarded, never emitted";
}

// Operator stop (~/stop): aborts the whole goal pipeline, emits one Cmd::Stop, holds
// goals rejected while latched, and releases only through ~/reset_fault (Cmd::Reset).
TEST_F(PassthroughTrajectoryControllerTest, OperatorStopAbortsGoalsAndEmitsStop)
{
  InjectPendingTrajectory(TinyTraj(0.1, 0.2));
  Update();  // BEGIN
  AckCurrentCommand();
  DriveUntilInflight(1);
  ASSERT_EQ(InflightCount(), 1u);

  RequestOperatorStop();
  Update();
  EXPECT_EQ(InflightCount(), 0u) << "stop aborts in-flight goals";
  EXPECT_FALSE(Feeding());
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Stop))
    << "the stop request emits Cmd::Stop through the mailbox";
  EXPECT_TRUE(StopLatched());

  // A goal arriving while latched is aborted, not adopted; the mailbox holds None.
  AckCurrentCommand();
  EnqueueGoal(TinyTraj(0.1, 0.2));
  Update();
  EXPECT_EQ(PendingCount(), 0u) << "goals are aborted while the stop latch holds";
  EXPECT_FALSE(Feeding());
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::None));

  // Release: ~/reset_fault emits Cmd::Reset and drops the latch; goals adopt again.
  RequestResetFault();
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Reset))
    << "the release emits Reset (re-arms the hardware, clears its STOPPED state)";
  EXPECT_FALSE(StopLatched()) << "the latch drops with the Reset emission";
  AckCurrentCommand();
  EnqueueGoal(TinyTraj(0.1, 0.2));
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Begin))
    << "after the release, a fresh goal adopts again";
}

// The stop must respect the single-slot mailbox: with the previous command un-acked it
// holds (retries next cycle) instead of overwriting the slot, and a release requested
// in the same window stays queued BEHIND the stop -- it can never overtake it.
TEST_F(PassthroughTrajectoryControllerTest, OperatorStopWaitsForMailboxAck)
{
  InjectPendingTrajectory(TinyTraj(0.1, 0.2));
  Update();  // BEGIN emitted (sequence 1), deliberately NOT acked

  RequestOperatorStop();
  RequestResetFault();  // operator mashes reset right behind the stop
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::None))
    << "un-acked mailbox: the stop holds, not overwriting the BEGIN slot";
  EXPECT_EQ(CurrentCommandSequence(), 1u);
  EXPECT_TRUE(StopLatched());

  AckCurrentCommand();  // hardware finally consumed the BEGIN
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Stop))
    << "the pending stop goes out first";
  EXPECT_EQ(CurrentCommandSequence(), 2u);
  EXPECT_TRUE(StopLatched()) << "the queued release must not clear the latch early";

  AckCurrentCommand();
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Reset))
    << "only after the stop is acked does the release emit Reset";
  EXPECT_FALSE(StopLatched());
}

// Lifecycle hard stop: deactivating mid-goal writes Cmd::Stop to the
// mailbox, drops the pipeline, and keeps the stop latch across a reactivation so goals
// are refused until ~/reset_fault emits Cmd::Reset.
TEST_F(PassthroughTrajectoryControllerTest, DeactivateEmitsStopAndHoldsLatchAcrossReactivate)
{
  InjectPendingTrajectory(TinyTraj(0.1, 0.2));
  Update();  // BEGIN
  AckCurrentCommand();
  DriveUntilInflight(1);
  ASSERT_EQ(InflightCount(), 1u);
  const std::uint64_t sequence_before = CurrentCommandSequence();

  ASSERT_EQ(controller_.on_deactivate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Stop))
    << "deactivation writes the hard stop to the interfaces";
  EXPECT_EQ(CurrentCommandSequence(), sequence_before + 1) << "a new mailbox sequence";
  EXPECT_TRUE(StopLatched());
  EXPECT_EQ(InflightCount(), 0u);
  EXPECT_EQ(PendingCount(), 0u);
  EXPECT_FALSE(Feeding());

  // Reactivate (the fixture bypasses on_activate; this is its RT-state reset). The
  // latch survives: a goal is refused and the mailbox holds None, not a second Stop.
  Reactivate();
  EXPECT_TRUE(StopLatched()) << "reactivation does not clear the lifecycle stop";
  AckCurrentCommand();
  EnqueueGoal(TinyTraj(0.1, 0.2));
  Update();
  EXPECT_EQ(PendingCount(), 0u) << "goals are refused until a reset";
  EXPECT_FALSE(Feeding());
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::None));

  // ~/reset_fault releases it and re-arms the hardware.
  RequestResetFault();
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Reset));
  EXPECT_FALSE(StopLatched());
  AckCurrentCommand();
  EnqueueGoal(TinyTraj(0.1, 0.2));
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Begin))
    << "after the reset a fresh goal adopts again";
}

// Unlike the operator stop, the lifecycle stop does not wait for the mailbox ack: the
// hardware accepts any sequence above its last ack, and the un-acked chunk it
// overwrites is motion being discarded anyway.
TEST_F(PassthroughTrajectoryControllerTest, DeactivateOverwritesUnackedMailboxSlot)
{
  InjectPendingTrajectory(TinyTraj(0.1, 0.2));
  Update();  // BEGIN emitted (sequence 1), deliberately NOT acked
  ASSERT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Begin));
  ASSERT_EQ(CurrentCommandSequence(), 1u);

  controller_.on_deactivate(rclcpp_lifecycle::State());
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Stop))
    << "the stop overwrites the un-acked BEGIN";
  EXPECT_EQ(CurrentCommandSequence(), 2u);
  EXPECT_TRUE(StopLatched());
}

// Shutdown: from ACTIVE it writes the same hard stop; from INACTIVE the
// group is already at rest and the interfaces are released, so nothing is written.
TEST_F(PassthroughTrajectoryControllerTest, ShutdownEmitsStopOnlyFromActive)
{
  InjectPendingTrajectory(TinyTraj(0.1, 0.2));
  Update();  // BEGIN
  AckCurrentCommand();
  DriveUntilInflight(1);
  const std::uint64_t sequence_before = CurrentCommandSequence();

  const rclcpp_lifecycle::State active(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  ASSERT_EQ(controller_.on_shutdown(active), controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Stop));
  EXPECT_EQ(CurrentCommandSequence(), sequence_before + 1);
  EXPECT_TRUE(StopLatched());
  EXPECT_EQ(InflightCount(), 0u);

  // A second shutdown from INACTIVE (already stopped) writes nothing more.
  const rclcpp_lifecycle::State inactive(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  ASSERT_EQ(controller_.on_shutdown(inactive), controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(CurrentCommandSequence(), sequence_before + 1) << "no new command from INACTIVE";
}

// Stop during an online jog: the open session is torn down, Cmd::Stop goes out, and a
// snapshot staged before the stop is stale -- after the release it must not ghost-reopen.
TEST_F(PassthroughTrajectoryControllerTest, OperatorStopTearsDownOnlineSession)
{
  PrepareOnline({0.0, 0.0});
  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.1, 5));
  Update();  // BEGIN (open move)
  AckCurrentCommand();
  Update();  // opening chunk
  AckCurrentCommand();
  ASSERT_TRUE(OnlineActive());

  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.1, 5));  // parked, pre-stop -> stale
  RequestOperatorStop();
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Stop));
  EXPECT_FALSE(OnlineActive()) << "the open jog session is torn down";
  EXPECT_FALSE(OnlineBusyFlag());

  // Release, then run healthy cycles: the pre-stop snapshot must stay ignored.
  AckCurrentCommand();
  RequestResetFault();
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Reset));
  AckCurrentCommand();
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::None))
    << "a snapshot staged before the stop never ghost-reopens a move";
  EXPECT_FALSE(OnlineActive());
}

// A hardware fault outranks the stop hold, and one ~/reset_fault acknowledgement
// re-arms everything: the fault's Reset also releases the stop latch and drops the
// now-moot Cmd::Stop (emitting it after the Reset would re-park the group in STOPPED).
TEST_F(PassthroughTrajectoryControllerTest, FaultResetAlsoReleasesStopLatch)
{
  InjectPendingTrajectory(TinyTraj(0.1, 0.2));
  RequestOperatorStop();
  SetState(proto::interface_names::ErrorCode, proto::Encode(5));  // fault wins the cycle
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::None));
  EXPECT_TRUE(StopLatched());

  RequestResetFault();
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Reset));
  EXPECT_FALSE(StopLatched()) << "one acknowledgement clears both latches";

  AckCurrentCommand();
  SetState(proto::interface_names::ErrorCode, proto::Encode(0));
  EnqueueGoal(TinyTraj(0.1, 0.2));
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Begin))
    << "no stale Cmd::Stop fires after the re-arm; goals adopt again";
}

// Multi-goal: a goal queued behind others (chain active) has its first-point continuity
// checked against the PREVIOUS goal's endpoint (the seam), not the live joint state --
// because the arm is mid-motion and live state is the wrong reference.
TEST_F(PassthroughTrajectoryControllerTest, QueuedGoalContinuityCheckedAgainstSeam)
{
  InjectPendingTrajectory(TinyTraj(0.1, 0.2));  // g1 ends at (0.1,0.2) -> seam
  Update();  // BEGIN g1
  AckCurrentCommand();
  DriveUntilInflight(1);
  ASSERT_EQ(InflightCount(), 1u);

  // Move live state far from the seam (simulate the arm executing mid-chain).
  SetJointPositionState(0, 5.0);
  SetJointPositionState(1, 5.0);

  // g2 starts at the seam (0.1,0.2), far from live (5,5): accepted because the check uses
  // the seam.
  EnqueueGoal(TinyTraj(0.1, 0.2));
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Begin));
  EXPECT_EQ(CurrentTrajectoryId(), 2);
  EXPECT_TRUE(Feeding());
}

// Multi-goal: a queued goal whose first point matches neither the seam is rejected (and,
// having no action handle here, is simply dropped without a BEGIN).
TEST_F(PassthroughTrajectoryControllerTest, QueuedGoalOffSeamIsRejected)
{
  InjectPendingTrajectory(TinyTraj(0.1, 0.2));
  Update();  // BEGIN g1
  AckCurrentCommand();
  DriveUntilInflight(1);
  ASSERT_EQ(InflightCount(), 1u);

  EnqueueGoal(TinyTraj(5.0, 5.0));  // off the seam (0.1,0.2)
  Update();  // adopt attempt -> continuity fails -> aborted, not fed
  EXPECT_NE(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Begin));
  EXPECT_FALSE(Feeding());
  EXPECT_EQ(PendingCount(), 0u) << "rejected goal is consumed, not left pending";
}

// --- online (jog) topic path ------------------------------------------------

// A staged jog snapshot is adopted as an OPEN move: BEGIN carries trajectory_size = -1
// (so the hardware pushes no completion threshold) and trajectory_id 0. The staged
// stream is the blend from the measured state to the window's velocity intent plus the
// constant-velocity extension (the window itself is never fed verbatim), and the
// opening chunk is a FULL chunk (gate bypass); it is never final.
TEST_F(PassthroughTrajectoryControllerTest, OnlineAdoptEmitsOpenBeginThenNonFinalChunk)
{
  PrepareOnline({0.0, 0.0});
  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.1, 5));

  Update();  // ingest -> adopt (blend from measured state) -> BEGIN (open move)
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Begin));
  EXPECT_EQ(CurrentCommandSequence(), 1u);
  EXPECT_EQ(TrajectorySizeCommanded(), -1) << "online BEGIN opens the move (size -1)";
  EXPECT_EQ(CurrentTrajectoryId(), 0) << "an open jog is not completion-tracked";
  EXPECT_TRUE(OnlineActive());
  EXPECT_TRUE(OnlineBusyFlag());
  EXPECT_NEAR(OnlinePos(0, 0), 0.0, 1.0e-9) << "blend anchors at the measured state";
  const std::size_t last = OnlineTraj().num_points - 1;
  EXPECT_NEAR(OnlineVel(last, 0), 0.1, 1.0e-9) << "tail runs at the velocity intent";

  AckCurrentCommand();
  Update();  // opening chunk: full chunk_size_, gate bypassed
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::AppendChunk));
  EXPECT_EQ(CurrentChunkFinal(), 0) << "streamed chunks never finalize";
  EXPECT_EQ(proto::Decode<int>(CommandValue(proto::interface_names::ChunkBaseIndex)), 0);
  EXPECT_EQ(CurrentChunkLen(), 64) << "opening chunk is a full chunk (watchdog floor)";
}

// Once the staged stream is fully fed, the online branch HOLDS (no command, still
// active, not retired) rather than retiring the way the action feed loop does.
TEST_F(PassthroughTrajectoryControllerTest, OnlineHoldsAtStagedTail)
{
  PrepareOnline({0.0, 0.0});
  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.1, 5));
  Update(); AckCurrentCommand();  // BEGIN
  const int staged_points = static_cast<int>(OnlineTraj().num_points);
  for (int attempt = 0; attempt < 100 && OnlineNextIndex() < staged_points; ++attempt)
  {
    Update();
    AckCurrentCommand();
  }
  ASSERT_EQ(OnlineNextIndex(), staged_points);

  Update();  // caught up: hold, well within the producer-silence timeout
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::None));
  EXPECT_TRUE(OnlineActive());
  EXPECT_FALSE(OnlineStopping());
  EXPECT_EQ(OnlineNextIndex(), staged_points) << "the staged tail is held, not retired";
}

// Splice-on-receive: a newer snapshot is re-anchored onto the committed seam and swaps
// the staged stream WITHOUT a new BEGIN -- the open move stays open.
TEST_F(PassthroughTrajectoryControllerTest, OnlineSpliceOnReceiveKeepsMoveOpen)
{
  PrepareOnline({0.0, 0.0});
  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.0, 5));  // held window; committed = (0,0)
  Update(); AckCurrentCommand();  // BEGIN
  Update(); AckCurrentCommand();  // opening chunk of the rest-blend
  ASSERT_TRUE(OnlineActive());
  ASSERT_GT(OnlineNextIndex(), 0);

  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.1, 8));  // continuous with the (0,0) seam
  Update();  // splice + feed the re-anchored blend's first chunk, no new BEGIN
  EXPECT_TRUE(OnlineActive());
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::AppendChunk))
    << "splice-on-receive keeps the move open (no new BEGIN)";
  EXPECT_EQ(CurrentChunkFinal(), 0);
}

// Online XOR action. (1) An active jog publishes online_busy_, the lock-free mirror
// HandleGoal reads to reject action goals. (2) The authoritative RT-side guard:
// ServiceOnlineStream refuses to adopt a staged jog while the action pipeline is busy,
// so the two can never both own the single-slot mailbox. (The executor-side reject /
// drop paths log via the lifecycle node, which the hermetic fixture has no way to host,
// so they are exercised in integration rather than here.)
TEST_F(PassthroughTrajectoryControllerTest, OnlineActionMutualExclusion)
{
  PrepareOnline({0.0, 0.0});
  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.1, 5));
  Update();  // adopt
  ASSERT_TRUE(OnlineActive());
  EXPECT_TRUE(OnlineBusyFlag()) << "an active jog publishes online_busy_ for HandleGoal";

  // Action pipeline busy: a staged jog is NOT adopted; the action goal owns the mailbox.
  PrepareOnline({0.0, 0.0});
  EnqueueGoal(TinyTraj(0.0, 0.0));  // pending action goal (continuous with live (0,0))
  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.1, 5));
  Update();
  EXPECT_FALSE(OnlineActive()) << "a jog is not adopted while the action pipeline is busy";
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Begin))
    << "the action goal is adopted instead";
  EXPECT_TRUE(ActionBusyFlag());
}

// The JointTrajectory overload (used by the online callback) resamples identically to
// the Goal overload (used by the action path) -- one resampler, two intakes.
TEST_F(PassthroughTrajectoryControllerTest, RemapJointTrajectoryOverloadMatchesGoalOverload)
{
  const auto goal = MakeTwoJointGoal();
  const auto from_goal = Remap(goal);
  const auto from_msg = RemapMsg(goal.trajectory);
  ASSERT_TRUE(from_goal);
  ASSERT_TRUE(from_msg);
  EXPECT_EQ(from_msg->num_points, from_goal->num_points);
  ASSERT_EQ(from_msg->positions.size(), from_goal->positions.size());
  for (std::size_t index = 0; index < from_goal->positions.size(); ++index)
  {
    EXPECT_DOUBLE_EQ(from_msg->positions[index], from_goal->positions[index]);
    EXPECT_DOUBLE_EQ(from_msg->velocities[index], from_goal->velocities[index]);
  }
}

// Producer silence synthesizes a decel-to-rest stop-tail: seeded with acceleration 0
// (the burst-runaway fix), finalized so its trailing chunk closes the move, ending the
// online session.
TEST_F(PassthroughTrajectoryControllerTest, OnlineProducerSilenceSynthesizesDecelStopTail)
{
  SetProducerTimeout(0.003);  // a few hold cycles at kSamplePeriod
  PrepareOnline({0.0, 0.0});
  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.3, 5));  // committed velocity 0.3 on joint0
  Update(); AckCurrentCommand();  // BEGIN
  Update(); AckCurrentCommand();  // stream chunk (5 points) -> committed vel = 0.3
  ASSERT_TRUE(OnlineActive());
  ASSERT_FALSE(OnlineStopping());

  for (int attempt = 0; attempt < 100 && !OnlineStopping(); ++attempt)
  {
    Update();
    AckCurrentCommand();
  }
  ASSERT_TRUE(OnlineStopping()) << "producer silence must synthesize a stop-tail";
  EXPECT_NEAR(StopTailAccel(0, 0), 0.0, 1.0e-9) << "stop-tail seeds acceleration 0 (joint0)";
  EXPECT_NEAR(StopTailAccel(0, 1), 0.0, 1.0e-9) << "stop-tail seeds acceleration 0 (joint1)";

  bool saw_final_chunk = false;
  for (int attempt = 0; attempt < 5000 && OnlineActive(); ++attempt)
  {
    Update();
    if (CurrentCommandToken() == static_cast<int>(proto::CommandToken::AppendChunk) &&
        CurrentChunkFinal() == 1)
    {
      saw_final_chunk = true;
    }
    AckCurrentCommand();
  }
  EXPECT_TRUE(saw_final_chunk) << "the stop-tail's trailing chunk finalizes the move";
  EXPECT_FALSE(OnlineActive()) << "the session ends once the tail closes the move";
  EXPECT_FALSE(OnlineBusyFlag());
}

// An empty JointTrajectory is the JTC's soft stop. The request starts the
// same decel-to-rest stop-tail as producer silence, on the very next cycle rather than
// after online.producer_timeout, and the tail closes the move as usual.
TEST_F(PassthroughTrajectoryControllerTest, OnlineEmptyTrajectoryStopsStream)
{
  SetProducerTimeout(10.0);  // silence alone must not be what stops the stream here
  PrepareOnline({0.0, 0.0});
  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.3, 5));
  Update(); AckCurrentCommand();  // BEGIN
  Update(); AckCurrentCommand();  // stream chunk -> committed vel = 0.3
  ASSERT_TRUE(OnlineActive());
  ASSERT_FALSE(OnlineStopping());

  RequestOnlineStop();  // the callback's action on an empty message
  Update();
  EXPECT_FALSE(OnlineStopRequested()) << "the request is consumed on the next cycle";
  ASSERT_TRUE(OnlineStopping()) << "the stop-tail starts on the next cycle";
  EXPECT_NEAR(StopTailAccel(0, 0), 0.0, 1.0e-9);
  AckCurrentCommand();

  bool saw_final_chunk = false;
  for (int attempt = 0; attempt < 5000 && OnlineActive(); ++attempt)
  {
    Update();
    if (CurrentCommandToken() == static_cast<int>(proto::CommandToken::AppendChunk) &&
        CurrentChunkFinal() == 1)
    {
      saw_final_chunk = true;
    }
    AckCurrentCommand();
  }
  EXPECT_TRUE(saw_final_chunk) << "the stop-tail's trailing chunk finalizes the move";
  EXPECT_FALSE(OnlineActive());
  EXPECT_FALSE(OnlineBusyFlag());
}

// The empty message arrived after any window still staged, so that window is superseded:
// it must not open (or splice into) a stream the client has just asked to stop. And a
// stop request with no stream open is consumed, not held against the next stream.
TEST_F(PassthroughTrajectoryControllerTest, OnlineEmptyTrajectorySupersedesStagedWindow)
{
  PrepareOnline({0.0, 0.0});
  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.3, 5));  // staged, not yet adopted
  RequestOnlineStop();                                    // then the empty message
  Update();
  EXPECT_FALSE(OnlineActive()) << "the superseded window does not open a stream";
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::None));
  EXPECT_FALSE(OnlineStopRequested());
  Update();
  EXPECT_FALSE(OnlineActive()) << "and it does not ghost-open on a later cycle";

  // A fresh window after the (moot) stop opens normally: the request was not held.
  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.3, 5));
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Begin));
  EXPECT_TRUE(OnlineActive());
  EXPECT_FALSE(OnlineStopping());
}

// After a stop-tail ends the session, a fresh snapshot reopens a new open move (a new
// BEGIN with size -1) -- the jog-stop -> jog-restart cycle.
TEST_F(PassthroughTrajectoryControllerTest, OnlineStopThenRestartReopens)
{
  SetProducerTimeout(0.003);
  PrepareOnline({0.0, 0.0});
  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.3, 5));
  for (int attempt = 0; attempt < 5000; ++attempt)
  {
    Update();
    AckCurrentCommand();
    if (!OnlineActive() && attempt > 10) { break; }
  }
  ASSERT_FALSE(OnlineActive());

  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.3, 5));
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Begin));
  EXPECT_EQ(TrajectorySizeCommanded(), -1);
  EXPECT_TRUE(OnlineActive());
}

// --- bounded lookahead (phase 2: the online feed gate) ------------------------
// The gate keeps the committed depth (points the hardware accepted but the robot has
// not executed) inside the [low_water, high_water] hysteresis band; the fixture band
// is 30/80 points (see SetUp).

// Depth at/above the high-water mark -> HOLD; a mid-band depth keeps holding (the
// hysteresis: the gate does not flip at a single edge); only draining below the
// low-water mark resumes feeding. The move's opening chunk goes out first (it bypasses
// the gate -- see OnlineOpeningChunkBypassesGate); the band governs from chunk two.
TEST_F(PassthroughTrajectoryControllerTest, OnlineFeedGateHoldsUntilLowWater)
{
  PrepareOnline({0.0, 0.0});
  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.1, 200));
  Update(); AckCurrentCommand();  // BEGIN
  Update(); AckCurrentCommand();  // opening chunk (gate bypass)
  const int opened_cursor = OnlineNextIndex();
  ASSERT_GT(opened_cursor, 0);

  SetCommittedDepth(80);  // at the high water: stop refilling
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::None))
    << "at/above the high-water mark the online branch must hold";
  EXPECT_EQ(OnlineNextIndex(), opened_cursor) << "nothing fed while holding";
  EXPECT_TRUE(OnlineActive());

  SetCommittedDepth(50);  // mid-band: still holding (hysteresis)
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::None))
    << "a mid-band depth must not resume feeding";

  SetCommittedDepth(10);  // below the low water: refill
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::AppendChunk));
  EXPECT_EQ(CurrentChunkLen(), 64) << "horizon space 70 leaves the full 64-point chunk";
}

// While refilling, each post-opening chunk is capped to the horizon space (high water -
// depth) so a single feed can never overshoot the band, and a mid-band depth KEEPS
// refilling until the high water is reached (the other half of the hysteresis).
TEST_F(PassthroughTrajectoryControllerTest, OnlineChunkLengthCappedByHorizonSpace)
{
  PrepareOnline({0.0, 0.0});
  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.1, 200));
  Update(); AckCurrentCommand();  // BEGIN
  Update(); AckCurrentCommand();  // opening chunk (gate bypass, full 64)
  const int opened_cursor = OnlineNextIndex();
  ASSERT_EQ(opened_cursor, 64);

  SetCommittedDepth(25);  // below the low water -> refilling; horizon space = 80-25 = 55
  Update(); AckCurrentCommand();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::AppendChunk));
  EXPECT_EQ(CurrentChunkLen(), 55)
    << "chunk must be capped to high_water - depth, not chunk_size";
  EXPECT_EQ(OnlineNextIndex(), opened_cursor + 55);

  SetCommittedDepth(50);  // mid-band while refilling: keep feeding up to the high water
  Update(); AckCurrentCommand();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::AppendChunk));
  EXPECT_EQ(CurrentChunkLen(), 30) << "refill continues mid-band, capped to 80-50";

  SetCommittedDepth(80);  // reached the high water: hold again
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::None));
  EXPECT_EQ(OnlineNextIndex(), opened_cursor + 85) << "cursor unchanged while holding";
}

// The move's OPENING chunk bypasses the horizon cap: the firmware arms its
// out-of-frames watchdog the instant a move opens, and right after a reopen the
// reported depth still counts the previous move's draining frames -- capping the
// opening chunk by horizon space would open the new move with a handful of points and
// starve it at birth (the live teleop stop/start fault).
TEST_F(PassthroughTrajectoryControllerTest, OnlineOpeningChunkBypassesGate)
{
  PrepareOnline({0.0, 0.0});
  SetCommittedDepth(80);  // previous move still draining: depth at the high water
  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.1, 5));
  Update();  // adopt + BEGIN
  ASSERT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Begin));
  AckCurrentCommand();
  Update();  // opening chunk despite the saturated depth
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::AppendChunk));
  EXPECT_EQ(CurrentChunkLen(), 64)
    << "the opening chunk must beat the firmware watchdog, not the gate";
  AckCurrentCommand();
  Update();  // from chunk two the gate reasserts
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::None))
    << "post-opening chunks are gated again";
}

// A snapshot that arrives DURING the stop-tail predates the stop: after the session
// closes it must NOT be adopted as a ghost reopen (its positions are stale by the
// whole stop duration). A genuinely new snapshot still reopens.
TEST_F(PassthroughTrajectoryControllerTest, OnlineStaleSnapshotDoesNotGhostReopen)
{
  SetProducerTimeout(0.003);
  PrepareOnline({0.0, 0.0});
  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.3, 5));
  Update(); AckCurrentCommand();  // BEGIN
  Update(); AckCurrentCommand();  // opening chunk
  for (int attempt = 0; attempt < 100 && !OnlineStopping(); ++attempt)
  {
    Update();
    AckCurrentCommand();
  }
  ASSERT_TRUE(OnlineStopping());
  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.3, 5));  // arrives mid-stop: stale
  for (int attempt = 0; attempt < 5000 && OnlineActive(); ++attempt)
  {
    Update();
    AckCurrentCommand();
  }
  ASSERT_FALSE(OnlineActive());

  Update();  // the stale window must not reopen the move
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::None))
    << "a snapshot staged before the stop must not spawn a ghost session";
  EXPECT_FALSE(OnlineActive());

  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.3, 5));  // fresh window: reopens
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Begin));
  EXPECT_TRUE(OnlineActive());
}

// The gate throttles ONLY the online (jog) stream: the action path keeps its feed-all
// behavior no matter what depth the hardware reports (finite goals want the buffer).
TEST_F(PassthroughTrajectoryControllerTest, ActionFeedIgnoresCommittedDepth)
{
  InjectPendingTrajectory(TinyTraj(0.5, 0.25, 5));
  SetCommittedDepth(1000);  // far above the band: would gate off any online feed
  Update();  // adopt + BEGIN
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Begin));
  AckCurrentCommand();
  Update();  // full chunk, ungated
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::AppendChunk));
  EXPECT_EQ(CurrentChunkLen(), 5) << "action chunks are not depth-capped";
}

// The stop-tail is paced by the gate too (it appends after the committed frames, so
// feeding it into a full buffer would overshoot the bound) -- but it still finalizes
// and closes the session once the depth drains.
TEST_F(PassthroughTrajectoryControllerTest, OnlineStopTailPacedByFeedGate)
{
  SetProducerTimeout(0.003);
  PrepareOnline({0.0, 0.0});
  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.3, 5));
  Update(); AckCurrentCommand();  // BEGIN
  Update(); AckCurrentCommand();  // stream chunk -> committed vel = 0.3

  SetCommittedDepth(100);  // above the high water while the producer goes silent
  for (int attempt = 0; attempt < 100 && !OnlineStopping(); ++attempt)
  {
    Update();
    AckCurrentCommand();
  }
  ASSERT_TRUE(OnlineStopping()) << "silence must still synthesize the stop-tail";
  Update();
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::None))
    << "the staged stop-tail holds while the depth is above the band";
  EXPECT_TRUE(OnlineActive());

  SetCommittedDepth(0);  // drained: the tail may now feed and finalize
  bool saw_final_chunk = false;
  for (int attempt = 0; attempt < 5000 && OnlineActive(); ++attempt)
  {
    Update();
    if (CurrentCommandToken() == static_cast<int>(proto::CommandToken::AppendChunk) &&
        CurrentChunkFinal() == 1)
    {
      saw_final_chunk = true;
    }
    AckCurrentCommand();
  }
  EXPECT_TRUE(saw_final_chunk) << "the paced stop-tail still finalizes the move";
  EXPECT_FALSE(OnlineActive());
}

// --- velocity-intent splice (phase 3) -----------------------------------------

// A Servo-style single-point window whose position is wildly off the committed seam
// (it anchors to the producer's stale measured view) must be ABSORBED, not dropped:
// no new BEGIN, the stream re-anchors at the committed (P,V) and its position target
// integrates from there -- the window's absolute position is ignored.
TEST_F(PassthroughTrajectoryControllerTest, OnlineSpliceAbsorbsOffPositionWindow)
{
  PrepareOnline({0.0, 0.0});
  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.2, 5));
  Update(); AckCurrentCommand();  // BEGIN
  Update(); AckCurrentCommand();  // opening chunk -> committed = 64 samples into the blend
  const double seam_pos = CommittedPos(0);
  const double seam_vel = CommittedVel(0);
  ASSERT_GT(seam_vel, 0.0) << "the blend toward 0.2 rad/s has begun";

  // Single point, 10 rad away -- the old continuity-gated replace dropped this.
  StageOnlineSnapshot(MovingWindow(10.0, 0.0, 0.4, 1));
  Update();
  EXPECT_TRUE(OnlineActive());
  EXPECT_NE(CurrentCommandToken(), static_cast<int>(proto::CommandToken::Begin))
    << "a splice never re-opens the move";
  EXPECT_EQ(CurrentCommandToken(), static_cast<int>(proto::CommandToken::AppendChunk))
    << "the re-anchored blend starts feeding the same cycle";
  // Seam continuity: the blend's first sample continues from the committed (P,V),
  // nowhere near the window's absolute 10 rad position.
  EXPECT_NEAR(OnlinePos(0, 0), seam_pos, 0.01) << "blend anchors at the committed seam";
  EXPECT_NEAR(OnlineVel(0, 0), seam_vel, 0.02) << "no velocity step at the seam";
}

// The splice blends to the window's velocity intent (its last point's velocities) and
// appends a constant-velocity extension that outlives the producer-silence timeout, so
// a short window can never starve the firmware before the stop-tail fires.
TEST_F(PassthroughTrajectoryControllerTest, OnlineSpliceBlendsToVelocityIntentWithExtension)
{
  PrepareOnline({0.0, 0.0});
  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.2, 5));
  Update(); AckCurrentCommand();  // BEGIN
  Update(); AckCurrentCommand();  // opening chunk -> committed = mid-blend toward 0.2
  const double seam_vel = CommittedVel(0);

  StageOnlineSnapshot(MovingWindow(5.0, 0.0, 0.4, 1));  // intent: 0.4 rad/s
  Update();
  const auto & spliced = OnlineTraj();
  const std::size_t last = spliced.num_points - 1;
  EXPECT_NEAR(OnlineVel(last, 0), 0.4, 1.0e-9) << "tail runs at the velocity intent";
  EXPECT_GE(OnlineSpanSeconds(), 0.05)
    << "the staged tail must outlive online.producer_timeout";
  // The whole spliced profile stays inside the blend's velocity envelope: no dip
  // toward the window's (backward) absolute position, no overshoot past the intent.
  for (std::size_t point = 0; point < spliced.num_points; ++point)
  {
    EXPECT_GE(OnlineVel(point, 0), seam_vel - 1.0e-6) << "velocity dip at point " << point;
    EXPECT_LE(OnlineVel(point, 0), 0.4 + 1.0e-6) << "velocity overshoot at point " << point;
  }
}

// The velocity intent is clamped to online.max_velocity (fixture cap: 2.0 rad/s), so a
// runaway producer cannot command an over-speed tail.
TEST_F(PassthroughTrajectoryControllerTest, OnlineSpliceClampsVelocityIntent)
{
  PrepareOnline({0.0, 0.0});
  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.2, 5));
  Update(); AckCurrentCommand();  // BEGIN
  Update(); AckCurrentCommand();  // chunk

  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 5.0, 1));  // 5.0 rad/s > 2.0 cap
  Update();
  const std::size_t last = OnlineTraj().num_points - 1;
  EXPECT_NEAR(OnlineVel(last, 0), 2.0, 1.0e-9) << "intent clamped to online.max_velocity";
}

// Producer silence after a splice still synthesizes the decel stop-tail from the
// committed state of the spliced stream and closes the move -- the jog lifecycle
// (splice -> silence -> stop -> reopen) stays intact.
TEST_F(PassthroughTrajectoryControllerTest, OnlineSpliceThenSilenceStopsCleanly)
{
  SetProducerTimeout(0.003);
  PrepareOnline({0.0, 0.0});
  StageOnlineSnapshot(MovingWindow(0.0, 0.0, 0.2, 5));
  Update(); AckCurrentCommand();  // BEGIN
  Update(); AckCurrentCommand();  // chunk
  StageOnlineSnapshot(MovingWindow(3.0, 0.0, 0.4, 1));  // splice to 0.4 rad/s
  Update(); AckCurrentCommand();

  bool saw_final_chunk = false;
  for (int attempt = 0; attempt < 5000 && OnlineActive(); ++attempt)
  {
    Update();
    if (CurrentCommandToken() == static_cast<int>(proto::CommandToken::AppendChunk) &&
        CurrentChunkFinal() == 1)
    {
      saw_final_chunk = true;
    }
    AckCurrentCommand();
  }
  EXPECT_TRUE(saw_final_chunk) << "the stop-tail finalizes the spliced stream";
  EXPECT_FALSE(OnlineActive());
  EXPECT_FALSE(OnlineBusyFlag());
}

}  // namespace rapidcode_passthrough_trajectory_controller

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

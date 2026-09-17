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

// Unit tests for the RapidCodeSystemHardware mailbox decode + payload transfer.
//
// SCOPE: verify that controller commands written to the trajectory_transfer
// command interfaces land in the correct `xfer*` storage (slot-major / joint-minor
// layout) and that write()'s BEGIN / AppendChunk decode populates the transfer
// state fields (size, valid_fields, ack_sequence, accepted_point_index, error_code)
// exactly as the wire contract specifies.
//
// This deliberately does NOT exercise RapidCode motion: on_configure/on_activate
// are never called, so multiAxis == nullptr and the guarded MovePVT() call in
// AppendTrajectory() is skipped. The hardware is driven purely through its exported
// ros2_control interfaces -- no rmp, no EtherCAT, no firmware. Fast and hermetic.

#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/handle.hpp"

#include "rapidcode_system/rapidcode_system_hardware.hpp"
#include "rapidcode_trajectory_transfer/protocol.hpp"

namespace proto = rapidcode_trajectory_transfer;

// Defined in namespace rapidcode_system so the `friend class
// RapidCodeSystemHardwareTest;` declaration in the hardware header (which resolves
// in that namespace) actually grants this fixture access to the private xfer
// storage. TEST_F bodies call the public getters below.
namespace rapidcode_system
{

// Test double: overrides the backpressure/drain seams so the queue can be driven
// without a live rmp. freeFrames/remainingFrames default to 0 -> the FIFO never feeds
// (identical to the no-live-axes production path), so tests that don't script them
// behave exactly as before. FeedChunk is deliberately NOT overridden: the base runs
// (its firmware call is null-guarded), so the firstChunk / IsFinal state machine is
// still exercised.
class DrivableHardware : public RapidCodeSystemHardware
{
public:
  int32_t freeFrames = 0;       // scripted free space returned by FreeFramesAvailable()
  int32_t remainingFrames = 0;  // scripted firmware-buffer occupancy for FramesRemaining()
protected:
  int32_t FreeFramesAvailable() const override { return freeFrames; }
  int32_t FramesRemaining() const override { return remainingFrames; }
};

// Fixture is a friend of RapidCodeSystemHardware (see the header), so its methods
// can read the private xfer storage. TEST_F bodies call the public getters below.
class RapidCodeSystemHardwareTest : public ::testing::Test
{
protected:
  static constexpr std::size_t kNumJoints = 2;
  const std::string gpio_ = proto::DefaultGpioName;

  DrivableHardware hw_;
  std::vector<hardware_interface::CommandInterface> cmd_;
  std::vector<hardware_interface::StateInterface> state_;
  std::map<std::string, std::size_t> cmd_idx_;
  std::map<std::string, std::size_t> state_idx_;

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

    cmd_ = hw_.export_command_interfaces();
    state_ = hw_.export_state_interfaces();
    for (std::size_t idx = 0; idx < cmd_.size(); ++idx) { cmd_idx_[cmd_[idx].get_name()] = idx; }
    for (std::size_t idx = 0; idx < state_.size(); ++idx) { state_idx_[state_[idx].get_name()] = idx; }
  }

  // --- drive / observe via the exported ros2_control interfaces -----------------
  void SetCmd(const std::string & suffix, double value)
  {
    const std::string full = proto::FullName(gpio_, suffix);
    auto iter = cmd_idx_.find(full);
    ASSERT_NE(iter, cmd_idx_.end()) << "no command interface named '" << full << "'";
    (void)cmd_[iter->second].set_value(value);
  }

  double GetState(const std::string & suffix)
  {
    const std::string full = proto::FullName(gpio_, suffix);
    auto iter = state_idx_.find(full);
    EXPECT_NE(iter, state_idx_.end()) << "no state interface named '" << full << "'";
    if (iter == state_idx_.end()) { return std::numeric_limits<double>::quiet_NaN(); }
    return state_[iter->second].get_optional().value();
  }

  void Write() { (void)hw_.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.002)); }
  void Read() { (void)hw_.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.002)); }

  // --- friend-access getters into the private xfer storage ----------------------
  int traj_size() const { return hw_.transferTrajectorySize; }
  int valid_fields() const { return hw_.transferValidFields; }
  bool move_open() const { return hw_.transferMoveOpen; }
  bool first_chunk() const { return hw_.firstChunk; }
  std::size_t queue_depth() const { return hw_.excessChunks.size(); }
  // Friendship is not inherited by the TEST_F-generated subclasses, so private xfer
  // storage must be poked through a fixture method like this one, not from a test body.
  void set_latched_error(int code) { hw_.xferErrorCode = proto::Encode(code); }
  std::uint64_t completed_id()
  {
    return proto::Decode<std::uint64_t>(GetState(proto::interface_names::CompletedTrajectoryId));
  }
  std::uint64_t ack_seq_member() const { return hw_.ackSequence; }
  double slot_duration(std::size_t slot) const { return hw_.xferSlotDuration[slot]; }
  double slot_pos(std::size_t joint, std::size_t slot) const
  {
    return hw_.xferSlotPosition[slot * kNumJoints + joint];
  }
  double slot_vel(std::size_t joint, std::size_t slot) const
  {
    return hw_.xferSlotVelocity[slot * kNumJoints + joint];
  }
  double slot_acc(std::size_t joint, std::size_t slot) const
  {
    return hw_.xferSlotAcceleration[slot * kNumJoints + joint];
  }
  double slot_jerk(std::size_t joint, std::size_t slot) const
  {
    return hw_.xferSlotJerk[slot * kNumJoints + joint];
  }
};

// The export must hand out exactly the scalar mailbox + K slots * (duration + per
// joint pos/vel/acc/jerk), and the four scalar state fields + per-joint pos/vel.
TEST_F(RapidCodeSystemHardwareTest, ExportsExpectedInterfaceCounts)
{
  const std::size_t chunk_capacity = static_cast<std::size_t>(proto::ChunkCapacity);
  EXPECT_EQ(cmd_.size(), 8u + chunk_capacity * (1u + kNumJoints * 4u));
  // 6 scalar state fields (is_moving, ack_sequence, accepted_point_index, error_code,
  // completed_trajectory_id, committed_depth_points) + per-joint pos/vel.
  EXPECT_EQ(state_.size(), 6u + kNumJoints * 2u);

  // a couple of names must resolve (sanity that FullName/leaf helpers agree)
  EXPECT_NE(cmd_idx_.find(proto::FullName(gpio_, proto::interface_names::CommandToken)),
    cmd_idx_.end());
  EXPECT_NE(cmd_idx_.find(proto::FullName(gpio_, proto::JointSlotJerk(1, 63))), cmd_idx_.end());
  EXPECT_NE(state_idx_.find(proto::FullName(gpio_, proto::interface_names::AckSequence)),
    state_idx_.end());
  EXPECT_NE(state_idx_.find(proto::FullName(gpio_, proto::interface_names::CompletedTrajectoryId)),
    state_idx_.end());
}

// Each "j{joint}_s{slot}_*" command interface must alias xferSlot*[slot*nj + joint]
// (slot-major / joint-minor). Probe the corners of the grid with unique sentinels.
TEST_F(RapidCodeSystemHardwareTest, SlotPayloadIsSlotMajorJointMinor)
{
  struct Cell { std::size_t joint, slot; };
  const std::vector<Cell> cells = {
    {0, 0}, {1, 0}, {0, 1}, {1, 1}, {0, 63}, {1, 63}};

  // pos/vel/acc/jerk are per (joint, slot); duration is per slot only. Use sentinels
  // that encode their indices so a transposed/aliased store is caught.
  const auto dur_val = [](std::size_t slot) { return 100000.0 + static_cast<double>(slot); };
  for (const auto & cell : cells)
  {
    const double base = 1000.0 * static_cast<double>(cell.slot) + 10.0 * static_cast<double>(cell.joint);
    SetCmd(proto::JointSlotPosition(cell.joint, cell.slot), base + 1.0);
    SetCmd(proto::JointSlotVelocity(cell.joint, cell.slot), base + 2.0);
    SetCmd(proto::JointSlotAcceleration(cell.joint, cell.slot), base + 3.0);
    SetCmd(proto::JointSlotJerk(cell.joint, cell.slot), base + 4.0);
    SetCmd(proto::SlotDuration(cell.slot), dur_val(cell.slot));  // per slot (joint-independent)
  }

  for (const auto & cell : cells)
  {
    const double base = 1000.0 * static_cast<double>(cell.slot) + 10.0 * static_cast<double>(cell.joint);
    EXPECT_DOUBLE_EQ(slot_pos(cell.joint, cell.slot), base + 1.0) << "pos j" << cell.joint << " s" << cell.slot;
    EXPECT_DOUBLE_EQ(slot_vel(cell.joint, cell.slot), base + 2.0) << "vel j" << cell.joint << " s" << cell.slot;
    EXPECT_DOUBLE_EQ(slot_acc(cell.joint, cell.slot), base + 3.0) << "acc j" << cell.joint << " s" << cell.slot;
    EXPECT_DOUBLE_EQ(slot_jerk(cell.joint, cell.slot), base + 4.0) << "jerk j" << cell.joint << " s" << cell.slot;
    EXPECT_DOUBLE_EQ(slot_duration(cell.slot), dur_val(cell.slot)) << "dur s" << cell.slot;
  }
}

// BEGIN must latch trajectory_size + valid_fields, open the move, reset
// accepted_point_index to -1, and ack the command sequence. error_code reads 0 here
// because nothing set it -- BEGIN no longer clears it (see BeginPreservesLatchedError).
TEST_F(RapidCodeSystemHardwareTest, BeginDecodesSizeValidFieldsAndAcks)
{
  SetCmd(proto::interface_names::TrajectorySize, proto::Encode(5));
  SetCmd(proto::interface_names::ValidFields,
    proto::Encode(proto::FieldMaskPosition | proto::FieldMaskVelocity |
      proto::FieldMaskAcceleration | proto::FieldMaskJerk));  // 0xF
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::Begin));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(1));
  Write();

  EXPECT_EQ(traj_size(), 5);
  EXPECT_EQ(valid_fields(), 0xF);
  EXPECT_TRUE(move_open());
  EXPECT_EQ(proto::Decode<std::uint64_t>(GetState(proto::interface_names::AckSequence)), 1u);
  EXPECT_EQ(proto::Decode<int>(GetState(proto::interface_names::AcceptedPointIndex)), -1);
  EXPECT_EQ(proto::Decode<int>(GetState(proto::interface_names::ErrorCode)), 0);
}

// AppendChunk must advance accepted_point_index to (base + len - 1) and ack.
TEST_F(RapidCodeSystemHardwareTest, AppendChunkSetsAcceptedIndexAndAcks)
{
  // open a transfer first
  SetCmd(proto::interface_names::TrajectorySize, proto::Encode(10));
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::Begin));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(1));
  Write();

  SetCmd(proto::interface_names::ChunkBaseIndex, proto::Encode(0));
  SetCmd(proto::interface_names::ChunkLen, proto::Encode(3));
  SetCmd(proto::interface_names::ChunkFinal, proto::Encode(0));
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::AppendChunk));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(2));
  Write();

  EXPECT_EQ(proto::Decode<std::uint64_t>(GetState(proto::interface_names::AckSequence)), 2u);
  EXPECT_EQ(proto::Decode<int>(GetState(proto::interface_names::AcceptedPointIndex)), 2);  // 0+3-1
  EXPECT_EQ(proto::Decode<int>(GetState(proto::interface_names::ErrorCode)), 0);

  // a second chunk at base 3, len 4 -> accepted 6
  SetCmd(proto::interface_names::ChunkBaseIndex, proto::Encode(3));
  SetCmd(proto::interface_names::ChunkLen, proto::Encode(4));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(3));
  Write();
  EXPECT_EQ(proto::Decode<int>(GetState(proto::interface_names::AcceptedPointIndex)), 6);  // 3+4-1
}

// The single-slot mailbox is gated on command_sequence: re-issuing a sequence the
// hardware already acked must be a no-op (no re-decode, no accepted-index change).
TEST_F(RapidCodeSystemHardwareTest, StaleSequenceIsIgnored)
{
  SetCmd(proto::interface_names::TrajectorySize, proto::Encode(10));
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::Begin));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(1));
  Write();

  SetCmd(proto::interface_names::ChunkBaseIndex, proto::Encode(0));
  SetCmd(proto::interface_names::ChunkLen, proto::Encode(3));
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::AppendChunk));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(2));
  Write();
  ASSERT_EQ(proto::Decode<int>(GetState(proto::interface_names::AcceptedPointIndex)), 2);

  // change the chunk fields but DON'T bump the sequence -> must be ignored
  SetCmd(proto::interface_names::ChunkBaseIndex, proto::Encode(50));
  SetCmd(proto::interface_names::ChunkLen, proto::Encode(9));
  Write();
  EXPECT_EQ(proto::Decode<std::uint64_t>(GetState(proto::interface_names::AckSequence)), 2u);
  EXPECT_EQ(proto::Decode<int>(GetState(proto::interface_names::AcceptedPointIndex)), 2)
    << "stale (un-bumped) sequence must not re-decode the chunk";
}

// chunk_len is clamped to [0, ChunkCapacity] before it drives accepted_point_index.
TEST_F(RapidCodeSystemHardwareTest, AppendChunkClampsLenToCapacity)
{
  SetCmd(proto::interface_names::TrajectorySize, proto::Encode(10000));
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::Begin));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(1));
  Write();

  SetCmd(proto::interface_names::ChunkBaseIndex, proto::Encode(0));
  SetCmd(proto::interface_names::ChunkLen, proto::Encode(999));  // > ChunkCapacity
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::AppendChunk));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(2));
  Write();

  EXPECT_EQ(proto::Decode<int>(GetState(proto::interface_names::AcceptedPointIndex)),
    proto::ChunkCapacity - 1);  // 0 + clamp(999)=64 - 1
}

// Multi-trajectory: BEGIN must NOT clear a latched error_code. Recovery is only via an
// explicit Reset, so a fault raised before BEGIN survives it (the controller stays in
// its faulted state instead of a queued BEGIN silently unlatching the error).
TEST_F(RapidCodeSystemHardwareTest, BeginPreservesLatchedError)
{
  set_latched_error(3);  // simulate a latched fault (via fixture; see set_latched_error)

  SetCmd(proto::interface_names::TrajectorySize, proto::Encode(5));
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::Begin));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(1));
  Write();

  EXPECT_EQ(proto::Decode<int>(GetState(proto::interface_names::ErrorCode)), 3)
    << "BEGIN must not clear a latched error_code -- only Reset may";

  // ...and Reset clears it.
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::Reset));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(2));
  Write();
  EXPECT_EQ(proto::Decode<int>(GetState(proto::interface_names::ErrorCode)), 0);
}

// Multi-trajectory: a BEGIN arriving while a previous trajectory's chunks are still
// queued must NOT wipe the FIFO (pipelined goals). freeFrames stays 0 so nothing feeds
// and the queued chunk is observable.
TEST_F(RapidCodeSystemHardwareTest, BeginDoesNotClearQueuedChunks)
{
  SetCmd(proto::interface_names::TrajectorySize, proto::Encode(10));
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::Begin));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(1));
  Write();

  SetCmd(proto::interface_names::ChunkBaseIndex, proto::Encode(0));
  SetCmd(proto::interface_names::ChunkLen, proto::Encode(3));
  SetCmd(proto::interface_names::ChunkFinal, proto::Encode(0));
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::AppendChunk));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(2));
  Write();
  ASSERT_EQ(queue_depth(), 1u) << "chunk should be queued (freeFrames=0, nothing feeds)";

  // A second BEGIN must leave the queued chunk in place.
  SetCmd(proto::interface_names::TrajectorySize, proto::Encode(5));
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::Begin));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(3));
  Write();
  EXPECT_EQ(queue_depth(), 1u) << "BEGIN must not clear the queued tail of a prior trajectory";
}

// Multi-trajectory: firstChunk (the reopen latch) starts true, drops to false once the
// opening chunk is fed, and re-arms to true when an IsFinal chunk is fed (that chunk
// closes the firmware move, so the next feed must reopen it). freeFrames is opened up so
// the FIFO actually feeds.
TEST_F(RapidCodeSystemHardwareTest, FirstChunkReArmsAfterFinal)
{
  hw_.freeFrames = 100000;  // let DrainQueue feed everything immediately

  SetCmd(proto::interface_names::TrajectorySize, proto::Encode(4));
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::Begin));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(1));
  Write();
  EXPECT_TRUE(first_chunk()) << "BEGIN must not touch the reopen latch; it starts true";

  // A non-final chunk opens the move -> latch drops.
  SetCmd(proto::interface_names::ChunkBaseIndex, proto::Encode(0));
  SetCmd(proto::interface_names::ChunkLen, proto::Encode(2));
  SetCmd(proto::interface_names::ChunkFinal, proto::Encode(0));
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::AppendChunk));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(2));
  Write();
  ASSERT_EQ(queue_depth(), 0u) << "chunk should have been fed";
  EXPECT_FALSE(first_chunk()) << "opening chunk fed -> move open -> latch false";

  // A final chunk closes the move -> latch re-arms.
  SetCmd(proto::interface_names::ChunkBaseIndex, proto::Encode(2));
  SetCmd(proto::interface_names::ChunkLen, proto::Encode(2));
  SetCmd(proto::interface_names::ChunkFinal, proto::Encode(1));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(3));
  Write();
  EXPECT_TRUE(first_chunk()) << "final chunk closes the move -> next feed must reopen";
}

// Fault recovery: Reset must clear the latched error_code AND re-arm the firstChunk
// reopen latch -- a fault kills the move mid-open (latch false), and without the
// re-arm the next move's opening chunk would skip the reopen gate / NO_WAIT toggle.
TEST_F(RapidCodeSystemHardwareTest, ResetClearsErrorAndRearmsReopenLatch)
{
  hw_.freeFrames = 100000;  // let the opening chunk feed so the latch drops

  SetCmd(proto::interface_names::TrajectorySize, proto::Encode(-1));  // open (jog) move
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::Begin));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(1));
  Write();
  SetCmd(proto::interface_names::ChunkBaseIndex, proto::Encode(0));
  SetCmd(proto::interface_names::ChunkLen, proto::Encode(2));
  SetCmd(proto::interface_names::ChunkFinal, proto::Encode(0));
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::AppendChunk));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(2));
  Write();
  ASSERT_FALSE(first_chunk()) << "opening chunk fed -> move open mid-stream";

  set_latched_error(3);  // the drive faulted mid-open (via fixture; see set_latched_error)

  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::Reset));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(3));
  Write();
  EXPECT_EQ(proto::Decode<int>(GetState(proto::interface_names::ErrorCode)), 0)
    << "Reset clears the latched error_code";
  EXPECT_TRUE(first_chunk()) << "Reset re-arms the reopen latch for the next move";
  EXPECT_FALSE(move_open());
  EXPECT_EQ(queue_depth(), 0u);
}

// Abort mid-open-move must also re-arm the reopen latch (the documented invariant):
// the aborted move is closed, so the next feed must reopen the firmware move.
TEST_F(RapidCodeSystemHardwareTest, AbortRearmsReopenLatch)
{
  hw_.freeFrames = 100000;

  SetCmd(proto::interface_names::TrajectorySize, proto::Encode(10));
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::Begin));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(1));
  Write();
  SetCmd(proto::interface_names::ChunkBaseIndex, proto::Encode(0));
  SetCmd(proto::interface_names::ChunkLen, proto::Encode(2));
  SetCmd(proto::interface_names::ChunkFinal, proto::Encode(0));
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::AppendChunk));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(2));
  Write();
  ASSERT_FALSE(first_chunk());

  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::Abort));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(3));
  Write();
  EXPECT_TRUE(first_chunk()) << "Abort closes the move -> next feed must reopen";
  EXPECT_FALSE(move_open());
  EXPECT_EQ(queue_depth(), 0u);
}

// A commanded Stop closes the move and clears the queued tail exactly like Abort --
// but it is an operator action, not a fault, so error_code must stay 0 (the controller
// holds its own stop latch; latching here would demand a second acknowledgement).
TEST_F(RapidCodeSystemHardwareTest, StopClosesMoveWithoutLatchingError)
{
  hw_.freeFrames = 100000;

  SetCmd(proto::interface_names::TrajectorySize, proto::Encode(10));
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::Begin));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(1));
  Write();
  SetCmd(proto::interface_names::ChunkBaseIndex, proto::Encode(0));
  SetCmd(proto::interface_names::ChunkLen, proto::Encode(2));
  SetCmd(proto::interface_names::ChunkFinal, proto::Encode(0));
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::AppendChunk));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(2));
  Write();
  ASSERT_FALSE(first_chunk());

  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::Stop));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(3));
  Write();
  EXPECT_TRUE(first_chunk()) << "Stop closes the move -> next feed must reopen";
  EXPECT_FALSE(move_open());
  EXPECT_EQ(queue_depth(), 0u);
  EXPECT_EQ(proto::Decode<int>(GetState(proto::interface_names::ErrorCode)), 0)
    << "a commanded stop is not a fault; error_code stays clear";
  EXPECT_EQ(ack_seq_member(), 3u) << "Stop is acked like any other command";
}

// committed_depth_points must count BOTH stores of accepted-but-unexecuted motion:
// firmware frames still pending (ceil-divided by the 2 frames/point) AND points parked
// in the host FIFO. Missing either would make the controller's online feed gate
// under-count exactly when the buffer backs up.
TEST_F(RapidCodeSystemHardwareTest, CommittedDepthCountsFirmwareAndQueuedPoints)
{
  // Open a transfer and enqueue 3 points with freeFrames=0 -> the chunk parks in the FIFO.
  SetCmd(proto::interface_names::TrajectorySize, proto::Encode(-1));  // open (jog) transfer
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::Begin));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(1));
  Write();
  SetCmd(proto::interface_names::ChunkBaseIndex, proto::Encode(0));
  SetCmd(proto::interface_names::ChunkLen, proto::Encode(3));
  SetCmd(proto::interface_names::ChunkFinal, proto::Encode(0));
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::AppendChunk));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(2));
  Write();
  ASSERT_EQ(queue_depth(), 1u) << "chunk should be parked (freeFrames=0, nothing feeds)";

  hw_.remainingFrames = 5;  // firmware: 5 frames pending -> ceil(5/2) = 3 points
  Read();
  EXPECT_EQ(proto::Decode<int>(GetState(proto::interface_names::CommittedDepthPoints)), 3 + 3)
    << "depth = firmware points (3) + FIFO points (3)";

  // Free the firmware buffer: the parked chunk feeds, leaving only the firmware share.
  hw_.freeFrames = 100000;
  Write();
  ASSERT_EQ(queue_depth(), 0u) << "parked chunk should have fed once space freed";
  hw_.remainingFrames = 6;  // its 3 points (6 frames) now pending in the firmware
  Read();
  EXPECT_EQ(proto::Decode<int>(GetState(proto::interface_names::CommittedDepthPoints)), 3)
    << "depth = firmware points only once the FIFO drains";
}

// Multi-trajectory: completed_trajectory_id advances per trajectory as the firmware
// EXECUTES each one's frames (2 frames/point), driven by the scripted drain. Two
// trajectories (id 1: 3 points = 6 frames; id 2: 2 points = 4 frames) are fed, then the
// buffer drains and the completed id must step 0 -> 1 -> 2.
TEST_F(RapidCodeSystemHardwareTest, CompletedTrajectoryIdAdvances)
{
  hw_.freeFrames = 100000;  // feed both trajectories immediately

  // --- trajectory id 1: 3 points ---
  SetCmd(proto::interface_names::TrajectoryId, proto::Encode<std::uint64_t>(1));
  SetCmd(proto::interface_names::TrajectorySize, proto::Encode(3));
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::Begin));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(1));
  Write();
  SetCmd(proto::interface_names::ChunkBaseIndex, proto::Encode(0));
  SetCmd(proto::interface_names::ChunkLen, proto::Encode(3));
  SetCmd(proto::interface_names::ChunkFinal, proto::Encode(0));  // non-final: flows into id 2
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::AppendChunk));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(2));
  Write();

  // --- trajectory id 2: 2 points, final ---
  SetCmd(proto::interface_names::TrajectoryId, proto::Encode<std::uint64_t>(2));
  SetCmd(proto::interface_names::TrajectorySize, proto::Encode(2));
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::Begin));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(3));
  Write();
  SetCmd(proto::interface_names::ChunkBaseIndex, proto::Encode(0));
  SetCmd(proto::interface_names::ChunkLen, proto::Encode(2));
  SetCmd(proto::interface_names::ChunkFinal, proto::Encode(1));
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::AppendChunk));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(4));
  Write();
  ASSERT_EQ(queue_depth(), 0u) << "both trajectories should have been fed";

  // 10 frames fed total (6 + 4). Drain step-by-step; completion is computed in read().
  hw_.remainingFrames = 10;  // nothing executed yet
  Read();
  EXPECT_EQ(completed_id(), 0u) << "no frames executed -> nothing complete";

  hw_.remainingFrames = 4;   // executed 6 -> all of id 1 (threshold 6) done, id 2 (10) not
  Read();
  EXPECT_EQ(completed_id(), 1u);

  hw_.remainingFrames = 0;   // executed 10 -> id 2 (threshold 10) done
  Read();
  EXPECT_EQ(completed_id(), 2u);
}

// Multi-trajectory: an IsFinal chunk in the MIDDLE of a trajectory (a planned interior
// dwell) must NOT complete the trajectory early. Completion keys off the whole
// trajectory's frame count, never a per-chunk IsFinal / MotionDone edge.
TEST_F(RapidCodeSystemHardwareTest, InteriorFinalDoesNotCompleteEarly)
{
  hw_.freeFrames = 100000;  // feed everything immediately

  // One trajectory (id 1), 6 points = 12 frames, fed as two 3-point chunks; the FIRST
  // (interior) chunk carries IsFinal=1, the second is the real last chunk.
  SetCmd(proto::interface_names::TrajectoryId, proto::Encode<std::uint64_t>(1));
  SetCmd(proto::interface_names::TrajectorySize, proto::Encode(6));
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::Begin));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(1));
  Write();

  SetCmd(proto::interface_names::ChunkBaseIndex, proto::Encode(0));
  SetCmd(proto::interface_names::ChunkLen, proto::Encode(3));
  SetCmd(proto::interface_names::ChunkFinal, proto::Encode(1));  // interior IsFinal (dwell)
  SetCmd(proto::interface_names::CommandToken, proto::Encode(proto::CommandToken::AppendChunk));
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(2));
  Write();

  SetCmd(proto::interface_names::ChunkBaseIndex, proto::Encode(3));
  SetCmd(proto::interface_names::ChunkLen, proto::Encode(3));
  SetCmd(proto::interface_names::ChunkFinal, proto::Encode(1));  // real last chunk
  SetCmd(proto::interface_names::CommandSequence, proto::Encode<std::uint64_t>(3));
  Write();
  ASSERT_EQ(queue_depth(), 0u);

  // Executed only the interior chunk's 6 frames -> the trajectory is NOT done, even though
  // an IsFinal chunk has fully executed.
  hw_.remainingFrames = 6;
  Read();
  EXPECT_EQ(completed_id(), 0u) << "interior IsFinal must not complete the trajectory early";

  // All 12 frames executed -> now complete.
  hw_.remainingFrames = 0;
  Read();
  EXPECT_EQ(completed_id(), 1u);
}

}  // namespace rapidcode_system

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

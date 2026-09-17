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

// Single source of truth for the ros2_control "trajectory_transfer" passthrough
// protocol, shared by the rapidcode_system hardware plugin and any passthrough
// trajectory controller. Both packages depend ONLY on this header, so the wire
// contract (interface names + token/state codes) can never drift between them.
//
// All ros2_control interfaces are `double`. The tokens/enums below are
// integer-valued doubles; read them back with DecodeInt() (rounds to the nearest
// integer) so a float round-trip through an interface handle is exact.

#pragma once
#ifndef RAPIDCODE_TRAJECTORY_TRANSFER__PROTOCOL_HPP
#define RAPIDCODE_TRAJECTORY_TRANSFER__PROTOCOL_HPP

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>

namespace rapidcode_trajectory_transfer
{
// Default <gpio> block name in the URDF ros2_control description. The controller
// exposes this as a parameter (so it stays hardware-agnostic); the hardware
// hard-codes it to match its own URDF block.
inline constexpr const char * DefaultGpioName = "trajectory_transfer";

// command interface "<gpio>/command": controller -> hardware, read in write().
enum class CommandToken : int
{
  None = 0,
  Reset = 1,
  Abort = 2,  // abort the running move
  Begin = 3,  // open a new transfer: clear accumulation, reset the FSM
  AppendChunk = 4, // append a chunk of points (1..K) to the transfer
  Stop = 5, // commanded stop: decelerate the group to rest at the stop rate (not a fault)
};

// // state interface "<gpio>/state": hardware -> controller, written in read().
// enum class TransferState : int
// {
//   Idle = 0,          // no transfer in progress; ready to accept BEGIN
//   Accepting = 1,     // BEGIN seen; buffers cleared, awaiting the first point
//   ReadyForNext = 2,  // hardware consumed up to accepted_index; send the next point
//   Executing = 3,     // motion is live (a frame has been appended and is running)
//   Failed = 4,        // aborted or faulted (e-stop, starvation, bad index, bad point)
// };

enum class ValidField : uint32_t
{
  Position = 1UL << 0,
  Velocity = 1UL << 1,
  Acceleration = 1UL << 2,
  Jerk = 1UL << 3,
};

// valid_fields bitmask ("<gpio>/valid_fields"): which setpoint fields a POINT
// carries (the profile-order ladder). pos-only -> MovePT; +vel -> MovePVT;
// +acc -> MovePVT as well (acceleration is carried for the diagnostic trace, not
// sent to RapidCode). Velocity implies position; acceleration implies both.
inline constexpr int FieldMaskPosition = 1;
inline constexpr int FieldMaskVelocity = 2;
inline constexpr int FieldMaskAcceleration = 4;
inline constexpr int FieldMaskJerk = 8;

// Chunk channel width: the gpio carries up to this many trajectory points per
// transfer cycle. The URDF <gpio> block declares this many slots, and the
// hardware + controller agree on it here. The controller may push fewer per cycle
// (its chunk_size param), and the final chunk is usually short. A single chunk
// holding the whole trajectory (N <= K) is the whole-trajectory single-shot.
inline constexpr int ChunkCapacity = 64;

namespace interface_names
{
  // --- transfer-level scalar command interface leaf names (under "<gpio>/") ------
  inline constexpr const char * CommandToken = "command_token";          // Command token
  inline constexpr const char * CommandSequence = "command_sequence";  // monotonically increasing command counter
  inline constexpr const char * TrajectoryId = "trajectory_id";
  inline constexpr const char * TrajectorySize = "trajectory_size";     // total number of points (0/-1 = open)
  inline constexpr const char * ValidFields = "valid_fields"; // bitmask above (same all points)
  // --- chunk-level scalar command interface leaf names --------------------------
  inline constexpr const char * ChunkBaseIndex = "chunk_base_index";  // global index of slot 0
  inline constexpr const char * ChunkLen = "chunk_len";          // valid slots this chunk (1..K)
  inline constexpr const char * ChunkFinal = "chunk_final";      // 1 if chunk holds point N-1
  
  // --- scalar state interface leaf names ----------------------------------------
  inline constexpr const char * IsMoving = "is_moving";
  inline constexpr const char * AckSequence = "ack_sequence";                 // monotonically increasing ack counter
  inline constexpr const char * AcceptedPointIndex = "accepted_point_index";  // last point index consumed
  inline constexpr const char * ErrorCode = "error_code";
  // Highest trajectory_id the hardware has fully EXECUTED (0 = none yet). Monotonic.
  // Lets the controller finish pipelined goals per-id instead of on the single
  // is_moving edge.
  inline constexpr const char * CompletedTrajectoryId = "completed_trajectory_id";
  // Committed DEPTH: trajectory points the hardware has accepted (acked) but the robot
  // has NOT physically executed yet -- firmware frame-buffer points still pending PLUS
  // points parked in the hardware's host-side FIFO. In POINTS (one sample_period each),
  // not firmware frames, so the controller never needs the frames-per-point factor.
  // Answers "if the producer went silent now, how long would the robot keep moving?".
  // The online (jog) feed gate bounds this against its low/high-water marks; ack-on-
  // enqueue alone would let the controller front-load an unbounded depth.
  inline constexpr const char * CommittedDepthPoints = "committed_depth_points";
}

// --- per-slot + per-joint-per-slot command interface leaf names ---------------
// A chunk carries up to kChunkCapacity slots. Each slot s has a duration
// "s{s}_duration" and, per joint i (hardware axis order, j0 = first
// <joint>), a (pos,vel,acc, jerk) triple "j{i}_s{s}_position" / "_velocity" /
// "_acceleration". The controller maps its own joint list onto j0..jN-1.
inline std::string SlotDuration(std::size_t slot)
{
  return "s" + std::to_string(slot) + "_duration";
}
inline std::string JointSlotPosition(std::size_t joint, std::size_t slot)
{
  return "j" + std::to_string(joint) + "_s" + std::to_string(slot) + "_position";
}
inline std::string JointSlotVelocity(std::size_t joint, std::size_t slot)
{
  return "j" + std::to_string(joint) + "_s" + std::to_string(slot) + "_velocity";
}
inline std::string JointSlotAcceleration(std::size_t joint, std::size_t slot)
{
  return "j" + std::to_string(joint) + "_s" + std::to_string(slot) + "_acceleration";
}
inline std::string JointSlotJerk(std::size_t joint, std::size_t slot)
{
  return "j" + std::to_string(joint) + "_s" + std::to_string(slot) + "_jerk";
}

// Build a full interface name "<gpio>/<suffix>".
inline std::string FullName(const std::string & gpio, const std::string & suffix)
{
  return gpio + "/" + suffix;
}

// --- encode / decode (interfaces are doubles) ---------------------------------
template<typename IntType = int, std::enable_if_t<std::is_integral_v<IntType> || std::is_enum_v<IntType>, bool> = true>
inline double Encode(IntType value) { return static_cast<double>(value); }
template<typename IntType = int, std::enable_if_t<std::is_integral_v<IntType> || std::is_enum_v<IntType>, bool> = true>
inline IntType Decode(double value) 
{
  return static_cast<IntType>(std::llround(value));
}

}  // namespace rapidcode_trajectory_transfer

#endif  // RAPIDCODE_TRAJECTORY_TRANSFER__PROTOCOL_HPP

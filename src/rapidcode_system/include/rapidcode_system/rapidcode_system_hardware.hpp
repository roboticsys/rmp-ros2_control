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

// RapidCode <-> ros2_control SystemInterface (boilerplate).
//
// This is the "Option B / recommended core" from
// ros2-rapidcode-integration-brainstorm.md: a ros2_control hardware plugin that
// makes the RMP MotionController a ros2_control "system". The fixed-rate
// read()/write() loop maps onto RapidCode telemetry getters and MovePVT
// streaming, and the controller_manager process becomes the single rmp owner.
//
// on_init parses the URDF parameters; on_configure creates the rmp
// MotionController and brings up the (phantom or hardware) axes + the MultiAxis
// group; on_activate/on_deactivate arm and abort the chunk stream; read()/write()
// run the trajectory_transfer mailbox + host-FIFO drain.

#pragma once
#ifndef RAPIDCODE_SYSTEM__RAPIDCODE_SYSTEM_HARDWARE_HPP
#define RAPIDCODE_SYSTEM__RAPIDCODE_SYSTEM_HARDWARE_HPP

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rapidcode_trajectory_transfer/protocol.hpp"

#include <rsi.h>  // RapidCode public API: RSI::RapidCode::{MotionController,Axis}

namespace rapidcode_system
{

struct TrajectoryChunk
{
  bool IsFinal = false;
  std::size_t Count = 0;
  // Which trajectory (BEGIN) this chunk belongs to. Chunks inherit the id of the most
  // recent BEGIN; used for per-trajectory completion accounting (see UpdateCompletion).
  std::uint64_t TrajectoryId = 0;
  std::vector<double> Durations;
  std::vector<double> Positions;
  std::vector<double> Velocities;
  std::vector<double> Accelerations;
  std::vector<double> Jerks;

  TrajectoryChunk(bool inputFinal, size_t inputCount, const std::vector<double>& inputDurations,
    const std::vector<double>& inputPositions, const std::vector<double>& inputVelocities,
    const std::vector<double>& inputAccelerations, const std::vector<double>& inputJerks,
    std::uint64_t inputTrajectoryId = 0)
    : IsFinal(inputFinal)
    , Count(inputCount)
    , TrajectoryId(inputTrajectoryId)
    // perform deep-copies of input vectors to ensure data integrity and pointer stability
    , Durations(inputDurations)
    , Positions(inputPositions)
    , Velocities(inputVelocities)
    , Accelerations(inputAccelerations)
    , Jerks(inputJerks)
  {}
};

class RapidCodeSystemHardware : public hardware_interface::SystemInterface
{
  // Grant the unit-test fixture read access to the private `xfer*` mailbox storage
  // so it can verify that controller commands decode into the right fields and the
  // slot-major payload layout is correct. Test-only; no production effect.
  friend class RapidCodeSystemHardwareTest;

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RapidCodeSystemHardware)

  RapidCodeSystemHardware() = default;
  // The lifecycle teardown (on_cleanup/on_shutdown) stops the rmp firmware and
  // frees the controller, but the destructor is the last-resort safety net: if
  // the component is destroyed without that teardown ever running (controller_
  // manager unload, or process exit straight from ACTIVE), still Shutdown +
  // Delete so we never leak the firmware process or its shared memory.
  ~RapidCodeSystemHardware() override;

  // Owns a raw MotionController* (RapidCode hands back raw pointers); forbid
  // copy/move so the single-owner invariant can't be duplicated by accident.
  RapidCodeSystemHardware(const RapidCodeSystemHardware &) = delete;
  RapidCodeSystemHardware & operator=(const RapidCodeSystemHardware &) = delete;
  RapidCodeSystemHardware(RapidCodeSystemHardware &&) = delete;
  RapidCodeSystemHardware & operator=(RapidCodeSystemHardware &&) = delete;

  // --- Lifecycle (LifecycleNodeInterface callbacks) ---------------------------
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  // --- Interface export -------------------------------------------------------
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // --- Real-time-ish read/write loop ------------------------------------------
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // --- on_init blocks ----------------------------------------------------------
  // Size every per-joint and transfer-mailbox buffer to the URDF joint count (all
  // pointers exported later must be stable for the component's lifetime).
  void AllocateJointStorage();
  // Parse the controller-wide URDF <hardware><param> block (mode, NIC, affinity,
  // sample rate, amp-enable delay) + the RAPIDCODE_FRAME_DEBUG env toggle.
  void ReadHardwareParameters();
  // Parse the per-joint URDF params: axis index, user_units, amp_enable_group, and
  // the optional origin / error_limit (presence-tracked so a value of 0 is honored).
  void ReadJointParameters();

  // --- on_configure blocks (each may throw RsiError; on_configure catches) ------
  // Create the rmp MotionController (single owner for this process), set the sample
  // rate, and pre-allocate recorder 0 (must precede any AxisGet, per the SDK).
  void CreateController();
  // Hardware: start EtherCAT and require OPERATIONAL. Phantom: require the network
  // down and size the axis count. Then size the motion-supervisor pool and seed
  // frameBufferSize.
  void SetupNetworkAndAxisCount();
  // Fetch + configure every joint's Axis (reset to IDLE, origin calibration, user
  // units, phantom shaping / error limit) and seed the exported state from it.
  void ConfigureAxes();
  // Bind all axes into the one MultiAxis group on the free supervisor and log the
  // e-stop window the MovePVT emptyCount runway should cover.
  void BindMultiAxisGroup();
  // Configure (but do NOT start) recorder 0: per axis command/actual pos + vel.
  void ConfigureRecorder();
  // on_configure's catch: log, best-effort Delete the half-created controller, and
  // null every RapidCode handle so the component drops cleanly to UNCONFIGURED.
  void CleanupFailedConfigure(const char * what);

  // --- on_activate / read() / write() blocks -----------------------------------
  // Open the opt-in MovePVT input trace (env RAPIDCODE_PVT_CSV=<path>) and
  // write its header. No-op when the env var is unset or the file is already open
  // (re-activation reuses the handle). Fully buffered so per-cycle rows are cheap.
  void OpenPvtTrace();
  // Refresh the exported joint position/velocity state from the axes: hardware
  // reads the encoder (Actual*), a phantom reports its integrated command.
  void ReadJointTelemetry();
  // Publish the transfer-side state: is_moving (firmware executing OR points still
  // queued/awaited), per-trajectory completion, and the committed-depth gate input.
  void PublishTransferTelemetry();
  // Consume one controller command from the mailbox when command_sequence advanced:
  // dispatch the token (Begin/AppendChunk/Abort/Reset), then ack-on-enqueue by
  // echoing the sequence. No-op when no new command is pending.
  void ConsumePendingCommand();

  // Non-blocking, cross-cycle recovery from a transient streaming e-stop.
  void AttemptStreamRecovery();
  // Stop the RMP firmware (Shutdown) and free the MotionController (Delete), in
  // that required order (rsi.h: the only valid call after Shutdown is Delete).
  // Idempotent and noexcept -- the single teardown path for on_cleanup,
  // on_shutdown and the destructor.
  void ReleaseController() noexcept;
  // Throttled per-axis firmware-frame introspection: FRAME_INDEX (consuming),
  // FRAME_LOAD_INDEX (loaded), STATUS bits, feedrate, axis+group state. Used to
  // diagnose why streamed points stop advancing for a >=2-axis MultiAxis. No-op
  // unless debug_frames_ (env RAPIDCODE_FRAME_DEBUG) is set.
  void LogFrameDebug(const char * tag);
  // Drain RapidCode's object-local error log into the ROS logger. Returns true
  // when at least one entry was emitted. When hadError is given, it is set to
  // true if any drained entry was an error rather than a warning.
  bool DrainRapidCodeErrorLog(
    const char * label, RSI::RapidCode::RapidCodeObject * object,
    bool * hadError = nullptr);
  // Drain the error log after creating a RapidCode object and throw if it holds
  // an error. Create, AxisGet, and MultiAxisGet report failures (bad licence,
  // wrong path, firmware not started, missing node) through the object's error
  // log rather than by returning null or throwing, so this is the only way to
  // see them. Warnings are logged and tolerated. Mirrors the RSI sample apps'
  // CheckErrors helper.
  void RequireCleanErrorLog(const char * label, RSI::RapidCode::RapidCodeObject * object);
  // Snapshot MultiAxis state/source/status bits, logging on changes and after
  // RapidCode exceptions. This is deliberately diagnostic-only: it does not clear
  // faults or alter the motion state.
  void LogMultiAxisDiagnostics(const char * context, bool force);
  // Append one CSV row per point in a MovePVT call -- the exact (P,V,A,J,t)
  // the plugin hands the firmware (point-major / axis-minor: pos[p*n + axis]).
  // No-op unless pvt_csv_ is open (env RAPIDCODE_PVT_CSV=<path>). tag marks
  // the source ("stream" | "final"); chunk_id is the ordinal of the MovePVT call,
  // so rows can be grouped back into the chunk they were fed in. Called from
  // FeedChunk().
  void LogPvtPoints(const char * tag, const double * pos, const double * vel,
    const double * acc, const double * jrk, const double * times, int point_count,
    bool final_flag, long chunk_id);

  void BeginTrajectoryTransfer();
  void AppendTrajectory();
  // Recompute which trajectory ids the firmware has fully executed and publish the
  // highest as xferCompletedTrajectoryId. Called every read() cycle. Keys off
  // executed frames (totalFramesFed - frames still queued), NOT MotionDone -- a
  // MotionDone edge fires at every IsFinal, including an interior dwell, which would
  // report a trajectory done early.
  void UpdateCompletion();
  // Drop all per-trajectory completion bookkeeping back to the "nothing begun,
  // nothing completed" base. Called on activate/deactivate and on Reset/Abort/error
  // so a controller restart (or fault recovery) resyncs both sides to a known base.
  void ResetCompletionTracking();
  // Abort/Reset command handlers: clear the queued-chunk FIFO and the transfer
  // FSM (re-arming the firstChunk reopen latch) so a faulted or re-armed goal
  // doesn't inherit the previous goal's tail. Reset additionally clears the
  // latched error_code -- the ONLY place it clears; drive-side fault recovery
  // (ClearFaults) stays a manual RapidSetupX step.
  void HandleAbort();
  void HandleReset();
  // Stop command handler: decelerate the group to rest at the stop rate (amps stay
  // enabled, group settles in STOPPED -- not an error latch), then clear the queued
  // tail and the transfer FSM like HandleAbort. The group leaves STOPPED for IDLE on
  // the operator's Reset (HandleReset ClearFaults a commanded stop; real faults stay
  // a manual RapidSetupX step).
  void HandleStop();
  // Stop feeding and fail the active transfer: optionally e-stop the group, drop the
  // queued tail, latch a nonzero error_code so the controller aborts the goal (instead
  // of hanging on is_moving), and surface any drive-side fault text. Leaves the group
  // stopped/faulted -- recovery needs a ClearFaults (e.g. re-activate the hardware).
  void AbortTransfer(const char * reason, bool eStop, int errorCode);
  // Called from write()'s catch when a std::exception escaped the feed path; e-stops
  // and aborts the transfer via AbortTransfer().
  void HandleWriteFault();
  // Feed as many queued chunks to the firmware as currently fit, front-to-back
  // (FIFO). Recomputes free space each iteration. Called every write() cycle so
  // the trajectory tail still drains after the controller stops sending.
  void DrainQueue();
  // Frames a chunk needs in the firmware buffer (2 frames per point, per MovePVT).
  int32_t NeededFrames(const TrajectoryChunk & chunk) const;
  // Committed-but-unexecuted depth in POINTS (see protocol.hpp CommittedDepthPoints):
  // firmware frames still pending (FramesRemaining, ceil-divided by the 2 frames/point)
  // plus every point parked in the excessChunks host FIFO. Both stores count -- a point
  // acked into the FIFO commits the robot to motion just as surely as a fed frame.
  int32_t CommittedDepthPoints() const;

  rclcpp::Logger logger = rclcpp::get_logger("RapidCodeSystemHardware");

  // RapidCode handles. controller_ is the single rmp owner for this process:
  // created in on_configure via MotionController::Create(&params), one Axis* per
  // joint via AxisGet(axis_index); released in on_cleanup (still a stub).
  RSI::RapidCode::MotionController * controller = nullptr;
  std::vector<RSI::RapidCode::Axis *> axes;
  // All joints are commanded as one coordinated group: a single MultiAxis over
  // axes_ (built in on_configure via MultiAxisGet/AxesAdd). write() issues one
  // MultiAxis::MovePVT per cycle and the hold/stream/finalize state is decided
  // for the group as a whole (multiAxis->StateGet()), not per axis.
  RSI::RapidCode::MultiAxis * multiAxis = nullptr;

  // Controller-wide parameters parsed from the URDF <hardware> block in on_init.
  bool useHardware = false;     // false => phantom axes (no EtherCAT hardware)
  std::string nicPrimary;       // EtherCAT NIC name (empty for phantom)
  int32_t cpuAffinity = 0;      // isolated core for rmp (Linux)
  double sampleRateHz = 1000.0;
  // INFORMATIONAL ONLY. Seconds between amp-enable groups, parsed from the
  // <hardware> param "amp_enable_delay" so the URDF carries the operator's
  // out-of-band enable procedure. The plugin never reads it after on_init and
  // never enables physical amps (see on_activate and the RUNBOOK).
  double ampEnableDelaySeconds = 0.0;
  // Diagnostic: when set (env RAPIDCODE_FRAME_DEBUG), write() logs throttled
  // per-axis firmware-frame state so the >=2-axis streaming stall is visible.
  // Read once in on_init; zero cost on the hot path when off.
  bool debugFrames = false;

  int frameBufferSize = 0;

  // Optional CSV trace of every MovePVT input point (env RAPIDCODE_PVT_CSV=
  // <path>). Opened in on_activate once axes_ are known; closed in
  // ReleaseController. pvt_seq_ is a monotonic row counter; pvt_csv_t_ is the
  // cumulative firmware time (sum of per-point dt) so the trace lines up with the
  // recorder's time axis. nullptr (the default) => zero cost on the hot path.
  std::FILE * pvtCsv = nullptr;
  long pvtSeq = 0;
  long pvtChunkId = 0;  // ordinal of the MovePVT call each logged row belongs to
  double pvtCsvTime = 0.0;

  // Read-loop diagnostics are decimated so normal telemetry does not spam logs.
  int rapidCodeDiagReadCycles = 0;
  bool haveLastMultiAxisDiag = false;
  RSI::RapidCode::RSIState lastMultiAxisDiagState =
    RSI::RapidCode::RSIState::RSIStateIDLE;
  uint64_t lastMultiAxisDiagStatusBits = 0;

  // write() liveness heartbeat + drain-stall throttle (see write()/DrainQueue).
  uint64_t writeCycles = 0;      // ++ every write(); heartbeat every kWriteHeartbeatPeriod
  int drainBlockedCycles = 0;    // throttles the "DrainQueue blocked" warn; reset when unblocked

  // Per-joint parameters parsed in on_init (indexed like info_.joints):
  //   axis_indices_[j] = RapidCode axis number   (URDF joint <param name="axis">)
  //   user_units_[j]   = counts per user unit     (URDF joint <param name="user_units">)
  std::vector<int32_t> axisIndices;
  std::vector<double> userUnits;
  // Optional hardware-calibration parameters (all indexed like info_.joints;
  // ignored in phantom mode). Defaults reproduce the pre-calibration behavior:
  //   origins[j]         = axis origin in RAW ENCODER COUNTS (OriginPositionSet);
  //                        applied only when hasOrigin[j] (URDF supplied "origin").
  //   ampEnableGroups[j] = amp-enable group (URDF "amp_enable_group", default 0).
  //                        INFORMATIONAL ONLY: documents the operator's manual
  //                        enable order; the plugin does not act on it.
  //   errorLimits[j]     = following-error trigger in user units (URDF
  //                        "error_limit"); E_STOP action, applied only when
  //                        hasErrorLimit[j]. Left at the drive default otherwise.
  std::vector<double> origins;
  std::vector<bool> hasOrigin;
  std::vector<int32_t> ampEnableGroups;
  std::vector<double> errorLimits;
  std::vector<bool> hasErrorLimit;

  // RMP Recorder capturing the motion profile (per axis: command/actual position
  // and velocity), set up + started in on_configure and removed in on_cleanup.
  // -1 = none. Read it back with RecorderRecordDataRetrieve + ...ValueGet, or
  // scope it live in RapidSetupX/WorkBench over rapidserver.
  int32_t recorderIndex = -1;


  // Command-mode
  enum class CommandMode
  {
    Streaming,
    Passthrough,
  };
  CommandMode commandMode = CommandMode::Streaming;

  uint64_t ackSequence = 0; // The last command_sequence acted upon and acknowledged.
  int transferTrajectorySize = -1; // The total number of points in the current transfer.
  int transferValidFields = 0; // The valid fields (position, velocity, acceleration, jerk) in the current transfer.
  bool transferMoveOpen = false; // Whether a move is currently in progress.
  // True whenever the firmware move is currently CLOSED, so the next chunk fed must
  // (re)open it. Starts true (nothing open yet); set false once the opening chunk is
  // fed; set true again when an IsFinal chunk is consumed (that closes the move). No
  // longer tied to BEGIN, so a BEGIN on a still-open, flowing move never re-toggles it.
  bool firstChunk = true;

  // A Reset arrived while the group was still decelerating from a commanded stop
  // (STOPPING or, with commandedStopFault, STOPPING_ERROR): the ClearFaults back to
  // IDLE is owed and runs in DrainQueue once the group settles. Never set for real
  // (unflagged) fault states.
  bool stopRecoverPending = false;

  // The group's current/pending ERROR is SELF-INFLICTED: HandleStop commanded an
  // EStop() to close the open streamed move (Stop() would leave the OUT_OF_FRAMES
  // supervisor armed). HandleReset may ClearFaults this ERROR; real faults never
  // set the flag and stay a manual recovery. Cleared on Reset, Abort, or any
  // AbortTransfer (a real fault supersedes it).
  bool commandedStopFault = false;

  // --- Per-trajectory completion tracking (multi-trajectory streaming) -----------
  // currentTrajectoryId is the id from the most recent BEGIN; chunks enqueued after it
  // inherit it. totalFramesFed accumulates frames actually handed to the firmware (2
  // per point). Each BEGIN appends its cumulative expected-frame threshold so read()
  // can report the highest fully-executed id.
  std::uint64_t currentTrajectoryId = 0;
  std::uint64_t totalFramesFed = 0;
  std::uint64_t cumulativeExpectedFrames = 0;
  struct TrajectoryCompletion { std::uint64_t id = 0; std::uint64_t cumulativeFrames = 0; };
  std::deque<TrajectoryCompletion> trajectoryCompletions;

  // trajectory_transfer interface backing storage. Prefixed with `xfer_`
  // All ros2_control interfaces are `double`.
  // Pointers to these fields are exported as the command and state interfaces.
  // ros2_control caches the raw pointers.
  // These fields are stable for the component's lifetime
  // (AllocateTransferStorage in on_init);
  //   command scalars (controller -> hw, read in write()):
  double xferCommandToken = 0.0;           // kCmdCommandToken        (Command token)
  double xferCommandSequence = 0.0;  // kCommandSequence
  double xferTrajectoryId = 0.0;     // kCmdTrajectoryId
  double xferTrajectorySize = 0.0;      // kCmdTotalPoints
  double xferValidFields = 0.0;      // kCmdValidFields
  double xferChunkBaseIndex = 0.0;        // kCmdChunkBase
  double xferChunkLen = 0.0;         // kCmdChunkLen
  double xferChunkFinal = 0.0;       // kCmdChunkFinal
  //   state scalars (hw -> controller, written in read()):
  double xferIsMoving = 0.0;  // kMotionDone
  double xferAckSequence = 0.0;    // kAckSequence
  double xferAcceptedPointIndex = 0.0;  // kAcceptedPointIndex
  double xferErrorCode = 0.0;      // kErrorCode
  double xferCompletedTrajectoryId = 0.0;  // highest fully-executed trajectory id (0 = none)
  // Committed depth (see protocol.hpp CommittedDepthPoints): points accepted but not yet
  // executed, in points = firmware frames pending / 2 + points queued in excessChunks.
  // Refreshed every read(); the controller's online feed gate throttles against it.
  double xferCommittedDepthPoints = 0.0;

  //   chunk payload, slot-major / joint-minor, sized [n*K] (and [K] for duration):
  std::vector<double> xferSlotDuration;      // [kChunkCapacity]  (== times[] for MovePVT)
  std::vector<double> xferSlotPosition;       // [n_joints * kChunkCapacity]
  std::vector<double> xferSlotVelocity;
  std::vector<double> xferSlotAcceleration;
  std::vector<double> xferSlotJerk;

  std::vector<double> statePositions;
  std::vector<double> stateVelocities;
protected:
  // Host FIFO of chunks the firmware frame buffer couldn't accept yet, drained by
  // DrainQueue() as space frees. Owned + touched only on the control-loop thread
  // (read()/write() run sequentially there), so no locking is needed.
  std::deque<TrajectoryChunk> excessChunks;
  // True from BeginTrajectoryTransfer() until the chunk carrying final=true is
  // actually fed to the firmware. Keeps is_moving asserted across the two windows
  // !excessChunks.empty() misses: just after Begin (before any chunk arrives) and
  // the instant the final chunk leaves the queue but motion hasn't registered yet.
  bool pendingFinal = false;

  // --- Backpressure seams ----------------------------------------------------
  // Virtual so a unit-test subclass can drive the queue without a live rmp: it
  // overrides these to script free space and record feeds. Production reads/feeds
  // the real MultiAxis.
  //
  // Frames free in the firmware buffer: min across axes, minus kSafetyMargin.
  // Null-safe (skips unconfigured axes) so the no-rmp tests don't dereference.
  virtual int32_t FreeFramesAvailable() const;
  // Frames still queued in the firmware buffer: MAX across axes (most-remaining =
  // slowest-draining axis), so executed-frame accounting for completion is
  // conservative (reports a trajectory done late, never early). Null-safe (returns 0
  // with no live axes) and virtual so the no-rmp tests can script drain progress.
  virtual int32_t FramesRemaining() const;
  // Issue one MovePVT for the chunk (and trace it); clears pendingFinal on the
  // final chunk. No-op on the firmware call when multiAxis is null (still traces).
  virtual void FeedChunk(const TrajectoryChunk & chunk);
};

}  // namespace rapidcode_system

#endif  // RAPIDCODE_SYSTEM__RAPIDCODE_SYSTEM_HARDWARE_HPP

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

// Passthrough trajectory controller for RapidCode (ros2_control).
//
// The hardware-agnostic half of the trajectory_transfer passthrough path: it
// hosts a FollowJointTrajectory action server (so MoveIt drives it exactly like a
// stock JointTrajectoryController), and instead of sampling the trajectory to one
// setpoint per cycle, it streams the trajectory's points through the
// trajectory_transfer command/state interfaces to whatever hardware implements
// them (here, rapidcode_system, which interpolates via MovePVT). update() runs
// the per-cycle streaming decision inline (adopt goal + BEGIN -> ack-gated chunk
// stream -> drain -> report the action result); see the Phase enum below.
//
// A second intake, the "~/joint_trajectory" TOPIC subscriber, drives continuous
// online jogging (MoveIt Servo / a jog publisher): a latest-wins stream fed as ONE
// long-lived OPEN firmware move (every chunk finalize=false), stopped by a
// controller-generated quintic decel-to-rest tail (finalize=true) on producer
// silence. The action path (finite goals) and the online path (open jog) are
// mutually exclusive -- reject, don't preempt.
//
// Activate this controller XOR the stock JointTrajectoryController -- they both
// claim command interfaces and are mutually exclusive (the hardware flips
// active_mode_ in perform_command_mode_switch).

#pragma once
#ifndef RAPIDCODE_PASSTHROUGH_TRAJECTORY_CONTROLLER__PASSTHROUGH_TRAJECTORY_CONTROLLER_HPP
#define RAPIDCODE_PASSTHROUGH_TRAJECTORY_CONTROLLER__PASSTHROUGH_TRAJECTORY_CONTROLLER_HPP

#include <atomic>
#include <cstdio>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_server_goal_handle.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "rapidcode_trajectory_transfer/protocol.hpp"

namespace rapidcode_passthrough_trajectory_controller
{
// Per-segment resampling order, mirroring ros2_controllers
// JointTrajectoryController::interpolate_between_points. Auto picks the order from the
// fields present on the segment endpoints -- positions -> Linear, +velocities -> Cubic,
// +accelerations -> Quintic -- exactly as the JTC does; the others force one order
// regardless of the fields. The integer values are the position polynomial's degree.
// Quadratic is forced-only (Auto never selects it, matching JTC which has no quadratic):
// it interpolates velocity linearly, giving constant acceleration per segment (a
// trapezoid, no overshoot/oscillation) at the cost of not pinning the endpoint position.
enum class Interpolation : int { Auto = 0, Linear = 1, Quadratic = 2, Cubic = 3, Quintic = 5 };

struct GoalTrajectory
{
  uint32_t num_joints = 0;
  uint32_t num_points = 0;
  int valid_fields = 0;  // rapidcode_trajectory_transfer::FieldMask* bitmask (whole trajectory)
  // Whether this trajectory's trailing chunk should close the firmware move (IsFinal).
  // Default true; the auto queued-behind policy in update() may still withhold it when
  // another goal is queued right behind this one.
  bool finalize = true;
  std::vector<double> durations;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;
  std::vector<double> jerks;

  GoalTrajectory(std::size_t num_joints, std::size_t num_points) : num_joints(num_joints), num_points(num_points)
  {
    durations.reserve(num_points);
    const std::size_t vector_size = num_joints * num_points;
    positions.reserve(vector_size);
    velocities.reserve(vector_size);
    accelerations.reserve(vector_size);
    jerks.reserve(vector_size);
  }
};

class PassthroughTrajectoryController : public controller_interface::ControllerInterface
{
  // Test fixture verifies the controller's raw trajectory_transfer command-interface
  // output without controller_manager or RapidCode hardware in the loop.
  friend class PassthroughTrajectoryControllerTest;
  friend class PassthroughTrajectoryControllerMotionTest;

public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<FollowJointTrajectory>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;

  PassthroughTrajectoryController() = default;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // A goal moving through the pipeline. traj + goal + terminal are set at accept time;
  // trajectory_id and next_index are assigned/advanced by update() (RT) as the goal is
  // fed. `terminal` is shared with the timer's MonitoredGoal: the RT thread sets it true
  // once it reports a result, and the timer prunes the monitored entry after flushing.
  // (Defined here, before the helpers that take it as a parameter.)
  struct GoalEntry
  {
    std::shared_ptr<GoalTrajectory> traj;
    RealtimeGoalHandlePtr goal;
    std::shared_ptr<std::atomic<bool>> terminal;
    // Mirror of trajectory_id shared with the timer's MonitoredGoal, so the feedback
    // builder can match the RT snapshot's executing id to a goal without touching RT state.
    std::shared_ptr<std::atomic<std::uint64_t>> assigned_id;
    std::uint64_t trajectory_id = 0;  // assigned at BEGIN (0 == not yet fed)
    int next_index = 0;               // chunk cursor while feeding
  };
  struct MonitoredGoal
  {
    RealtimeGoalHandlePtr goal;
    std::shared_ptr<std::atomic<bool>> terminal;
    std::shared_ptr<GoalTrajectory> traj;                    // for feedback `desired`
    std::shared_ptr<std::atomic<std::uint64_t>> assigned_id;  // set by RT at BEGIN
  };

  // --- on_configure blocks (executor thread) ----------------------------------
  // Cycle-period check: is the controller manager's cycle sustainable by the
  // firmware? The RMP consumes each transferred point as a whole number of its own
  // samples, so the update period must be a whole multiple of sample_period (sub-sample
  // jitter cannot be expressed in MovePVT time), and that multiple must be at least
  // kMinSamplesPerCycle -- at 1:1 the soft-RT loop feeds slower than the firmware drains
  // and the stream e-stops OUT_OF_FRAMES (see the rapidcode_bringup controller yamls).
  // Pure; false with a human-readable reason (the hermetic test drives it via the fixture).
  static bool CyclePeriodSustainable(
    double update_period_seconds, double sample_period_seconds, std::string & reason);
  static constexpr int kMinSamplesPerCycle = 2;
  // Relative slack for the whole-multiple test (update_rate is an integer in Hz, so
  // 1/update_rate carries representation error against a decimal sample_period).
  static constexpr double kCycleRatioTolerance = 1.0e-6;

  // Read + validate the core controller parameters (joints, gpio, chunk size,
  // tolerances, interpolation, finalize policy). False (with an error log) on an
  // invalid config. Side-effects: the corresponding parameter members.
  bool ReadControllerParameters();

  // Read + validate the online (jog) parameter group and derive the feed-gate band in
  // points. False (with an error log) on an invalid config. Side-effects: the online_*
  // parameter members.
  bool ReadOnlineParameters();

  // Size the cached interface-index arrays to the configured joint count (indices are
  // resolved in on_activate).
  void SizeInterfaceIndexCaches();

  // Create the two intakes: the FollowJointTrajectory action server and, when
  // online.enabled, the ~/joint_trajectory jog subscription.
  void CreateIntakes();

  // --- on_activate / on_deactivate blocks (executor thread) --------------------
  // Resolve every trajectory_transfer + joint interface name to a cached index so
  // update() never searches by name. False when any interface is missing (bad URDF /
  // joint list). Side-effects: the ci_* / si_* index members.
  bool ResolveInterfaceIndices();

  // Reset the multi-goal pipeline to empty (feeding_/pending_/inflight_/incoming_/
  // monitored_goals_, id counter, seam, feedback throttle). Called from on_activate
  // before update() runs, so the RT-owned members are safe to touch.
  void ResetActionPipeline();

  // Pre-allocate the online (jog) working set -- committed seed, stop-tail and splice
  // buffers (worst-case reserves), scratch vectors, identity mapping -- so the RT path
  // never allocates; then flush any stale staged snapshot and reset the online state.
  void PrepareOnlineWorkingSet();

  // Start the goal-monitor wall timer that flushes each in-flight goal's queued
  // result/feedback off the RT path (runNonRealtime) and prunes terminal entries.
  void StartGoalMonitorTimer();

  // Open the opt-in RAPIDCODE_TRAJ_INPUT_CSV input trace (no-op unless the env var is
  // set and the file isn't already open) and write its header.
  void OpenInputTrace();

  // Abort every monitored goal (whatever pipeline stage it is in) with "controller
  // deactivated" and clear the monitored set, so no client dangles when the controller
  // is switched out.
  void AbortAllMonitoredGoals();

  // --- action server callbacks (executor thread) ------------------------------
  rclcpp_action::GoalResponse HandleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse HandleCancel(std::shared_ptr<GoalHandle> goal_handle);
  void HandleAccepted(std::shared_ptr<GoalHandle> goal_handle);

  // Goals accepted but not yet reported terminal (locks monitored_mutex_) -- the
  // action-layer backpressure count HandleGoal bounds against kMaxInFlightGoals.
  std::size_t ActiveMonitoredGoalCount();

  // --- online (jog) topic path ------------------------------------------------
  // Executor-thread subscription callback: validate + remap a streamed
  // JointTrajectory into a finalize=false snapshot and hand it to the RT thread
  // (latest-wins single slot). Drops the message when the action pipeline is busy
  // (mutual exclusion) or the trajectory is malformed. Never touches the RT-owned
  // pipeline members; the only shared state it reads is the action_busy_ atomic.
  void JointTrajectoryTopicCallback(
    const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr message);

  // True while the action pipeline owns the mailbox (a goal is feeding, pending, or
  // in flight). RT thread only -- it reads the RT-owned deques directly. Executor
  // threads (the callback / HandleGoal) read the action_busy_ / online_busy_ atomics.
  bool ActionBusy() const;

  // --- update() dispatch (all RT thread) ---------------------------------------
  // The hardware-reported scalars update() dispatches on, decoded once per cycle.
  // prev_acked is the single-slot mailbox handshake: the hardware has consumed the
  // last command once it echoes command_sequence_ back.
  struct HardwareState
  {
    uint64_t ack_sequence = 0;
    int error_code = 0;
    uint64_t completed_id = 0;  // highest trajectory id the hardware finished
    bool prev_acked = false;
  };

  // Decode the per-cycle hardware state scalars from the state interfaces.
  HardwareState ReadHardwareState() const;

  // Finish every in-flight goal the hardware has fully executed (its id <=
  // completed_id). Only the goal that ends the chain at rest (traj->finalize) gets the
  // live goal-tolerance check; mid-chain the arm has already moved past its endpoint.
  void RetireCompletedGoals(uint64_t completed_id);

  // Hardware fault (error_code != 0): abort every queued/in-flight goal, drop the open
  // jog session, and hold the mailbox at Cmd::None -- every cycle until the error
  // clears. When the operator has acknowledged the fault via ~/reset_fault
  // (reset_requested_) and the mailbox is free (prev_acked), emit Cmd::Reset instead:
  // the hardware's HandleReset clears the latched error_code and this branch stops
  // being taken. Drive-side recovery (ClearFaults) stays a manual RapidSetupX step.
  void HandleHardwareFault(bool prev_acked);

  // Own the cycle while the operator stop latch (~/stop) holds (RT). Every latched
  // cycle: abort any goal that raced the latch, keep the jog session torn down, and
  // mark staged snapshots seen (stale by definition). Emit Cmd::Stop as soon as the
  // mailbox is free (the hardware decelerates the group to rest at the stop rate),
  // then hold Cmd::None. The operator's ~/reset_fault emits Cmd::Reset -- re-arming
  // the hardware and clearing its commanded STOPPED state -- and releases the latch,
  // ordered after the pending Stop so a release can never overtake the stop itself.
  // Returns true while the latch owns the cycle.
  bool ServiceOperatorStop(bool prev_acked);

  // Drive the action pipeline for one acked, online-idle cycle: adopt the next pending
  // goal (BEGIN) when nothing is feeding, else present the feeding goal's next chunk.
  // Sets command/emit when it emits a command.
  void ServiceActionStream(
    rapidcode_trajectory_transfer::CommandToken & command, bool & emit);

  // Pop and adopt the next pending goal: check first-point continuity (abort
  // INVALID_GOAL on violation -- reject, don't e-stop), assign the monotonic
  // trajectory id, write the BEGIN payload (id/size/valid_fields), and record the
  // goal's endpoint as the seam for whatever is queued behind it. Sets command/emit
  // on a successful adopt.
  void TryAdoptPendingGoal(
    rapidcode_trajectory_transfer::CommandToken & command, bool & emit);

  // Present the feeding goal's next chunk (AppendChunk); the trailing chunk carries
  // IsFinal per the goal's finalize policy and moves the goal to inflight_ (completion
  // is reported by id, not is_moving). Sets command/emit.
  void FeedActionChunk(
    rapidcode_trajectory_transfer::CommandToken & command, bool & emit);

  // First-point continuity precondition for adopting `traj`: every joint's first point
  // must lie within first_point_tolerance_ of the reference -- the previous adopted
  // goal's endpoint (the seam MoveIt planned to) while a chain is executing, else the
  // live measured joint state.
  bool FirstPointContinuous(const GoalTrajectory & traj) const;

  // Record `traj`'s final point (joints_ order) as the continuity seam for the next
  // queued goal. Side-effects: last_adopted_end_, have_last_end_.
  void RecordAdoptedEndSeam(const GoalTrajectory & traj);

  // Drive the command interfaces for this cycle: bump command_sequence_ and write
  // command+sequence when emit is set (the ONLY place the sequence increments, so the
  // prev_acked handshake stays meaningful), else write Cmd::None.
  void EmitCommand(rapidcode_trajectory_transfer::CommandToken command, bool emit);

  // Publish the lock-free arbitration mirrors (action_busy_ / online_busy_) the
  // executor-thread callbacks read. Reject, don't preempt.
  void PublishArbitration();

  // Drive the whole online branch for this cycle (RT): ingest a fresh snapshot (adopt
  // an open move -> BEGIN, or replace-on-receive), advance the producer-liveness
  // timer, and feed a chunk / synthesize the decel-to-rest stop-tail. Sets
  // command/emit when it emits a BEGIN or AppendChunk this cycle. `period_seconds` is
  // the control period (drives the producer-silence timeout); `committed_depth_points`
  // is the hardware-reported committed depth the feed gate throttles against (see the
  // bounded-lookahead comment block by online_committed_horizon_). Reuses PresentChunk
  // and the shared command-sequence mailbox exactly like the action feed loop.
  void ServiceOnlineStream(bool prev_acked, double period_seconds,
    int committed_depth_points,
    rapidcode_trajectory_transfer::CommandToken & command, bool & emit);

  // Act on a fresh snapshot (RT). Inactive: seed the blend anchor from the measured
  // joint state, build the spliced stream, and open a new move (emit BEGIN). Active:
  // re-anchor onto the committed stream via the same builder. Either way the window's
  // absolute positions are never commanded, so there is no continuity gate -- windows
  // are absorbed, not dropped. Sets command/emit only when it emits a BEGIN; an
  // un-acked adopt is left for the next cycle to retry.
  void IngestOnlineSnapshot(const std::shared_ptr<GoalTrajectory> & snapshot,
    bool prev_acked, rapidcode_trajectory_transfer::CommandToken & command, bool & emit);

  // Seed the online blend anchor (the committed P,V) from the measured joint state,
  // velocities clamped to online.max_velocity -- the blend base for the FIRST window
  // of a jog, where no committed stream exists yet. RT.
  void SeedOnlineAnchorFromState();

  // The ONE builder for online staged streams (RT, producer rate): a quintic velocity
  // blend from the anchor -- the committed (P,V); the first adopt seeds it from the
  // measured state -- to the window's velocity intent (its last point's velocities,
  // clamped to online.max_velocity), then a constant-velocity extension that outlives
  // the producer-silence timeout so a short window can never starve the firmware
  // before the stop-tail fires. The window's absolute positions are deliberately
  // ignored (they anchor to the producer's stale, measured view of the robot -- the
  // live position-error/starvation churn came from commanding them). Refills
  // online_splice_traj_ in place and swaps it in as online_traj_ with the cursor
  // rewound to the blend start; emits no command. Returns false -- stream untouched --
  // only on a degenerate setup (no joints / no buffer / empty snapshot).
  bool SpliceOnlineSnapshot(const GoalTrajectory & snapshot);

  // A session boundary just passed (stop-tail closed the move, a fault reset, or
  // deactivate): mark whatever snapshot is parked in the latest-wins buffer as already
  // seen. Anything staged before the boundary is stale by definition -- adopting it
  // spawned ghost reopens that commanded hundreds-of-ms-old positions. RT.
  void MarkStagedSnapshotSeen();

  // Emit the open-move BEGIN for the online stream (RT): trajectory_id 0 (an open move
  // is not completion-tracked) and trajectory_size -1 (the hardware pushes no
  // completion threshold for a non-positive size), plus the valid-field mask.
  void EmitOnlineBegin();

  // Present the next online chunk (RT): reuse PresentChunk, advance the cursor, capture
  // the committed state for a future stop-tail, and, when the finalizing stop-tail's
  // last chunk goes out, close the online session. Sets command/emit. `max_points`
  // caps the chunk below chunk_size_ so one feed can never overshoot the high-water
  // mark (the "horizon space" cap: chunk_size_ sizes the mailbox pipe, the gate sizes
  // how much future may be committed); a non-positive cap feeds nothing.
  void FeedOnlineChunk(int max_points,
    rapidcode_trajectory_transfer::CommandToken & command, bool & emit);

  // Build the quintic decel-to-rest stop-tail from the committed end-of-stream state
  // (seed acceleration 0 -- the burst-runaway fix), swap it in as online_traj_ with
  // finalize=true, and enter the stopping state. Returns false if it cannot be built
  // (degenerate), leaving the firmware OUT_OF_FRAMES e-stop as the backstop. RT thread;
  // refills the pre-allocated online_stop_traj_ buffer (no RT heap allocation).
  bool BeginOnlineStopTail();

  // Capture online_traj_'s (position, velocity) at point_index into the committed-state
  // seed used by BeginOnlineStopTail. No-op once stopping (keep the stream's state).
  void CaptureOnlineCommitted(int point_index);

  // Reset all RT-owned online state to idle (called on activate/deactivate, on a
  // hardware fault, and after a stop-tail closes the session).
  void ResetOnlineState();

  // Remap a trajectory from its own joint order into joints_ order, resampling each
  // segment onto the sample_period_ grid and computing per-point valid_fields +
  // cumulative time. Returns nullptr on a structural problem (missing joint / empty).
  // The Goal overload is a thin wrapper so the action + online intakes share one
  // resampler. Runs on the executor thread (HandleAccepted / the topic callback).
  std::shared_ptr<GoalTrajectory> RemapTrajectory(
    const trajectory_msgs::msg::JointTrajectory & trajectory) const;
  std::shared_ptr<GoalTrajectory> RemapTrajectory(
    const FollowJointTrajectory::Goal & goal) const;

  // Resolve each managed joint's column within the trajectory's own joint order; empty
  // if a managed joint is absent. Shared by RemapTrajectory and the input trace so the
  // mapping lives in one place. The Goal overload forwards to the trajectory one.
  // Executor thread.
  std::vector<int> ResolveJointMapping(
    const trajectory_msgs::msg::JointTrajectory & trajectory) const;
  std::vector<int> ResolveJointMapping(const FollowJointTrajectory::Goal & goal) const;

  // Write the input-trace CSV column header to input_csv_ (no-op if it isn't open).
  // Extracted so on_activate and the unit tests share one column layout.
  void WriteInputTraceHeader();

  // Append the goal's raw waypoints (pre-interpolation, remapped to joints_ order) to
  // the RAPIDCODE_TRAJ_INPUT_CSV trace when it is open; a no-op otherwise. Executor
  // thread (HandleAccepted), never the RT update() path.
  void LogInputTrajectory(const FollowJointTrajectory::Goal & goal);

  // Find a loaned command/state interface by full name; -1 if absent. Used once in
  // on_activate to cache indices so update() is allocation- and lookup-free.
  int FindCommandInterface(const std::string & full_name) const;
  int FindStateInterface(const std::string & full_name) const;

  // Thin wrappers over the Jazzy loaned-interface accessors: get_optional()
  // (get_value() is deprecated) and the [[nodiscard]] bool set_value(). A failed
  // set_value would leave the hardware reading a stale command (a dropped POINT/
  // BEGIN stalls the FSM), so we don't silently discard it -- warn, throttled.
  double ReadStateValue(int index) const;
  void SetCommandValue(int index, double value);

  // Lifecycle hard stop: write Cmd::Stop to the mailbox and hold the
  // stop latch. Shared by on_deactivate() and on_shutdown(). A no-op when the
  // interfaces were never resolved (activation never completed).
  void EmitLifecycleStop();
  // Drop every pipeline, abort monitored goals, close the trace file. Idempotent; the
  // shared tail of on_deactivate() and on_shutdown().
  void TearDownPipelines();

  // Write a chunk's payload (base/len/final + per-transfer valid_fields, then per-slot
  // time and per-joint setpoints) into the command interfaces for the chunk being
  // presented this cycle. Reads the SoA `traj` at global points base..base+len-1.
  void PresentChunk(const GoalTrajectory & traj, int base, int len, bool final);

  // RT: record which goal the firmware is executing (head of inflight_, else feeding_),
  // how long it has been executing, and the live joint positions, into
  // feedback_snapshot_. Lock-free atomic stores only: no allocation, no mutex.
  void UpdateFeedbackSnapshot(const rclcpp::Duration & period);
  // Timer thread: build the FollowJointTrajectory feedback for the executing goal from
  // the RT snapshot (desired = trajectory sampled at the elapsed time, actual = live
  // positions, error = desired - actual) and queue it with setFeedback(). Runs right
  // before runNonRealtime() on the same thread, so the goal handle's lock is free and
  // the message is never shared with the RT thread. Allocation is fine here.
  void PublishGoalFeedback(
    const MonitoredGoal & monitored, std::uint64_t executing_id, double elapsed_seconds);

  // Drain the executor->RT incoming-goal queue into pending_ (try_lock; deferred to the
  // next cycle on lock contention -- pushes are rare). RT thread.
  void DrainIncomingGoals();

  // Report a terminal action result for one goal (RT thread) and mark it terminal so the
  // monitored-goal timer flushes + prunes it. FinishGoal reports SUCCESSFUL, plus the
  // live goal-tolerance check when do_tolerance (only for the goal that ends the chain at
  // rest -- mid-chain the arm has already moved on, so the check is meaningless).
  void FinishGoal(const GoalEntry & entry, bool do_tolerance);
  void AbortGoal(const GoalEntry & entry, int32_t result_code, const std::string & message);
  // Abort every goal still in the pipeline (feeding_, inflight_, pending_) and clear them
  // -- used when the hardware faults.
  void AbortAllGoals(int32_t result_code, const std::string & message);
  // True if the trajectory's final point is within goal_position_tolerance_ of the live
  // joint state (si_joint_position_).
  bool WithinGoalTolerance(const GoalTrajectory & traj) const;

  // --- parameters -------------------------------------------------------------
  std::vector<std::string> joint_names_;
  std::string gpio_name_ = rapidcode_trajectory_transfer::DefaultGpioName;
  int chunk_size_ = rapidcode_trajectory_transfer::ChunkCapacity;  // points/cycle, <= K
  double goal_position_tolerance_ = 0.05;
  double first_point_tolerance_ = 0.05;
  std::string action_name_ = "~/follow_joint_trajectory";
  static constexpr double DefaultSamplePeriod = 0.001;  // seconds
  double sample_period_ = DefaultSamplePeriod;
  Interpolation interpolation_ = Interpolation::Auto;  // resampler order; see Interpolation
  // Master default for GoalTrajectory::finalize (whether a goal's trailing chunk closes
  // the firmware move). true reproduces stop-at-end behavior; set false for a continuous
  // producer that should never finalize. The auto queued-behind policy still applies.
  bool finalize_last_chunk_ = true;

  // --- online (jog) topic parameters ------------------------------------------
  bool online_enabled_ = true;                 // create the ~/joint_trajectory subscription
  // Producer silence (seconds, wall time from the last accepted snapshot) that triggers
  // the decel-to-rest stop-tail. A healthy jog producer republishes faster than this.
  double online_producer_timeout_ = 0.05;
  // --- bounded lookahead (the online feed gate) --------------------------------
  // "Committed depth" = trajectory points the hardware has accepted but the robot has
  // not executed yet (firmware frame buffer + the hardware's host FIFO), reported by
  // the committed_depth_points state interface. It is how far into the future the
  // robot is already committed: large depth = laggy jog response, long stop overshoot,
  // and a big seam for a splice to absorb. The gate below keeps it inside a classic
  // two-threshold hysteresis band instead of a single edge:
  //   depth >= low water  -> HOLD (enough motion is committed; let execution drain it)
  //   depth <  low water  -> refill: feed chunks until depth reaches the high water,
  //                          then hold again.
  // The band (rather than one threshold) avoids drip-feeding 1-2 point chunks every
  // cycle at the edge, and the low water doubles as the starvation floor: it must
  // cover a few missed update cycles so the firmware buffer can never drain to empty
  // mid-open-move (a starvation fault). Applies ONLY to the online (jog) stream; the
  // action path keeps its feed-all behavior (finite goals want the whole buffer).
  //
  // High-water mark, in seconds of motion (converted to points via sample_period).
  // The most future motion the online stream ever commits -- the lookahead bound.
  double online_committed_horizon_ = 0.08;
  // Low-water mark, in seconds of motion. Refilling starts when the committed depth
  // drains below this. Keep it a few controller update periods so jitter can't
  // starve the firmware, and below committed_horizon (validated in on_configure).
  double online_low_water_ = 0.03;
  // The two marks converted to points on the sample_period grid (on_configure).
  int online_low_water_points_ = 0;
  int online_high_water_points_ = 0;
  // Per-joint kinematic limits for the stop-tail (rad/s, rad/s^2, rad/s^3). Filled to
  // joint count with the defaults below when the param arrays are empty.
  std::vector<double> online_max_velocity_;
  std::vector<double> online_max_acceleration_;
  std::vector<double> online_max_jerk_;
  static constexpr double kDefaultOnlineMaxVelocity = 1.0;
  static constexpr double kDefaultOnlineMaxAcceleration = 2.0;
  static constexpr double kDefaultOnlineMaxJerk = 20.0;
  // Floor / cap for the synthesized stop-tail duration (seconds). The floor keeps a
  // near-stationary stop well-formed (>0, a few samples); the cap bounds the reused
  // buffer's reserve and a pathological limit config.
  static constexpr double kMinStopTailDuration = 0.01;
  static constexpr double kMaxStopTailDuration = 2.0;

  uint64_t command_sequence_ = 0;

  // --- cached interface indices (resolved in on_activate) ---------------------
  // Slot arrays are slot-major ([slot*n + joint]), matching the hardware export.
  int ci_command_ = -1;
  int ci_command_sequence_ = -1;
  int ci_trajectory_id_ = -1;
  int ci_trajectory_size_ = -1;
  int ci_valid_fields_ = -1;
  int ci_chunk_base_ = -1;
  int ci_chunk_len_ = -1;
  int ci_chunk_final_ = -1;
  std::vector<int> ci_slot_time_;             // [slot]
  std::vector<int> ci_slot_joint_position_;   // [slot*n + joint]
  std::vector<int> ci_slot_joint_velocity_;
  std::vector<int> ci_slot_joint_acceleration_;
  std::vector<int> ci_slot_joint_jerk_;

  int si_is_moving_ = -1;
  int si_ack_sequence_ = -1;
  int si_accepted_index_ = -1;
  int si_error_code_ = -1;
  int si_completed_trajectory_id_ = -1;     // highest trajectory id the hardware finished
  int si_committed_depth_ = -1;             // committed depth in points (online feed gate)
  std::vector<int> si_joint_position_;      // per joint (continuity + tolerance + feedback)
  std::vector<int> si_joint_velocity_;

  // --- multi-goal pipeline -----------------------------------------------------
  // RT-owned pipeline (touched only by update()). Flow:
  //   HandleAccepted (executor) -> incoming_ (mutex) --DrainIncomingGoals--> pending_
  //   -> feeding_ (BEGIN + chunks) -> inflight_ (awaiting completed_trajectory_id)
  //   -> finished (result reported; the monitored-goal timer flushes it).
  std::optional<GoalEntry> feeding_;   // goal whose BEGIN/chunks update() is emitting
  std::deque<GoalEntry> pending_;      // accepted, not yet fed
  std::deque<GoalEntry> inflight_;     // fully fed, awaiting completion (ascending id)
  std::uint64_t next_trajectory_id_ = 1;  // monotonic; assigned at BEGIN (0 == none)
  // Last adopted goal's final point (joints_ order): the seam a queued goal's first point
  // is checked against (the arm is mid-motion, so live state is the wrong reference).
  std::vector<double> last_adopted_end_;
  bool have_last_end_ = false;         // false when the chain is fully idle -> use live state
  std::atomic<bool> cancel_requested_{false};

  // --- RT -> timer feedback snapshot -------------------------------------------
  // Written by update() every cycle with plain atomic stores, read by the goal-monitor
  // timer. Per-field atomics: joint values need not be mutually coherent for a
  // progress display, and this keeps the RT path free of locks and allocation.
  // actual_positions is sized once in on_configure (a vector of atomics cannot be
  // resized on the RT path).
  struct FeedbackSnapshot
  {
    std::atomic<std::uint64_t> trajectory_id{0};  // 0 == no goal executing
    std::atomic<double> elapsed_seconds{0.0};     // since that goal became the head
    std::vector<std::atomic<double>> actual_positions;
  };
  FeedbackSnapshot feedback_snapshot_;
  std::uint64_t feedback_head_id_ = 0;      // RT-only: goal currently timed
  double feedback_head_elapsed_ = 0.0;      // RT-only: accumulated update() periods

  // --- online (jog) stream: RT-owned, dedicated (NOT a GoalEntry) --------------
  // An open-ended online stream must HOLD at its staged tail, not retire like the
  // action feed loop does on catch-up, so it gets its own cursor + trajectory rather
  // than reusing feeding_.
  std::shared_ptr<GoalTrajectory> online_traj_;  // live staged stream (finalize=false)
  int online_next_index_ = 0;                    // chunk cursor into online_traj_
  bool online_active_ = false;                   // an open online move is in progress
  bool online_stopping_ = false;                 // feeding the finalizing decel-to-rest tail
  // Which side of the feed-gate hysteresis band we are on (see the bounded-lookahead
  // comment by online_committed_horizon_): true = refilling (feed until the committed
  // depth reaches the high water), false = holding (wait for it to drain below the low
  // water). Starts true so a freshly adopted stream (depth 0) feeds immediately.
  bool online_refilling_ = true;
  // True once the current open move's FIRST chunk has been fed. The opening chunk
  // bypasses the horizon cap: the firmware arms its out-of-frames watchdog
  // (rapidcode_system kEmptyCount = 64 frames = 32 points) the instant a move opens,
  // and right after a reopen the reported depth still carries the PREVIOUS move's
  // draining frames -- capping the opening chunk by horizon space then yields a
  // handful of points and an instant starvation e-stop (seen live in teleop
  // stop/start churn). One full chunk transiently overshoots the band by design.
  bool online_opened_ = false;
  double online_since_snapshot_sec_ = 0.0;       // wall time since the last accepted snapshot
  // Raw pointer of the last snapshot consumed from rt_incoming_online_, so a stale
  // buffer re-read (RealtimeBuffer keeps returning the last value) is not mistaken for
  // fresh data. Compared by identity only; never dereferenced.
  const GoalTrajectory * online_last_seen_ = nullptr;
  // Committed end-of-stream (position, velocity) -- the seed BeginOnlineStopTail decels
  // from. Sized to joint count in on_activate.
  std::vector<double> online_committed_pos_;
  std::vector<double> online_committed_vel_;
  // Pre-allocated reusable stop-tail buffer + identity joint mapping, so synthesizing a
  // tail on the RT path refills in place instead of allocating. Built in on_activate.
  std::shared_ptr<GoalTrajectory> online_stop_traj_;
  std::vector<int> online_identity_mapping_;
  static constexpr int kStopTailReservePoints = 2048;
  // Splice working set (phase 3), sized in on_activate so the producer-rate splice
  // refills in place: the reusable blend+extension buffer SpliceOnlineSnapshot swaps in
  // as online_traj_ (safe to rebuild even when online_traj_ already points at it -- the
  // anchor lives in online_committed_pos_/vel_, never in the old payload), the clamped
  // velocity-intent scratch, and the all-zeros rest target the stop-tail blends toward.
  std::shared_ptr<GoalTrajectory> online_splice_traj_;
  std::vector<double> online_splice_vel_;
  std::vector<double> online_rest_vel_;
  // Constant-velocity extension past the splice blend, in seconds beyond
  // online.producer_timeout: the staged tail must outlive the silence window so the
  // firmware never starves between a short window's end and the stop-tail firing.
  static constexpr double kSpliceExtensionMargin = 0.02;

  // Executor<->RT arbitration mirrors (lock-free). update() (RT) publishes the pipeline
  // state each cycle; the callback reads action_busy_ to drop jog messages while an
  // action goal owns the mailbox, and HandleGoal reads online_busy_ to reject action
  // goals while a jog owns it. Reject, don't preempt.
  std::atomic<bool> action_busy_{false};
  std::atomic<bool> online_busy_{false};

  // --- operator fault acknowledgement (~/reset_fault) --------------------------
  // The service (executor thread) sets reset_requested_ after the operator has fixed
  // the drive manually (RapidSetupX ClearFaults); HandleHardwareFault (RT) consumes it
  // and emits Cmd::Reset so the hardware clears its latched error_code. update()
  // discards a stale request on any healthy cycle, so a request can never fire
  // outside the fault hold. hardware_error_code_ is the RT->executor mirror of the
  // error_code state interface (published each cycle) the service callback reads.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_fault_service_;
  std::atomic<bool> reset_requested_{false};
  std::atomic<int> hardware_error_code_{0};
  // Cycle-counted throttle for the fault-hold warning (a plain counter, not
  // RCLCPP_*_THROTTLE, so the RT branch stays clock-free). ~5 s at a 500 Hz update.
  int fault_warn_countdown_ = 0;
  static constexpr int kFaultHoldWarnPeriodCycles = 2500;

  // --- operator stop (~/stop) ---------------------------------------------------
  // The service (executor thread) latches stop_latched_ -- both intakes reject
  // immediately -- and sets stop_requested_ for update() (RT) to consume: abort every
  // pipeline goal, drop the open jog session, and emit Cmd::Stop so the hardware
  // decelerates the group to rest at the stop rate. The latch holds (goals and jog
  // input rejected, mailbox at Cmd::None) until ~/reset_fault clears it -- motion
  // never resumes from a stop without an explicit reset. Independent of the hardware
  // fault latch (error_code); one ~/reset_fault acknowledgement clears both.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> stop_latched_{false};
  // RT-only: a Cmd::Stop owed to the hardware (set when the request lands, cleared
  // when the mailbox accepts it -- retries while an un-acked command occupies it).
  bool stop_command_pending_ = false;

  // Admission rule 1: neither intake admits a trajectory while the
  // operator stop or a hardware fault is latched. Node-free so the fixture can call it;
  // returns the reason to append to the intake's own log line, or nullptr to admit.
  // hardware_error_code_ is the RT mirror, so it lags the state interface by at most
  // one cycle; update()'s fault branch remains the backstop for that cycle.
  const char * IntakeLatchReason() const;

  // Non-RT (callback) -> RT (update) latest-wins handoff of the newest jog snapshot.
  realtime_tools::RealtimeBuffer<std::shared_ptr<GoalTrajectory>> rt_incoming_online_;
  // An empty JointTrajectory on ~/joint_trajectory is the JTC's soft stop:
  // the callback sets this and ServiceOnlineStream() consumes it, starting the same
  // decel-to-rest stop-tail the producer timeout does. It supersedes any window staged
  // before it; consumed (and discarded) on the next cycle even when no stream is open.
  std::atomic<bool> online_stop_requested_{false};
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    joint_command_subscriber_;

  // --- executor <-> RT handoff -------------------------------------------------
  // Goals accepted on the executor thread wait here until update() (RT) drains them into
  // pending_. Plain mutex + try_lock keeps update() non-blocking (pushes are rare).
  std::mutex incoming_mutex_;
  std::deque<GoalEntry> incoming_;
  // Goals the action-monitor timer must flush (runNonRealtime) off the RT path. Touched
  // only by the executor (push) and the timer (flush/prune), never by update(); each
  // entry shares its `terminal` flag with the RT-side GoalEntry so the timer knows when
  // a result has been reported and the entry can be pruned. Depth also bounds acceptance.
  std::mutex monitored_mutex_;
  std::vector<MonitoredGoal> monitored_goals_;
  // Max goals accepted but not yet finished (backpressure at the action layer; the
  // hardware FIFO cap is the deeper backstop). HandleGoal rejects beyond this.
  static constexpr std::size_t kMaxInFlightGoals = 32;

  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  // Flushes the RealtimeServerGoalHandle's queued result/feedback off the RT path
  // (runNonRealtime), like JointTrajectoryController's goal_handle_timer_.
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;
  rclcpp::Duration action_monitor_period_ = rclcpp::Duration::from_seconds(0.02);

  // --- optional input-trajectory trace (opt-in via env RAPIDCODE_TRAJ_INPUT_CSV) ----
  // Dumps the raw FollowJointTrajectory waypoints (pre-interpolation), remapped into
  // joints_ order so it lines up column-for-column with the hardware MovePVT trace.
  // Opened in on_activate, written in HandleAccepted, closed in on_deactivate -- all
  // on the executor thread, so this never touches the RT update() path.
  std::FILE * input_csv_ = nullptr;
  long input_seq_ = 0;      // monotonic CSV row counter
  long input_goal_id_ = 0;  // increments per accepted goal, separating moves in one file
};

}  // namespace rapidcode_passthrough_trajectory_controller

#endif  // RAPIDCODE_PASSTHROUGH_TRAJECTORY_CONTROLLER__PASSTHROUGH_TRAJECTORY_CONTROLLER_HPP

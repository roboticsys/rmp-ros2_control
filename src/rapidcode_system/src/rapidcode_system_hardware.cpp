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

// RapidCode <-> ros2_control SystemInterface.
//
// The hardware half of the trajectory_transfer passthrough path. The lifecycle
// callbacks bring the rmp MotionController + one MultiAxis group up and down;
// write() consumes chunk commands from the single-slot mailbox into a host FIFO
// (excessChunks) and DrainQueue() feeds them to the firmware (one MovePVT per
// chunk, 2 frames per point); read() publishes joint telemetry, per-trajectory
// completion (frame accounting, NOT MotionDone) and the committed-depth gate
// input.

#include "rapidcode_system/rapidcode_system_hardware.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"

// RMP_INSTALL_PATH is injected as a compile definition by /rsi/cmake/RSI.cmake
// (from the rmp Debian package) through the RSI::RapidCode interface target.
// Guard it so this file still parses in an editor that lacks the define; any
// build that links RSI::RapidCode carries the real "/rsi" path.
#ifndef RMP_INSTALL_PATH
#define RMP_INSTALL_PATH ""
#endif

using RSI::RapidCode::Axis;
using RSI::RapidCode::MotionController;
using RSI::RapidCode::RapidCodeObject;
using RSI::RapidCode::RSIAction;
using RSI::RapidCode::RSIAxisAddressType;
using RSI::RapidCode::RSIMotorType;
using RSI::RapidCode::RSINetworkState;
using RSI::RapidCode::RSISource;
using RSI::RapidCode::RSIState;
using RSI::RapidCode::RsiError;

namespace
{
// Streaming-motion tuning for the chunked MovePVT feed (see FeedChunk/DrainQueue).
// emptyCount is the OUT_OF_FRAMES threshold passed to every MovePVT call: when the
// firmware's queued frame count falls below it while the move is still open, the
// firmware e-stops. It must stay below 20% of the axis frame buffer (default 1024).
// Each PVT point occupies 2 frames, so 64 frames is 32 points. The runway in
// seconds is 32 x the point duration and therefore depends on the trajectory, not
// on the control cycle: 5 ms points give 160 ms, 20 ms points give 640 ms. The
// intent is that this runway covers the axis EStopTime (see BindMultiAxisGroup),
// but the plugin does not check that at run time.
constexpr int32_t kEmptyCount = 64;  // frames (2 per point); < 20% of the 1024 buffer

// Backpressure metering (see DrainQueue/FreeFramesAvailable/AppendTrajectory).
// Extra frames kept free below the top of the firmware ring when feeding queued
// chunks. 0 means FreeFramesAvailable() trusts FramesToExecuteGet() exactly. Raise
// it if a stale read or firmware-added closing frames ever tip a feed into overflow.
constexpr int32_t kSafetyMargin = 0;  // frames
// Runaway backstop: with ack-on-enqueue nothing throttles the producer, so cap the
// host FIFO. A bounded goal never approaches this; hitting it means the firmware
// isn't draining, so we abort rather than grow host memory without bound.
constexpr std::size_t kMaxQueuedChunks = 4096;
// Nonzero error_code published when the FIFO overflows (controller aborts on != 0).
constexpr int kErrorQueueOverflow = 1;
// Nonzero error_code published when a std::exception escapes the feed path; the
// write() catch e-stops + clears the queue and latches this so the controller aborts.
constexpr int kErrorWriteFault = 2;
// Nonzero error_code published when DrainQueue sees the group already in an error
// state (a drive fault that did NOT throw): stop feeding, drop the queue, latch this.
constexpr int kErrorDriveFault = 3;

// Motion-profile recorder: record every N samples (recorder.cs uses 1 = full
// rate). At the default ~4096-record buffer that's ~4 s of history at 1 kHz;
// raise this for a longer window.
constexpr uint32_t kRecorderPeriodSamples = 1;

// Poll RapidCode diagnostic state from read() at a low rate. Error-log entries
// remain queued until ErrorLogGet(), so decimation should not lose entries.
constexpr int kRapidCodeDiagReadPeriod = 100;

// Diagnostic throttles (control-loop cycles). At the 500 Hz controller rate these are
// ~1 s and ~5 s. kDrainBlockedLogPeriod: how often to warn while DrainQueue is stalled
// (firmware buffer not draining). kWriteHeartbeatPeriod: write() liveness heartbeat.
constexpr int kDrainBlockedLogPeriod = 500;
constexpr int kWriteHeartbeatPeriod = 2500;

// Lower-case + interpret a URDF parameter string as a boolean.
bool ParseBool(const std::string & value, bool fallback)
{
  std::string lower;
  lower.reserve(value.size());
  for (const char chr : value)
  {
    lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(chr))));
  }
  if (lower == "true" || lower == "1" || lower == "yes") { return true; }
  if (lower == "false" || lower == "0" || lower == "no") { return false; }
  return fallback;
}

// Parse a URDF parameter string as a double; warn (via `logger`) and return the
// fallback on a malformed value.
double ParseDouble(const std::string & value, double fallback, const rclcpp::Logger & logger)
{
  try { return std::stod(value); }
  catch (const std::exception &)
  {
    RCLCPP_WARN(logger, "Could not parse '%s' as a number; using %g.", value.c_str(), fallback);
    return fallback;
  }
}

// Parse a URDF parameter string as an int; warn (via `logger`) and return the
// fallback on a malformed value.
int ParseInt(const std::string & value, int fallback, const rclcpp::Logger & logger)
{
  try { return std::stoi(value); }
  catch (const std::exception &)
  {
    RCLCPP_WARN(logger, "Could not parse '%s' as an integer; using %d.", value.c_str(), fallback);
    return fallback;
  }
}

// Look up `key` in a URDF parameter map, falling back to `fallback` when absent.
std::string ParamOr(const std::unordered_map<std::string, std::string> & parameters,
  const std::string & key, const std::string & fallback)
{
  const auto iter = parameters.find(key);
  return iter != parameters.end() ? iter->second : fallback;
}

// Phantom-axis setup mirroring SampleApps/cpp/src/config.h: a phantom axis has
// no real feedback, so neutralize every limit/fault action and set the motor
// type to PHANTOM (the axis then integrates commanded motion internally).
void ConfigurePhantomAxis(Axis * axis)
{
  axis->PositionSet(0);
  axis->ErrorLimitActionSet(RSIAction::RSIActionNONE);
  axis->AmpFaultActionSet(RSIAction::RSIActionNONE);
  axis->AmpFaultTriggerStateSet(true);
  axis->HardwareNegLimitActionSet(RSIAction::RSIActionNONE);
  axis->HardwarePosLimitActionSet(RSIAction::RSIActionNONE);
  axis->SoftwareNegLimitActionSet(RSIAction::RSIActionNONE);
  axis->SoftwarePosLimitActionSet(RSIAction::RSIActionNONE);
  axis->HomeActionSet(RSIAction::RSIActionNONE);

  // Settling tolerances near DBL_MAX so a phantom axis reports MotionDone
  // immediately on reaching target; backed off from the true max so rmp's XML
  // (de)serialization doesn't overflow (same trick as config.h).
  const double position_tolerance = std::numeric_limits<double>::max() / 10.0;
  axis->PositionToleranceCoarseSet(position_tolerance);
  axis->PositionToleranceFineSet(position_tolerance);

  axis->MotorTypeSet(RSIMotorType::RSIMotorTypePHANTOM);
}

// Format each axis's FramesToExecuteGet() into `buffer` as a comma-separated list
// (-1 for an unconfigured axis) -- the key drain signal shared by the DrainQueue
// stall warning and the MultiAxis diagnostics. Truncates safely at the buffer end.
void FormatFramesToExecute(const std::vector<Axis *> & axes, char * buffer, std::size_t size)
{
  int offset = 0;
  buffer[0] = '\0';
  for (std::size_t axis_index = 0; axis_index < axes.size(); ++axis_index)
  {
    offset += std::snprintf(buffer + offset, size - static_cast<std::size_t>(offset),
      "%s%d", axis_index ? "," : "",
      axes[axis_index] != nullptr ? axes[axis_index]->FramesToExecuteGet() : -1);
    if (offset >= static_cast<int>(size)) { break; }
  }
}

// Bring a (possibly dirty) axis to IDLE. The rmp firmware persists axis state
// across launches, so an axis can still be MOVING/STOPPED/ERROR from a previous
// run -- and PositionSet/ClearFaults reject the transient decel states. Abort to
// initiate a stop, wait for it to settle out of MOVING/STOPPING/STOPPING_ERROR,
// then ClearFaults to reach IDLE. Bounded so a stuck axis can't hang configure.
void ResetAxisToIdle(Axis * axis)
{
  if (axis->StateGet() == RSIState::RSIStateIDLE) { return; }
  axis->Abort();
  for (int attempt = 0; attempt < 300; ++attempt)  // up to ~3 s
  {
    const RSIState state = axis->StateGet();
    if (state == RSIState::RSIStateIDLE) { return; }
    if (state != RSIState::RSIStateMOVING &&
      state != RSIState::RSIStateSTOPPING &&
      state != RSIState::RSIStateSTOPPING_ERROR)
    {
      axis->ClearFaults();  // STOPPED / ERROR -> IDLE
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
}  // namespace

namespace rapidcode_system
{

RapidCodeSystemHardware::~RapidCodeSystemHardware()
{
  // Safety net: if we reach destruction with a live controller (the lifecycle
  // teardown didn't run), still stop the firmware and free it. Never throws.
  ReleaseController();
}

hardware_interface::CallbackReturn RapidCodeSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // Let the base class parse the URDF <ros2_control> block into info_.
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  AllocateJointStorage();
  ReadHardwareParameters();
  ReadJointParameters();

  if (useHardware && nicPrimary.empty())
  {
    RCLCPP_ERROR(logger,
      "on_init: hardware.primary_nic is empty. Set it to the EtherCAT NIC name in "
      "the hardware config (elfin5_hardware.yaml); find the name with `ip link`.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (cpuAffinity < 0)
  {
    RCLCPP_ERROR(logger,
      "on_init: hardware.cpu_affinity is %d. RMP needs one CPU core to run on; set it "
      "to an isolated core (never 0) in the hardware config (elfin5_hardware.yaml).",
      cpuAffinity);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger,
    "on_init: %zu joint(s), mode=%s, sample_rate=%.0f Hz, cpu_affinity=%d.",
    info_.joints.size(), useHardware ? "hardware" : "phantom", sampleRateHz, cpuAffinity);
  return hardware_interface::CallbackReturn::SUCCESS;
}

void RapidCodeSystemHardware::AllocateJointStorage()
{
  const std::size_t numJoints = info_.joints.size();
  axes.assign(numJoints, nullptr);
  axisIndices.assign(numJoints, 0);
  userUnits.assign(numJoints, 1.0);
  origins.assign(numJoints, 0.0);
  hasOrigin.assign(numJoints, false);
  ampEnableGroups.assign(numJoints, 0);
  errorLimits.assign(numJoints, 0.0);
  hasErrorLimit.assign(numJoints, false);
  statePositions.assign(numJoints, 0.0);
  stateVelocities.assign(numJoints, 0.0);

  xferSlotDuration.assign(rapidcode_trajectory_transfer::ChunkCapacity, 0.0);
  const std::size_t vectorSize = numJoints * rapidcode_trajectory_transfer::ChunkCapacity;
  xferSlotPosition.assign(vectorSize, 0.0);
  xferSlotVelocity.assign(vectorSize, 0.0);
  xferSlotAcceleration.assign(vectorSize, 0.0);
  xferSlotJerk.assign(vectorSize, 0.0);
}

void RapidCodeSystemHardware::ReadHardwareParameters()
{
  // Controller-wide parameters (URDF <ros2_control><hardware><param ...>). These
  // describe the robot statically, so they're parsed in on_init; the rmp
  // MotionController they configure isn't created until on_configure.
  const auto & params = info_.hardware_parameters;
  useHardware = ParseBool(ParamOr(params, "use_hardware", "false"), false);
  nicPrimary = ParamOr(params, "primary_nic", "");
  cpuAffinity = ParseInt(ParamOr(params, "cpu_affinity", "-1"), -1, logger);
  sampleRateHz = ParseDouble(ParamOr(params, "sample_rate", "1000.0"), 1000.0, logger);
  ampEnableDelaySeconds = ParseDouble(ParamOr(params, "amp_enable_delay", "0.0"), 0.0, logger);
  // Diagnostic frame logging is opt-in via the environment (no URDF edit / rebuild
  // to toggle): set RAPIDCODE_FRAME_DEBUG=1 in the controller_manager's env.
  debugFrames = (std::getenv("RAPIDCODE_FRAME_DEBUG") != nullptr);
}

void RapidCodeSystemHardware::ReadJointParameters()
{
  for (std::size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index)
  {
    const auto & joint = info_.joints[joint_index];
    // Default the axis index to the joint's ordinal so a single-joint URDF need
    // not specify it; user_units defaults to 1 (raw counts == user units).
    axisIndices[joint_index] = ParseInt(
      ParamOr(joint.parameters, "axis", std::to_string(joint_index)),
      static_cast<int>(joint_index), logger);
    userUnits[joint_index] =
      ParseDouble(ParamOr(joint.parameters, "user_units", "1.0"), 1.0, logger);
    ampEnableGroups[joint_index] =
      ParseInt(ParamOr(joint.parameters, "amp_enable_group", "0"), 0, logger);
    // origin / error_limit are optional: apply only when the URDF supplies them
    // (a missing origin must NOT zero the axis; a missing error_limit must leave
    // the drive default). Presence is tracked so a value of 0 is honored.
    if (joint.parameters.find("origin") != joint.parameters.end())
    {
      origins[joint_index] = ParseDouble(joint.parameters.at("origin"), 0.0, logger);
      hasOrigin[joint_index] = true;
    }
    if (joint.parameters.find("error_limit") != joint.parameters.end())
    {
      errorLimits[joint_index] = ParseDouble(joint.parameters.at("error_limit"), 0.0, logger);
      hasErrorLimit[joint_index] = true;
    }
  }
}

hardware_interface::CallbackReturn RapidCodeSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Re-entrant guard: on_cleanup releases the controller, so reaching here with
  // a live controller means we're already configured.
  if (controller != nullptr)
  {
    RCLCPP_WARN(logger, "on_configure: controller already created; skipping.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  if (std::string(RMP_INSTALL_PATH).empty())
  {
    RCLCPP_ERROR(logger,
      "RMP_INSTALL_PATH is empty -- the rmp package's RSI.cmake compile "
      "definition was not applied; cannot locate the RMP firmware.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // RapidCode throws (RsiError : std::exception) on error by default. Keep the
  // entire bring-up inside one try/catch and translate any failure into a clean
  // ERROR return -- an exception must never cross the ros2_control boundary.
  try
  {
    CreateController();
    SetupNetworkAndAxisCount();
    ConfigureAxes();
    BindMultiAxisGroup();
    ConfigureRecorder();
  }
  catch (const std::exception & error)
  {
    CleanupFailedConfigure(error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger,
    "on_configure: rmp controller ready (%s mode, %zu axis/axes, %.0f Hz).",
    useHardware ? "hardware" : "phantom", axes.size(), sampleRateHz);
  return hardware_interface::CallbackReturn::SUCCESS;
}

void RapidCodeSystemHardware::CreateController()
{
  MotionController::CreationParameters params;  // ctor zero-fills the buffers
  std::strncpy(params.RmpPath, RMP_INSTALL_PATH,
    MotionController::CreationParameters::PathLengthMaximum - 1);
  if (!nicPrimary.empty())
  {
    std::strncpy(params.NicPrimary, nicPrimary.c_str(),
      MotionController::CreationParameters::PathLengthMaximum - 1);
  }
  params.CpuAffinity = cpuAffinity;

  // Single rmp owner: this controller_manager process is the only caller of
  // Create(). It starts the RMP firmware if it isn't already running.
  controller = MotionController::Create(&params);
  if (controller == nullptr)
  {
    throw std::runtime_error("MotionController::Create returned null.");
  }
  // Create does not throw on licence, path, or firmware-start failures; it logs them.
  RequireCleanErrorLog("MotionController", controller);
  controller->ThrowExceptions(true);  // default; set explicitly for clarity

  // Sample rate must be set before NetworkStart (it has to match the ENI).
  controller->SampleRateSet(sampleRateHz);

  // The recorder count MUST be set before any other get/create (AxisGet /
  // AxisCountSet), per the SDK (SampleApps/csharp/recorder.cs). Allocate
  // recorder 0 now; its data addresses are configured in ConfigureRecorder, once
  // the axes exist. (Setting it after AxisGet segfaults the firmware.)
  controller->RecorderCountSet(1);
  recorderIndex = 0;
}

void RapidCodeSystemHardware::SetupNetworkAndAxisCount()
{
  const RSINetworkState net_state = controller->NetworkStateGet();
  if (useHardware)
  {
    if (net_state != RSINetworkState::RSINetworkStateOPERATIONAL)
    {
      controller->NetworkStart();
    }
    if (controller->NetworkStateGet() != RSINetworkState::RSINetworkStateOPERATIONAL)
    {
      // The start error is only a code; the network log carries the reason (wrong
      // ENI, bad NIC name, unpowered or unplugged nodes). Log every entry before
      // throwing.
      const int32_t startError = static_cast<int>(controller->LastNetworkStartErrorGet());
      const int32_t messageCount = controller->NetworkLogMessageCountGet();
      RCLCPP_ERROR(logger,
        "EtherCAT network start failed: state=%d, last start error %d, %d network log message(s):",
        static_cast<int>(controller->NetworkStateGet()), startError, messageCount);
      for (int32_t idx = 0; idx < messageCount; ++idx)
      {
        const char * const message = controller->NetworkLogMessageGet(idx);
        RCLCPP_ERROR(logger, "  network log[%d]: %s", idx, message != nullptr ? message : "(null)");
      }
      throw std::runtime_error(
        "EtherCAT network did not reach OPERATIONAL (last start error " +
        std::to_string(startError) + "; see the network log entries above).");
    }
  }
  else
  {
    // Phantom mode: the network must be down so the axis objects are ours to
    // shape. A live network here means hardware was left running.
    if (net_state != RSINetworkState::RSINetworkStateUNINITIALIZED &&
      net_state != RSINetworkState::RSINetworkStateSHUTDOWN)
    {
      throw std::runtime_error(
        "Phantom mode requested but the EtherCAT network is live; shut it "
        "down or set use_hardware:=true.");
    }
    int32_t required_axes = 0;
    for (const int32_t idx : axisIndices)
    {
      required_axes = std::max(required_axes, idx + 1);
    }
    if (controller->AxisCountGet() < required_axes)
    {
      controller->AxisCountSet(required_axes);
    }
  }

  // Size the motion-supervisor pool so a free supervisor exists ABOVE the axes
  // for the joint group. Done before AxisGet (matches the SDK order
  // AxisCountSet -> MotionCountSet -> AxisGet) so the axis handles fetched next
  // stay valid. MultiAxisGet/AxesAdd happen in BindMultiAxisGroup, once axes is
  // filled.
  controller->MotionCountSet(controller->AxisCountGet() + 1);

  // Seed with the first axis's buffer size; ConfigureAxes mins it down to the
  // most-conservative size across the group. (MotionController::
  // AxisFrameBufferSizeGet takes an axis number.)
  frameBufferSize = controller->AxisFrameBufferSizeGet(axisIndices[0]);
}

void RapidCodeSystemHardware::ConfigureAxes()
{
  // One Axis* per joint: fetch, configure, and seed its buffers.
  for (std::size_t joint_index = 0; joint_index < axisIndices.size(); ++joint_index)
  {
    Axis * axis = controller->AxisGet(axisIndices[joint_index]);
    if (axis == nullptr)
    {
      throw std::runtime_error(
        "AxisGet returned null for axis " + std::to_string(axisIndices[joint_index]) + ".");
    }
    RequireCleanErrorLog(info_.joints[joint_index].name.c_str(), axis);
    axis->MotionAttributeMaskOnSet(RSI::RapidCode::RSIMotionAttrMask::RSIMotionAttrMaskNO_WAIT);
    axis->ThrowExceptions(true);
    ResetAxisToIdle(axis);  // recover a dirty axis (firmware persists state)
    // Hardware calibration: set the axis origin in RAW ENCODER COUNTS. We
    // temporarily set UserUnits=1 so OriginPositionSet's argument is counts
    // (OriginPositionSet takes user units; rsi.h), then restore the joint's
    // real user units below, the same sequence the vendor's setup tool uses. Phantom
    // axes have no encoder, so their origin is meaningless (they zero via
    // ConfigurePhantomAxis/PositionSet(0)); only calibrate real hardware.
    if (useHardware && hasOrigin[joint_index])
    {
      axis->UserUnitsSet(1.0);
      axis->OriginPositionSet(origins[joint_index]);
    }
    axis->UserUnitsSet(userUnits[joint_index]);
    if (!useHardware)
    {
      ConfigurePhantomAxis(axis);
    }
    else if (hasErrorLimit[joint_index])
    {
      // Following-error safety limit (user units == radians here): EStop the
      // axis if command and actual diverge beyond this. Left at the drive/XML
      // default when the URDF omits error_limit.
      axis->ErrorLimitTriggerValueSet(std::copysign(errorLimits[joint_index], userUnits[joint_index]));
      axis->ErrorLimitActionSet(RSIAction::RSIActionE_STOP);
    }
    axes[joint_index] = axis;
    frameBufferSize = std::min(frameBufferSize, axis->FrameBufferSizeGet());

    // Seed command+state from the live position so the exported interfaces are
    // coherent while INACTIVE and the first write() won't command a jump. A
    // phantom reports CommandPosition (it has no actual feedback); hardware
    // reports the encoder. read() refreshes state each cycle; on_activate
    // re-seeds the command right before enabling amps.
    const double position =
      useHardware ? axis->ActualPositionGet() : axis->CommandPositionGet();
    statePositions[joint_index] = position;
    stateVelocities[joint_index] = 0.0;
  }
}

void RapidCodeSystemHardware::BindMultiAxisGroup()
{
  // Bind all joints into one MultiAxis on the free supervisor (== AxisCount, the
  // first index above the per-axis supervisors). AxisRemoveAll() drops any
  // membership the firmware persisted from a prior run before we (re)add ours.
  // write() then issues one MultiAxis::MovePVT per cycle for the whole group.
  multiAxis = controller->MultiAxisGet(controller->AxisCountGet());
  if (multiAxis == nullptr)
  {
    throw std::runtime_error("MultiAxisGet returned null for the joint group.");
  }
  RequireCleanErrorLog("MultiAxis", multiAxis);
  multiAxis->MotionAttributeMaskOnSet(RSI::RapidCode::RSIMotionAttrMask::RSIMotionAttrMaskNO_WAIT);
  multiAxis->ThrowExceptions(true);
  multiAxis->AxisRemoveAll();
  multiAxis->AxesAdd(axes.data(), static_cast<int32_t>(axes.size()));

  // Log the e-stop deceleration time next to the MovePVT emptyCount so an operator
  // can check that the runway (kEmptyCount / 2 points x point duration) covers it.
  // Point duration is not known here, so no automatic check is made.
  double e_stop_time = 0.0;
  for (Axis * axis : axes)
  {
    e_stop_time = std::max(e_stop_time, axis->EStopTimeGet());
  }
  RCLCPP_INFO(logger,
    "on_configure: EStopTime=%.4f s; MovePVT emptyCount=%d frames (%d points). Runway in "
    "seconds = points x point duration; keep it above EStopTime.",
    e_stop_time, kEmptyCount, kEmptyCount / 2);
}

void RapidCodeSystemHardware::ConfigureRecorder()
{
  // Configure recorder 0 to capture, per axis, command + actual position and
  // velocity. Mirrors SampleApps/csharp/recorder.cs (RecorderCountSet was done in
  // CreateController, before AxisGet). Record layout per axis: cmdPos, actPos,
  // cmdVel, actVel. We deliberately do NOT RecorderStart() -- start recording
  // yourself (RapidSetupX / WorkBench scope over rapidserver, or a RecorderStart
  // call), then read it back with RecorderRecordDataRetrieve +
  // RecorderRecordDataFirmwareValueGet(i).Double.
  constexpr int32_t kValuesPerAxis = 4;
  const int32_t values_per_record = static_cast<int32_t>(axes.size()) * kValuesPerAxis;

  if (controller->RecorderEnabledGet())  // stop a prior run before reconfiguring
  {
    controller->RecorderStop();
    controller->RecorderReset();
  }
  controller->RecorderPeriodSet(kRecorderPeriodSamples);
  controller->RecorderCircularBufferSet(false);
  controller->RecorderDataCountSet(values_per_record);
  int32_t slot = 0;
  for (Axis * axis : axes)
  {
    controller->RecorderDataAddressSet(slot++,
      axis->AddressGet(RSIAxisAddressType::RSIAxisAddressTypeCOMMAND_POSITION));
    controller->RecorderDataAddressSet(slot++,
      axis->AddressGet(RSIAxisAddressType::RSIAxisAddressTypeACTUAL_POSITION));
    controller->RecorderDataAddressSet(slot++,
      axis->AddressGet(RSIAxisAddressType::RSIAxisAddressTypeCOMMAND_VELOCITY));
    controller->RecorderDataAddressSet(slot++,
      axis->AddressGet(RSIAxisAddressType::RSIAxisAddressTypeACTUAL_VELOCITY));
  }
  RCLCPP_INFO(logger,
    "on_configure: recorder 0 configured (%d values/record, period %u sample(s)); "
    "NOT started -- start recording manually.",
    values_per_record, kRecorderPeriodSamples);
}

void RapidCodeSystemHardware::CleanupFailedConfigure(const char * what)
{
  RCLCPP_ERROR(logger, "on_configure failed: %s", what);
  // Leave nothing half-created: drop cleanly back to UNCONFIGURED so the
  // operator can fix the environment and retry configure. Delete() can itself
  // throw on a bad controller -- swallow it so the exception never escapes
  // on_configure (which would std::terminate the controller_manager).
  if (controller != nullptr)
  {
    try { controller->Delete(); }
    catch (const std::exception &) { /* best effort cleanup */ }
    controller = nullptr;
  }
  multiAxis = nullptr;  // owned by controller; invalid once it's deleted
  std::fill(axes.begin(), axes.end(), nullptr);
}

hardware_interface::CallbackReturn RapidCodeSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (controller == nullptr)
  {
    RCLCPP_ERROR(logger, "on_activate: controller not configured.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  OpenPvtTrace();

  // Amp enable is INTENTIONALLY mode-dependent.
  //
  // Phantom: the axes are simulated, so enabling them here is harmless and lets
  // the demos run unattended.
  //
  // Hardware: this plugin does NOT enable the physical amps. Powering a drive is
  // a deliberate operator action, taken with the E-stop in reach and the cell
  // clear, and it must not be a side effect of a ros2_control lifecycle
  // transition that a launch file or `ros2 control` command can trigger without
  // anyone looking at the robot. The operator enables the amps out of band
  // (RapidSetupX / WorkBench, or a RapidCode script) after the component is
  // ACTIVE and before the first trajectory. Until then the PVT stream is armed
  // but the drives will not move (see RUNBOOK section 7).
  //
  // The per-joint `group` and controller-wide `amp_enable_delay` parameters are
  // parsed for that out-of-band procedure's documentation but are not acted on
  // here.
  if (!useHardware && multiAxis != nullptr)
  {
    multiAxis->ClearFaults();
    multiAxis->AmpEnableSet(true);
  }

  // Fresh activation: re-arm the reopen latch and resync completion counters to a
  // known base so the controller's monotonic trajectory ids line up from id 0.
  firstChunk = true;
  ResetCompletionTracking();

  if (useHardware)
  {
    RCLCPP_WARN(logger,
      "on_activate: PVT stream armed (%zu axis/axes). Physical amps are NOT enabled by "
      "this plugin; enable them out of band (RapidSetupX / WorkBench) before commanding motion.",
      axes.size());
  }
  else
  {
    RCLCPP_INFO(logger, "on_activate: phantom amps enabled, PVT stream armed (%zu axis/axes).",
      axes.size());
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}


void RapidCodeSystemHardware::OpenPvtTrace()
{
  // Optional MovePVT input-trace CSV (env RAPIDCODE_PVT_CSV=<path>). Opened at
  // activation, once axes are known, so the header can name a column group per
  // joint. Full buffering keeps the per-cycle fprintf to a memcpy on the control
  // thread; flushed on close (ReleaseController). Re-activation reuses the handle.
  if (pvtCsv != nullptr) { return; }
  const char * csv_path = std::getenv("RAPIDCODE_PVT_CSV");
  if (csv_path == nullptr || csv_path[0] == '\0') { return; }
  pvtCsv = std::fopen(csv_path, "w");
  if (pvtCsv == nullptr)
  {
    RCLCPP_WARN(logger, "on_activate: could not open RAPIDCODE_PVT_CSV='%s'", csv_path);
    return;
  }
  std::setvbuf(pvtCsv, nullptr, _IOFBF, 1 << 20);  // 1 MiB, off the hot path
  pvtSeq = 0;
  pvtChunkId = 0;
  pvtCsvTime = 0.0;
  std::fprintf(pvtCsv, "seq,tag,t_cum,dt,final,chunk_id,chunk_idx,chunk_count");
  for (std::size_t axis_index = 0; axis_index < axes.size(); ++axis_index)
  {
    std::fprintf(pvtCsv, ",j%zu_pos,j%zu_vel,j%zu_acc,j%zu_jrk",
      axis_index, axis_index, axis_index, axis_index);
  }
  std::fprintf(pvtCsv, "\n");
  RCLCPP_INFO(logger, "on_activate: tracing MovePVT inputs to %s", csv_path);
}

hardware_interface::CallbackReturn RapidCodeSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (controller == nullptr)
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  try
  {
    if (multiAxis != nullptr)
    {
      multiAxis->Abort();  // halt the whole group's streamed motion immediately
    }
    excessChunks.clear();  // drop any queued tail so a re-activate starts clean
    pendingFinal = false;
    transferMoveOpen = false;
    firstChunk = true;     // move aborted/closed -> the next feed must reopen it
    ResetCompletionTracking();
    for (Axis * axis : axes)
    {
      if (axis == nullptr) { continue; }
      axis->AmpEnableSet(false);  // disable the (phantom) amp (per-axis)
    }
  }
  catch (const std::exception & error)
  {
    RCLCPP_WARN(logger, "on_deactivate: %s", error.what());
  }
  RCLCPP_INFO(logger, "on_deactivate: motion aborted, amps disabled.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RapidCodeSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Tear the rmp down completely (stop the firmware + free the controller). A
  // later on_configure re-Creates it (Create restarts the rmp if it isn't up),
  // so the component is still reconfigurable -- it just starts from a clean
  // firmware each time instead of leaving an orphaned rmp + shared memory.
  ReleaseController();
  RCLCPP_INFO(logger, "on_cleanup: rmp shut down and controller released.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void RapidCodeSystemHardware::ReleaseController() noexcept
{
  // Flush + close the MovePVT trace first (independent of the controller, and it
  // holds buffered rows that must reach disk on teardown).
  if (pvtCsv != nullptr)
  {
    std::fclose(pvtCsv);
    pvtCsv = nullptr;
  }
  if (controller == nullptr)
  {
    return;
  }
  // Best-effort quiesce before stopping the firmware: stop the recorder, halt
  // and unmap the group, and (hardware) bring the EtherCAT network down. Each
  // step is independent so one failure doesn't skip the rest.
  try
  {
    if (recorderIndex >= 0)
    {
      if (controller->RecorderEnabledGet()) { controller->RecorderStop(); }
      recorderIndex = -1;
    }
  }
  catch (const std::exception & error) { RCLCPP_WARN(logger, "ReleaseController: recorder: %s", error.what()); }
  try
  {
    if (multiAxis != nullptr)
    {
      multiAxis->Abort();          // halt any streamed motion
      multiAxis->AxisRemoveAll();  // unmap the supervisor
    }
  }
  catch (const std::exception & error) { RCLCPP_WARN(logger, "ReleaseController: group halt: %s", error.what()); }
  try
  {
    if (useHardware) { controller->NetworkShutdown(); }
  }
  catch (const std::exception & error) { RCLCPP_WARN(logger, "ReleaseController: network: %s", error.what()); }

  // Stop the RMP firmware process, then free all RapidCode objects. Order is
  // mandated by the API (rsi.h: the only valid call after Shutdown is Delete).
  // Guard each so a throw in one still runs the other and never escapes
  // (noexcept -- this runs from the destructor).
  try { controller->Shutdown(); }
  catch (const std::exception & error) { RCLCPP_WARN(logger, "ReleaseController: Shutdown: %s", error.what()); }
  try { controller->Delete(); }
  catch (const std::exception & error) { RCLCPP_WARN(logger, "ReleaseController: Delete: %s", error.what()); }

  controller = nullptr;
  multiAxis = nullptr;  // owned by controller; invalid once it's released
  std::fill(axes.begin(), axes.end(), nullptr);
}

void RapidCodeSystemHardware::LogFrameDebug(const char * tag)
{
  if (!debugFrames || controller == nullptr || multiAxis == nullptr) { return; }
  try
  {
    std::string line;
    char buf[160];
    for (std::size_t axis_index = 0; axis_index < axes.size(); ++axis_index)
    {
      Axis * axis = axes[axis_index];
      if (axis == nullptr) { continue; }
      using AT = RSIAxisAddressType;
      // FRAME_LOAD_INDEX = last frame the API loaded; FRAME_INDEX = frame the
      // firmware is executing. If LOAD advances but INDEX doesn't, the frames are
      // queued but not consumed; if LOAD doesn't advance for an axis, that axis is
      // not being fed at all (the >=2-axis symptom).
      const int32_t frameIdx = controller->MemoryGet(axis->AddressGet(AT::RSIAxisAddressTypeFRAME_INDEX));
      const int32_t loadIdx = controller->MemoryGet(axis->AddressGet(AT::RSIAxisAddressTypeFRAME_LOAD_INDEX));
      const int32_t status = controller->MemoryGet(axis->AddressGet(AT::RSIAxisAddressTypeSTATUS));
      const double feedrate = controller->MemoryDoubleGet(axis->AddressGet(AT::RSIAxisAddressTypeCURRENT_FEEDRATE));
      std::snprintf(buf, sizeof(buf), " ax%zu[st=%d frame=%d/load=%d status=0x%X fr=%.2f cmd=%.5f]",
        axis_index, static_cast<int>(axis->StateGet()), frameIdx, loadIdx,
        static_cast<unsigned>(status), feedrate, axis->CommandPositionGet());
      line += buf;
    }
    RCLCPP_INFO(logger, "frames[%s grp_state=%d]%s",
      tag, static_cast<int>(multiAxis->StateGet()), line.c_str());
  }
  catch (const std::exception & error)
  {
    RCLCPP_WARN(logger, "LogFrameDebug: %s", error.what());
  }
}

bool RapidCodeSystemHardware::DrainRapidCodeErrorLog(
  const char * label, RapidCodeObject * object, bool * hadError)
{
  if (hadError != nullptr) { *hadError = false; }
  if (object == nullptr) { return false; }

  bool logged = false;
  try
  {
    const int32_t count = object->ErrorLogCountGet();
    for (int32_t idx = 0; idx < count; ++idx)
    {
      const RsiError * const error = object->ErrorLogGet();
      if (error == nullptr) { break; }

      logged = true;
      if (error->isWarning)
      {
        RCLCPP_WARN(logger,
          "RapidCode %s warning log: number=%d object=%d function='%s' file='%s:%d' "
          "short='%s' text='%s'",
          label, static_cast<int>(error->number), error->objectIndex,
          error->functionName, error->fileName, error->lineNumber,
          error->shortText, error->text);
      }
      else
      {
        if (hadError != nullptr) { *hadError = true; }
        RCLCPP_ERROR(logger,
          "RapidCode %s error log: number=%d object=%d function='%s' file='%s:%d' "
          "short='%s' text='%s'",
          label, static_cast<int>(error->number), error->objectIndex,
          error->functionName, error->fileName, error->lineNumber,
          error->shortText, error->text);
      }
    }
  }
  catch (const std::exception & error)
  {
    RCLCPP_WARN(logger, "RapidCode %s error-log read failed: %s", label, error.what());
  }

  return logged;
}

void RapidCodeSystemHardware::RequireCleanErrorLog(const char * label, RapidCodeObject * object)
{
  bool hadError = false;
  DrainRapidCodeErrorLog(label, object, &hadError);
  if (hadError)
  {
    throw std::runtime_error(
      std::string(label) + ": RapidCode reported errors during creation (see the log above).");
  }
}

void RapidCodeSystemHardware::LogMultiAxisDiagnostics(const char * context, bool force)
{
  if (multiAxis == nullptr) { return; }

  const bool loggedError = DrainRapidCodeErrorLog("MultiAxis", multiAxis);
  try
  {
    const RSIState state = multiAxis->StateGet();
    const uint64_t statusBits = multiAxis->StatusBitsGet();
    const bool changed = !haveLastMultiAxisDiag ||
      state != lastMultiAxisDiagState || statusBits != lastMultiAxisDiagStatusBits;
    const bool abnormal = force || loggedError ||
      state == RSIState::RSIStateERROR ||
      state == RSIState::RSIStateSTOPPED ||
      state == RSIState::RSIStateSTOPPING_ERROR;

    if (abnormal || changed)
    {
      const RSISource source = multiAxis->SourceGet();
      const char * const sourceName = multiAxis->SourceNameGet(source);
      const bool motionDone = multiAxis->MotionDoneGet();
      // Per-axis frames-to-execute: the key drain signal. If these stay pinned near
      // the frame-buffer size while the group reads MOVING, the firmware isn't
      // draining the queued frames (a multi-axis streaming stall looks like this).
      char fteBuf[128];
      FormatFramesToExecute(axes, fteBuf, sizeof(fteBuf));
      if (abnormal)
      {
        RCLCPP_WARN(logger,
          "RapidCode MultiAxis diagnostics[%s]: state=%d source=%d('%s') "
          "status_bits=0x%llX motion_done=%d fte=[%s]",
          context, static_cast<int>(state), static_cast<int>(source),
          sourceName != nullptr ? sourceName : "", static_cast<unsigned long long>(statusBits),
          motionDone ? 1 : 0, fteBuf);
      }
      else
      {
        RCLCPP_INFO(logger,
          "RapidCode MultiAxis diagnostics[%s]: state=%d source=%d('%s') "
          "status_bits=0x%llX motion_done=%d fte=[%s]",
          context, static_cast<int>(state), static_cast<int>(source),
          sourceName != nullptr ? sourceName : "", static_cast<unsigned long long>(statusBits),
          motionDone ? 1 : 0, fteBuf);
      }
    }

    haveLastMultiAxisDiag = true;
    lastMultiAxisDiagState = state;
    lastMultiAxisDiagStatusBits = statusBits;
  }
  catch (const std::exception & error)
  {
    RCLCPP_WARN(logger, "RapidCode MultiAxis diagnostics[%s] failed: %s", context, error.what());
  }
}

void RapidCodeSystemHardware::LogPvtPoints(
  const char * tag, const double * pos, const double * vel, const double * acc,
  const double * jrk, const double * times, int point_count, bool final_flag, long chunk_id)
{
  if (pvtCsv == nullptr)
  {
    return;
  }
  // Arrays are point-major / axis-minor: value for point p, axis i is at [p*n + i].
  // One CSV row per point; t_cum accumulates each point's dt so the trace shares the
  // recorder's firmware-time axis. final_flag is the whole call's flag (the streamed
  // point has point_count==1; a streamed chunk is many points, all final=0). chunk_id
  // is constant across the call's rows; chunk_idx (p) and chunk_count locate each row
  // within its MovePVT chunk.
  const std::size_t axis_count = axes.size();
  for (int point = 0; point < point_count; ++point)
  {
    pvtCsvTime += times[point];
    std::fprintf(pvtCsv, "%ld,%s,%.6f,%.6f,%d,%ld,%d,%d",
      pvtSeq++, tag, pvtCsvTime, times[point], final_flag ? 1 : 0, chunk_id, point, point_count);
    for (std::size_t axis_index = 0; axis_index < axis_count; ++axis_index)
    {
      const std::size_t idx = static_cast<std::size_t>(point) * axis_count + axis_index;
      std::fprintf(pvtCsv, ",%.9f,%.9f,%.9f,%.9f",
        pos[idx], vel[idx], acc[idx], jrk[idx]);
    }
    std::fprintf(pvtCsv, "\n");
  }
  // Flush every record so a mid-move stall never leaves a torn final row on disk (a
  // partial last line previously read like a crash). This is a diagnostic-only CSV
  // (opened only when RAPIDCODE_PVT_CSV is set), so the extra fflush cost is fine.
  std::fflush(pvtCsv);
}

hardware_interface::CallbackReturn RapidCodeSystemHardware::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  // Same teardown as cleanup; on_cleanup is a no-op if already released.
  return on_cleanup(previous_state);
}

hardware_interface::CallbackReturn RapidCodeSystemHardware::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Safe-state the axes on any lifecycle error. Returning SUCCESS drops the
  // component to UNCONFIGURED (recoverable) rather than FINALIZED.
  for (Axis * axis : axes)
  {
    if (axis == nullptr) { continue; }
    try { axis->Abort(); }
    catch (const std::exception &) { /* best effort; already in error */ }
  }
  RCLCPP_ERROR(logger, "on_error: axes aborted.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RapidCodeSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> stateInterfaces;
  const std::string gpioPrefix = rapidcode_trajectory_transfer::DefaultGpioName;
  stateInterfaces.emplace_back(gpioPrefix, rapidcode_trajectory_transfer::interface_names::IsMoving, &xferIsMoving);
  stateInterfaces.emplace_back(gpioPrefix, rapidcode_trajectory_transfer::interface_names::AckSequence, &xferAckSequence);
  stateInterfaces.emplace_back(gpioPrefix, rapidcode_trajectory_transfer::interface_names::AcceptedPointIndex, &xferAcceptedPointIndex);
  stateInterfaces.emplace_back(gpioPrefix, rapidcode_trajectory_transfer::interface_names::ErrorCode, &xferErrorCode);
  stateInterfaces.emplace_back(gpioPrefix, rapidcode_trajectory_transfer::interface_names::CompletedTrajectoryId, &xferCompletedTrajectoryId);
  stateInterfaces.emplace_back(gpioPrefix, rapidcode_trajectory_transfer::interface_names::CommittedDepthPoints, &xferCommittedDepthPoints);

  for (std::size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index)
  {
    stateInterfaces.emplace_back(
      info_.joints[joint_index].name, hardware_interface::HW_IF_POSITION, &statePositions[joint_index]);
    stateInterfaces.emplace_back(
      info_.joints[joint_index].name, hardware_interface::HW_IF_VELOCITY, &stateVelocities[joint_index]);
  }
  return stateInterfaces;
}

std::vector<hardware_interface::CommandInterface>
RapidCodeSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> commandInterfaces;
  const std::string gpioPrefix = rapidcode_trajectory_transfer::DefaultGpioName;
  const std::size_t numChunks = rapidcode_trajectory_transfer::ChunkCapacity;

  // scalar command interfaces
  commandInterfaces.emplace_back(gpioPrefix, rapidcode_trajectory_transfer::interface_names::CommandToken, &xferCommandToken);
  commandInterfaces.emplace_back(gpioPrefix, rapidcode_trajectory_transfer::interface_names::CommandSequence, &xferCommandSequence);
  commandInterfaces.emplace_back(gpioPrefix, rapidcode_trajectory_transfer::interface_names::TrajectoryId, &xferTrajectoryId);
  commandInterfaces.emplace_back(gpioPrefix, rapidcode_trajectory_transfer::interface_names::TrajectorySize, &xferTrajectorySize);
  commandInterfaces.emplace_back(gpioPrefix, rapidcode_trajectory_transfer::interface_names::ValidFields, &xferValidFields);
  commandInterfaces.emplace_back(gpioPrefix, rapidcode_trajectory_transfer::interface_names::ChunkBaseIndex, &xferChunkBaseIndex);
  commandInterfaces.emplace_back(gpioPrefix, rapidcode_trajectory_transfer::interface_names::ChunkLen, &xferChunkLen);
  commandInterfaces.emplace_back(gpioPrefix, rapidcode_trajectory_transfer::interface_names::ChunkFinal, &xferChunkFinal);

  // Array command interfaces. Requires one scalar interface per slot.
  const std::size_t numJoints = info_.joints.size();
  for (std::size_t slot = 0; slot < numChunks; ++slot)
  {
    commandInterfaces.emplace_back(gpioPrefix, rapidcode_trajectory_transfer::SlotDuration(slot), &xferSlotDuration[slot]);
    for (std::size_t joint = 0; joint < numJoints; ++joint)
    {
      const std::size_t jointSlot = slot * numJoints + joint;
      commandInterfaces.emplace_back(gpioPrefix, rapidcode_trajectory_transfer::JointSlotPosition(joint, slot), &xferSlotPosition[jointSlot]);
      commandInterfaces.emplace_back(gpioPrefix, rapidcode_trajectory_transfer::JointSlotVelocity(joint, slot), &xferSlotVelocity[jointSlot]);
      commandInterfaces.emplace_back(gpioPrefix, rapidcode_trajectory_transfer::JointSlotAcceleration(joint, slot), &xferSlotAcceleration[jointSlot]);
      commandInterfaces.emplace_back(gpioPrefix, rapidcode_trajectory_transfer::JointSlotJerk(joint, slot), &xferSlotJerk[jointSlot]);
    }
  }

  return commandInterfaces;
}

hardware_interface::return_type RapidCodeSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Pull telemetry into the state buffers. Exception-free, and non-fatal: on error
  // keep the last values and flag for recovery rather than returning ERROR (which
  // would deactivate the hardware).
  try
  {
    ReadJointTelemetry();
    PublishTransferTelemetry();
  }
  catch (const std::exception & error)
  {
    RCLCPP_ERROR(logger, "read() fault: %s.", error.what());
    LogMultiAxisDiagnostics("read exception", true);
  }
  return hardware_interface::return_type::OK;
}

void RapidCodeSystemHardware::ReadJointTelemetry()
{
  // A phantom axis has no real feedback ("no actual position"), so its
  // CommandPosition IS its position; a hardware axis reports the real encoder via
  // ActualPosition.
  for (std::size_t axis_index = 0; axis_index < axes.size(); ++axis_index)
  {
    if (axes[axis_index] == nullptr) { continue; }  // not yet configured (e.g. hermetic unit tests)
    if (useHardware)
    {
      statePositions[axis_index] = axes[axis_index]->ActualPositionGet();
      stateVelocities[axis_index] = axes[axis_index]->ActualVelocityGet();
    }
    else
    {
      statePositions[axis_index] = axes[axis_index]->CommandPositionGet();
      stateVelocities[axis_index] = axes[axis_index]->CommandVelocityGet();
    }
  }
}

void RapidCodeSystemHardware::PublishTransferTelemetry()
{
  if (multiAxis != nullptr)
  {
    // Still "moving" if the firmware is executing OR points are queued/awaited:
    // with the host FIFO, MotionDoneGet() can read true between feeds while the
    // tail is still queued, so completion must also account for the queue and the
    // not-yet-fed final chunk (else the controller finishes the goal early).
    xferIsMoving = rapidcode_trajectory_transfer::Encode(
      !multiAxis->MotionDoneGet() || !excessChunks.empty() || pendingFinal);
    if (++rapidCodeDiagReadCycles >= kRapidCodeDiagReadPeriod)
    {
      rapidCodeDiagReadCycles = 0;
      LogMultiAxisDiagnostics("read", false);
    }
  }

  // Per-trajectory completion: publish the highest trajectory id the firmware has
  // fully executed. Not gated on multiAxis so the no-rmp unit tests exercise it.
  UpdateCompletion();

  // Committed depth for the controller's online feed gate (points accepted but not
  // executed: firmware pending + host FIFO). Not gated on multiAxis either --
  // FramesRemaining() is null-safe, and the no-rmp tests still see the FIFO part.
  xferCommittedDepthPoints = rapidcode_trajectory_transfer::Encode(CommittedDepthPoints());
}

void RapidCodeSystemHardware::BeginTrajectoryTransfer()
{
  transferTrajectorySize = rapidcode_trajectory_transfer::Decode(xferTrajectorySize);
  transferValidFields = rapidcode_trajectory_transfer::Decode(xferValidFields);
  currentTrajectoryId = rapidcode_trajectory_transfer::Decode<std::uint64_t>(xferTrajectoryId);
  transferMoveOpen = true;
  pendingFinal = true;     // a final chunk is expected before the move completes

  // Multi-trajectory streaming: a BEGIN may arrive while a previous trajectory's
  // chunks are still queued (pipelined goals), so we must NOT clear excessChunks here
  // -- the FIFO is cleared only on an explicit Reset/Abort or an error. We also must
  // NOT clear error_code here: a latched fault survives until an explicit Reset
  // acknowledges it. Likewise firstChunk is driven by the move lifecycle (FeedChunk),
  // not by BEGIN, so a BEGIN on a still-open move never re-toggles the reopen latch.

  // Record this trajectory's cumulative expected-frame threshold (2 frames per point)
  // for per-trajectory completion. A non-positive size is an open/streaming transfer
  // with no fixed end, so it gets no completion threshold.
  if (transferTrajectorySize > 0)
  {
    cumulativeExpectedFrames += 2ull * static_cast<std::uint64_t>(transferTrajectorySize);
    trajectoryCompletions.push_back({currentTrajectoryId, cumulativeExpectedFrames});
  }

  xferAcceptedPointIndex = rapidcode_trajectory_transfer::Encode(-1);  // no points accepted yet
}

int32_t RapidCodeSystemHardware::NeededFrames(const TrajectoryChunk & chunk) const
{
  return 2 * static_cast<int32_t>(chunk.Count);  // 2 frames per point (MovePVT docs, rsi.h)
}

int32_t RapidCodeSystemHardware::FreeFramesAvailable() const
{
  // Most-conservative free space across the group: the slowest-draining axis bounds
  // how much we can safely feed. Skip unconfigured (null) axes so this is callable
  // from the unit tests, where no rmp axes exist. kSafetyMargin keeps headroom
  // below the top of the ring.
  int32_t freeMin = std::numeric_limits<int32_t>::max();
  for (Axis * axis : axes)  // non-const: RapidCode getters (FramesToExecuteGet) are non-const
  {
    if (axis == nullptr) { continue; }
    freeMin = std::min(freeMin, frameBufferSize - axis->FramesToExecuteGet() - kSafetyMargin);
  }
  if (freeMin == std::numeric_limits<int32_t>::max()) { return 0; }  // no live axes
  return std::max(0, freeMin);
}

int32_t RapidCodeSystemHardware::FramesRemaining() const
{
  // MAX frames-to-execute across the group: the slowest-draining axis bounds how far
  // the whole group has actually progressed, so completion accounting stays
  // conservative (reports a trajectory finished late, never early). Null-safe.
  int32_t maxRemaining = 0;
  for (Axis * axis : axes)  // non-const: FramesToExecuteGet is a non-const getter
  {
    if (axis == nullptr) { continue; }
    maxRemaining = std::max(maxRemaining, axis->FramesToExecuteGet());
  }
  return maxRemaining;
}

int32_t RapidCodeSystemHardware::CommittedDepthPoints() const
{
  // Firmware side: FramesRemaining() is in frames (2 per point); ceil-divide so a
  // firmware-added odd closing frame still counts as a full pending point (the depth
  // reads slightly high, never low -- conservative for a gate that bounds lookahead).
  const int32_t firmwarePoints = (FramesRemaining() + 1) / 2;
  // Host side: points acked into the FIFO but not yet fed to the firmware.
  int32_t queuedPoints = 0;
  for (const TrajectoryChunk & chunk : excessChunks)
  {
    queuedPoints += static_cast<int32_t>(chunk.Count);
  }
  return firmwarePoints + queuedPoints;
}

void RapidCodeSystemHardware::FeedChunk(const TrajectoryChunk & chunk)
{
  // The firmware call is guarded so the queue bookkeeping (and the CSV trace) stay
  // exercised even without a live MultiAxis. In production multiAxis is always set.
  if (multiAxis != nullptr)
  {
    // disable NO_WAIT on the first chunk to ensure motion starts
    if (firstChunk)
    {
      multiAxis->MotionAttributeMaskOffSet(RSI::RapidCode::RSIMotionAttrMask::RSIMotionAttrMaskNO_WAIT);
    }
    multiAxis->MovePVT(chunk.Positions.data(), chunk.Velocities.data(), chunk.Durations.data(),
      static_cast<int32_t>(chunk.Count), kEmptyCount, /*retain=*/false, /*final=*/chunk.IsFinal);

    if (firstChunk)
    {
      multiAxis->MotionAttributeMaskOnSet(RSI::RapidCode::RSIMotionAttrMask::RSIMotionAttrMaskNO_WAIT);
    }
    LogMultiAxisDiagnostics(chunk.IsFinal ? "MovePVT final chunk" : "MovePVT chunk", false);
  }
  // The opening chunk has now been fed (whether or not a live MultiAxis processed it),
  // so the firmware move is open; an IsFinal chunk below re-arms the reopen latch.
  firstChunk = false;
  LogPvtPoints(chunk.IsFinal ? "final" : "stream", chunk.Positions.data(), chunk.Velocities.data(),
    chunk.Accelerations.data(), chunk.Jerks.data(), chunk.Durations.data(),
    static_cast<int>(chunk.Count), chunk.IsFinal, pvtChunkId++);
  if (chunk.IsFinal)
  {
    pendingFinal = false;
    firstChunk = true;  // final chunk closes the firmware move; the next feed reopens it
    RCLCPP_INFO(logger, "FeedChunk: FINAL chunk fed to firmware (count=%zu).", chunk.Count);
  }
}

void RapidCodeSystemHardware::DrainQueue()
{
  // Never feed into a faulted or stopped group. A drive fault (following-error trip,
  // amp fault, an EStop from RapidSetup) drops the MultiAxis into an error state
  // WITHOUT throwing in write(), so HandleWriteFault() never sees it -- clearing the
  // queue only there is not enough. Check the group state here, every cycle, first.
  // (A StateGet() throw propagates to write()'s catch -> HandleWriteFault, which is
  // the correct handling.)
  const auto getGroupState = [this]()
  {
    return (multiAxis != nullptr) ? multiAxis->StateGet() : RSIState::RSIStateIDLE;
  };
  const RSIState groupState = getGroupState();
  if (groupState == RSIState::RSIStateERROR || groupState == RSIState::RSIStateSTOPPING_ERROR)
  {
    // Hard fault. Stop feeding, drop the tail, latch so the controller aborts -- once
    // per fault (after the queue/FSM is clear this is a no-op, so it won't spam). No
    // EStop here: the group is already in an error/stopping state.
    if (!excessChunks.empty() || pendingFinal || transferMoveOpen)
    {
      AbortTransfer("DrainQueue: MultiAxis in error state", /*eStop=*/false, kErrorDriveFault);
      return;
    }
    // A Reset that raced the commanded e-stop's deceleration (HandleStop) owes the
    // self-inflicted ERROR -> IDLE ClearFaults; finish it once the group settles.
    if (stopRecoverPending && commandedStopFault && groupState == RSIState::RSIStateERROR)
    {
      stopRecoverPending = false;
      commandedStopFault = false;
      multiAxis->ClearFaults();
    }
    return;
  }
  if (groupState == RSIState::RSIStateSTOPPING || groupState == RSIState::RSIStateSTOPPED)
  {
    // A Reset that raced the deceleration owes the STOPPED->IDLE ClearFaults; finish
    // it here once the group settles so the re-arm doesn't need a second Reset.
    if (stopRecoverPending && groupState == RSIState::RSIStateSTOPPED)
    {
      stopRecoverPending = false;
      multiAxis->ClearFaults();
      return;  // IDLE next cycle; feed then
    }
    return;  // decelerating / stopped: don't feed; wait for IDLE or an explicit error
  }

  // Feed front-to-back while the head chunk fits, recomputing free space each pass
  // (it drops as we feed and rises as the firmware executes). FIFO order is
  // preserved; nothing is fed ahead of an older chunk.
  while (!excessChunks.empty() && NeededFrames(excessChunks.front()) <= FreeFramesAvailable())
  {
    if (firstChunk && getGroupState() != RSIState::RSIStateIDLE)
    {
      break;
    }
    const bool chunkIsFinal = excessChunks.front().IsFinal;
    // Count frames handed to the firmware here (base path, always runs even if a test
    // overrides FeedChunk) so per-trajectory completion accounting stays accurate.
    // Online chunks (TrajectoryId 0, unsized BEGIN) get no completion threshold, so
    // they must not advance the executed-frame count either: counting them offsets
    // executed above every later sized goal's threshold permanently, and those goals
    // retire (and tolerance-abort) the moment they BEGIN. While residual online
    // frames still drain from the firmware, executed under-reports instead --
    // completion late, never early (the documented safe direction).
    if (excessChunks.front().TrajectoryId != 0)
    {
      totalFramesFed += static_cast<std::uint64_t>(NeededFrames(excessChunks.front()));
    }
    FeedChunk(excessChunks.front());
    excessChunks.pop_front();
    if (chunkIsFinal)
    {
      break;
    }
  }

  // A chunk remains that would not fit -> the firmware buffer isn't draining fast
  // enough (or at all: the exact failure we chased). Warn, throttled, with the numbers
  // that separate "firmware not draining" (fte pinned high, need > free) from "producer
  // out-running the drive". Reset the throttle when unblocked so a fresh stall's first
  // cycle always logs.
  if (!excessChunks.empty())
  {
    if (drainBlockedCycles++ % kDrainBlockedLogPeriod == 0)
    {
      char fteBuf[128];
      FormatFramesToExecute(axes, fteBuf, sizeof(fteBuf));
      RCLCPP_WARN(logger,
        "DrainQueue blocked: need=%d free=%d fte=[%s] qdepth=%zu pendingFinal=%d "
        "(firmware buffer not draining)",
        NeededFrames(excessChunks.front()), FreeFramesAvailable(), fteBuf,
        excessChunks.size(), pendingFinal ? 1 : 0);
    }
  }
  else
  {
    drainBlockedCycles = 0;
  }
}

void RapidCodeSystemHardware::AppendTrajectory()
{
  const int baseIndex = rapidcode_trajectory_transfer::Decode(xferChunkBaseIndex);
  const int chunkLen = std::clamp(
    rapidcode_trajectory_transfer::Decode(xferChunkLen), 0, rapidcode_trajectory_transfer::ChunkCapacity);
  const bool chunkFinal = static_cast<bool>(rapidcode_trajectory_transfer::Decode(xferChunkFinal));

  // Take ownership of the incoming chunk: ALWAYS enqueue (deep copy -- the xferSlot*
  // buffers are overwritten next cycle), then let DrainQueue() feed whatever fits.
  // Always-enqueue keeps the FIFO strictly ordered and never drops or reorders a
  // chunk. Acking on enqueue (accepted-index below) is what keeps the controller
  // from stalling; completion is gated on is_moving (read()), which accounts for
  // the still-queued chunks.
  if (chunkLen > 0 || chunkFinal)  // skip an empty, non-final chunk
  {
    if (excessChunks.size() >= kMaxQueuedChunks)
    {
      // Runaway producer / firmware not draining: fail the goal rather than grow
      // host memory without bound. The nonzero error_code drives the controller's
      // abort path; HandleAbort halts motion and clears the FIFO.
      RCLCPP_ERROR(logger,
        "AppendTrajectory: queued-chunk FIFO hit cap (%zu); aborting transfer.",
        kMaxQueuedChunks);
      xferErrorCode = rapidcode_trajectory_transfer::Encode(kErrorQueueOverflow);
      HandleAbort();
      return;
    }
    excessChunks.emplace_back(chunkFinal, static_cast<std::size_t>(chunkLen),
      xferSlotDuration, xferSlotPosition, xferSlotVelocity, xferSlotAcceleration, xferSlotJerk,
      currentTrajectoryId);
  }

  if (chunkFinal)
  {
    RCLCPP_INFO(logger,
      "AppendTrajectory: FINAL chunk enqueued (baseIndex=%d len=%d qdepth=%zu).",
      baseIndex, chunkLen, excessChunks.size());
  }

  DrainQueue();  // feed as much of the FIFO as currently fits

  // Ack-on-enqueue: the hardware now owns these points, so report them accepted
  // even while they're still queued. error_code is left untouched here so an
  // overflow abort stays latched until the next Begin/Reset clears it.
  xferAcceptedPointIndex = rapidcode_trajectory_transfer::Encode(baseIndex + chunkLen - 1);
}

void RapidCodeSystemHardware::UpdateCompletion()
{
  if (trajectoryCompletions.empty()) { return; }
  // executed = frames handed to the firmware minus those still queued in its buffer.
  // Underflow-guarded: a final chunk can enqueue extra firmware closing frames that
  // totalFramesFed doesn't count, which only makes `executed` smaller (completion
  // reported late, never early).
  const std::uint64_t remaining = static_cast<std::uint64_t>(std::max(0, FramesRemaining()));
  const std::uint64_t executed = (totalFramesFed > remaining) ? (totalFramesFed - remaining) : 0;

  // Thresholds are pushed in increasing id / cumulative-frame order, so pop
  // front-to-back: every trajectory whose last frame has executed is complete. The
  // published id only ever advances (monotonic).
  std::uint64_t completedId =
    rapidcode_trajectory_transfer::Decode<std::uint64_t>(xferCompletedTrajectoryId);
  while (!trajectoryCompletions.empty() &&
    trajectoryCompletions.front().cumulativeFrames <= executed)
  {
    completedId = trajectoryCompletions.front().id;
    trajectoryCompletions.pop_front();
  }
  xferCompletedTrajectoryId = rapidcode_trajectory_transfer::Encode(completedId);
}

void RapidCodeSystemHardware::ResetCompletionTracking()
{
  trajectoryCompletions.clear();
  totalFramesFed = 0;
  cumulativeExpectedFrames = 0;
  currentTrajectoryId = 0;
  xferCompletedTrajectoryId = rapidcode_trajectory_transfer::Encode<std::uint64_t>(0);
}

void RapidCodeSystemHardware::HandleAbort()
{
  stopRecoverPending = false;
  commandedStopFault = false;  // an abort supersedes any pending stop recovery
  if (multiAxis != nullptr)
  {
    multiAxis->Abort();  // halt the running group move immediately
  }
  excessChunks.clear();
  pendingFinal = false;
  transferMoveOpen = false;
  firstChunk = true;  // the aborted move is closed; the next feed must reopen it
  ResetCompletionTracking();  // dropped goals will never complete; resync the counters
}

void RapidCodeSystemHardware::HandleStop()
{
  // Commanded stop (the controller's ~/stop service): decelerate at the e-stop rate.
  // Stop() cannot be used here: it decelerates but does NOT close an open streamed
  // (MovePVT) move, so the firmware's OUT_OF_FRAMES supervisor stays armed and
  // e-stops anyway once the decel drains the buffer below kEmptyCount. EStop() ends
  // the move outright; the amps stay enabled and the group settles in ERROR (rsi.h).
  // That ERROR is self-inflicted, so flag it: HandleReset may ClearFaults it, while
  // real (unflagged) faults stay a manual recovery. error_code stays 0: this is an
  // operator action, not a fault; the controller holds its own stop latch.
  stopRecoverPending = false;
  commandedStopFault = true;
  if (multiAxis != nullptr)
  {
    try
    {
      multiAxis->EStop();
    }
    catch (const std::exception & estopError)
    {
      // The group may already be faulted (e.g. the watchdog raced us); the ERROR
      // state we were steering to is already latched, so treat this as done.
      RCLCPP_WARN(logger, "HandleStop: EStop threw (group already faulted?): %s",
        estopError.what());
    }
  }
  excessChunks.clear();
  pendingFinal = false;
  transferMoveOpen = false;
  firstChunk = true;  // the stopped move is closed; the next feed must reopen it
  ResetCompletionTracking();  // dropped goals will never complete; resync the counters
}

void RapidCodeSystemHardware::HandleReset()
{
  // Re-arm for a fresh goal: drop any queued tail and clear the transfer FSM so a
  // faulted move does not poison the next one.
  excessChunks.clear();
  pendingFinal = false;
  transferMoveOpen = false;
  // The faulted/aborted move is closed; the next feed must reopen the firmware move
  // (reopen gate + the NO_WAIT open toggle in FeedChunk).
  firstChunk = true;
  xferAcceptedPointIndex = rapidcode_trajectory_transfer::Encode(-1);
  // Reset is the ONLY place error_code is cleared (BEGIN no longer clears it), so a
  // latched fault survives until the operator explicitly acknowledges it (the
  // controller's ~/reset_fault service emits this token). Drive-side recovery of a
  // REAL fault (ClearFaults after a drive trip / watchdog e-stop) is deliberately NOT
  // done here -- it stays a manual step (RapidSetupX), per the operator workflow. The
  // one exception is the SELF-INFLICTED e-stop from HandleStop (commandedStopFault):
  // the operator's stop -> reset pair is the sanctioned recovery for it. If Reset is
  // acknowledged while the drive is still faulted, the next motion attempt re-latches
  // error_code in DrainQueue before any motion.
  xferErrorCode = rapidcode_trajectory_transfer::Encode(0);
  ResetCompletionTracking();  // resync completion counters with the controller's fresh ids

  // Recover the group for the operator's re-arm. STOPPED -> IDLE via ClearFaults
  // (a stop from outside, e.g. RapidSetup). ERROR from a commanded stop (flagged
  // self-inflicted) -> IDLE via ClearFaults too. Transient decel states reject
  // ClearFaults -- defer to DrainQueue via the flag until the group settles.
  // Unflagged ERROR/STOPPING_ERROR are real faults and stay manual (the block above).
  if (multiAxis != nullptr)
  {
    const RSIState groupState = multiAxis->StateGet();
    if (groupState == RSIState::RSIStateSTOPPED)
    {
      multiAxis->ClearFaults();
    }
    else if (groupState == RSIState::RSIStateSTOPPING)
    {
      stopRecoverPending = true;
    }
    else if (commandedStopFault && groupState == RSIState::RSIStateERROR)
    {
      multiAxis->ClearFaults();
    }
    else if (commandedStopFault && groupState == RSIState::RSIStateSTOPPING_ERROR)
    {
      stopRecoverPending = true;  // consumed in DrainQueue once the group settles
    }
  }
  // The flag's lifetime is one stop -> reset window. Keep it only while a deferred
  // ClearFaults is still owed; otherwise a stale flag would let a LATER real fault
  // be auto-cleared by an unrelated Reset.
  if (!stopRecoverPending)
  {
    commandedStopFault = false;
  }
}

void RapidCodeSystemHardware::AbortTransfer(const char * reason, bool eStop, int errorCode)
{
  stopRecoverPending = false;
  commandedStopFault = false;  // a real fault supersedes the self-inflicted-stop flag
  // Optionally e-stop the group -- only when it may still be executing (the write()
  // exception path). A group already reported in an error state is stopping/stopped
  // already, so re-EStopping it is unnecessary and can itself throw.
  if (eStop && multiAxis != nullptr)
  {
    try
    {
      multiAxis->EStop();  // decelerate the group at the e-stop rate, then hold
    }
    catch (const std::exception & estopError)
    {
      RCLCPP_ERROR(logger, "%s: EStop failed: %s", reason, estopError.what());
    }
  }
  // Drop the queued trajectory + reset the transfer FSM so we stop feeding and don't
  // replay a bad move, then latch a nonzero error_code so the controller aborts the
  // active goal instead of waiting forever on is_moving.
  excessChunks.clear();
  pendingFinal = false;
  transferMoveOpen = false;
  firstChunk = true;  // the killed move is closed; the next feed must reopen it
  ResetCompletionTracking();  // dropped goals won't complete; resync (error is latched below)
  xferErrorCode = rapidcode_trajectory_transfer::Encode(errorCode);
  DrainRapidCodeErrorLog("MultiAxis(abort)", multiAxis);  // surface any drive fault text
  RCLCPP_ERROR(logger,
    "%s: eStop=%d -> cleared queue, latched error_code=%d "
    "(controller will abort; recovery needs ClearFaults).",
    reason, eStop ? 1 : 0, errorCode);
}

void RapidCodeSystemHardware::HandleWriteFault()
{
  // A std::exception escaped the feed path (typically a MovePVT/append the firmware
  // rejected). It may still be executing, so e-stop as part of the abort.
  AbortTransfer("write() fault", /*eStop=*/true, kErrorWriteFault);
}

hardware_interface::return_type RapidCodeSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  try
  {
    // Drain the queued-chunk FIFO every cycle -- including cycles with no new
    // command -- so the trajectory tail still feeds after the controller has sent
    // its last chunk and gone quiet. Must run before the no-new-command early-out.
    DrainQueue();

    // Liveness heartbeat: proves the RT write() thread is still cycling even when it
    // has nothing to feed (so a stalled move is distinguishable from a dead thread),
    // and reports queue/buffer state. Throttled; runs before the no-new-command exit.
    if (++writeCycles % kWriteHeartbeatPeriod == 0)
    {
      RCLCPP_INFO(logger,
        "write() heartbeat: cycle=%llu qdepth=%zu free_frames=%d pendingFinal=%d moveOpen=%d.",
        static_cast<unsigned long long>(writeCycles), excessChunks.size(),
        FreeFramesAvailable(), pendingFinal ? 1 : 0, transferMoveOpen ? 1 : 0);
    }

    ConsumePendingCommand();
  }
  catch (const std::exception & error)
  {
    RCLCPP_ERROR(logger, "write() fault: %s.", error.what());
    LogMultiAxisDiagnostics("write exception", true);
    HandleWriteFault();  // e-stop the group, drop the queued tail, latch the fault
  }
  return hardware_interface::return_type::OK;
}

void RapidCodeSystemHardware::ConsumePendingCommand()
{
  // Single-slot mailbox: a new command exists only when the controller advanced
  // command_sequence past our last ack. The ack is published AFTER the token
  // dispatch (ack-on-enqueue -- AppendTrajectory deep-copies the slot payload into
  // the host FIFO before we ack, never ack-on-execute).
  const uint64_t currentSequence =
    rapidcode_trajectory_transfer::Decode<uint64_t>(xferCommandSequence);
  if (currentSequence <= ackSequence) { return; }  // no new command this cycle

  const auto commandToken =
    rapidcode_trajectory_transfer::Decode<rapidcode_trajectory_transfer::CommandToken>(xferCommandToken);
  switch (commandToken)
  {
    case rapidcode_trajectory_transfer::CommandToken::Reset:
      RCLCPP_INFO(logger, "write(): command token RESET received");
      HandleReset();
      break;
    case rapidcode_trajectory_transfer::CommandToken::Abort:
      RCLCPP_INFO(logger, "write(): command token ABORT received");
      HandleAbort();
      break;
    case rapidcode_trajectory_transfer::CommandToken::Stop:
      RCLCPP_INFO(logger, "write(): command token STOP received");
      HandleStop();
      break;
    case rapidcode_trajectory_transfer::CommandToken::Begin:
      BeginTrajectoryTransfer();
      RCLCPP_INFO(logger,
        "write(): command token BEGIN received. Trajectory size=%d points, valid fields=0x%X",
        transferTrajectorySize, transferValidFields);
      break;
    case rapidcode_trajectory_transfer::CommandToken::AppendChunk:
      AppendTrajectory();
      break;
    case rapidcode_trajectory_transfer::CommandToken::None:
    default:
      break;
  }

  ackSequence = currentSequence;
  xferAckSequence = rapidcode_trajectory_transfer::Encode(ackSequence);
}

}  // namespace rapidcode_system

PLUGINLIB_EXPORT_CLASS(
  rapidcode_system::RapidCodeSystemHardware, hardware_interface::SystemInterface)

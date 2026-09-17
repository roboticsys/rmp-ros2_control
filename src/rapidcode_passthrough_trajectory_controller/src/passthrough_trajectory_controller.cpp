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

#include "rapidcode_passthrough_trajectory_controller/passthrough_trajectory_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"

namespace
{
namespace protocol = rapidcode_trajectory_transfer;
using Cmd = rapidcode_trajectory_transfer::CommandToken;
}  // namespace

namespace rapidcode_passthrough_trajectory_controller
{

static const char * InterpolationName(const Interpolation method)
{
  switch (method)
  {
    case Interpolation::Auto: return "auto";
    case Interpolation::Linear: return "linear";
    case Interpolation::Quadratic: return "quadratic";
    case Interpolation::Cubic: return "cubic";
    case Interpolation::Quintic: return "quintic";
  }
  return "unknown";
}

// The trajectory point fields an interpolation order requires on every point.
struct FieldNeeds
{
  bool velocity = false;
  bool acceleration = false;
};

// Which fields the resampler's configured interpolation order consumes: quintic needs
// velocities + accelerations, cubic and quadratic need velocities, linear needs only
// positions, and auto selects per segment from whatever is present (so nothing beyond
// positions is required up front). Pure.
static FieldNeeds RequiredFields(const Interpolation method)
{
  FieldNeeds needs;
  needs.velocity = method == Interpolation::Quadratic ||
    method == Interpolation::Cubic || method == Interpolation::Quintic;
  needs.acceleration = method == Interpolation::Quintic;
  return needs;
}

// Structural validation shared by BOTH intakes (HandleGoal and the online topic
// callback): non-empty, every managed joint present, and every point carrying
// positions plus the fields the interpolation order needs (`needs`), fully sized --
// a present derivative must cover every joint so a segment that uses it never indexes
// a short vector. Writes a short reason on rejection; returns true when
// RemapTrajectory can safely consume the trajectory. Pure.
static bool ValidateTrajectoryStructure(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<std::string> & joint_names, const FieldNeeds & needs,
  std::string & out_reason)
{
  const std::size_t joint_count = joint_names.size();
  if (trajectory.points.empty()) { out_reason = "empty trajectory"; return false; }
  for (const auto & joint : joint_names)
  {
    if (std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), joint) ==
        trajectory.joint_names.end())
    {
      out_reason = "managed joint '" + joint + "' is not in the trajectory";
      return false;
    }
  }
  const auto field_sized = [joint_count](const std::vector<double> & field, bool required) {
    return field.size() == joint_count || (!required && field.empty());
  };
  for (const auto & point : trajectory.points)
  {
    if (point.positions.size() != joint_count ||
        !field_sized(point.velocities, needs.velocity) ||
        !field_sized(point.accelerations, needs.acceleration))
    {
      out_reason = "a point is missing the position/velocity/acceleration fields the "
        "configured interpolation needs";
      return false;
    }
  }
  return true;
}

// Parse an interpolation parameter string to its enum value; std::nullopt on an
// unknown string so on_configure can report it. Pure.
static std::optional<Interpolation> ParseInterpolation(const std::string & name)
{
  if (name == "auto") { return Interpolation::Auto; }
  if (name == "linear") { return Interpolation::Linear; }
  if (name == "quadratic") { return Interpolation::Quadratic; }
  if (name == "cubic") { return Interpolation::Cubic; }
  if (name == "quintic") { return Interpolation::Quintic; }
  return std::nullopt;
}

// The online feed gate's hysteresis band converted from seconds of motion to points on
// the sample grid.
struct GateBand
{
  int low_water_points = 0;
  int high_water_points = 0;
};

// Convert the feed-gate band (seconds of motion) to points on the sample grid. Floors
// keep the band well-formed even on a bad config: the low water must cover a few
// control cycles of jitter (>= 2 points) so the firmware buffer can never drain to
// empty mid-open-move (starvation fault), and the high water must sit strictly above
// the low water so the hysteresis is a band, not a single edge. Pure.
static GateBand GateBandToPoints(const double low_water_seconds,
  const double high_water_seconds, const double sample_period)
{
  const double sample_grid = std::max(sample_period, 1.0e-6);
  GateBand band;
  band.low_water_points =
    std::max(2, static_cast<int>(std::ceil(low_water_seconds / sample_grid)));
  band.high_water_points = std::max(band.low_water_points + 1,
    static_cast<int>(std::ceil(high_water_seconds / sample_grid)));
  return band;
}

// Resolve a per-joint limit array from a parameter: an empty array fills to joint_count
// with `fallback`; a correctly-sized array passes through; any other size is an error.
// Writes the result into `out_limits`; returns false (out_limits untouched) on a size
// mismatch so on_configure can report it. Pure.
static bool ResolveJointLimits(const std::vector<double> & configured,
  std::size_t joint_count, double fallback, std::vector<double> & out_limits)
{
  if (configured.empty())
  {
    out_limits.assign(joint_count, fallback);
    return true;
  }
  if (configured.size() != joint_count) { return false; }
  out_limits = configured;
  return true;
}

controller_interface::CallbackReturn PassthroughTrajectoryController::on_init()
{
  // Declare parameters with defaults; values are read in on_configure (the node's
  // parameters aren't populated until then).
  try
  {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::string>("gpio_name", std::string(protocol::DefaultGpioName));
    auto_declare<int>("chunk_size", chunk_size_);
    auto_declare<double>("goal_position_tolerance", goal_position_tolerance_);
    auto_declare<double>("first_point_tolerance", first_point_tolerance_);
    auto_declare<std::string>("action_name", action_name_);
    auto_declare<double>("sample_period", sample_period_);
    auto_declare<std::string>("interpolation", std::string(InterpolationName(interpolation_)));
    auto_declare<bool>("finalize_last_chunk", finalize_last_chunk_);
    auto_declare<bool>("online.enabled", online_enabled_);
    auto_declare<double>("online.producer_timeout", online_producer_timeout_);
    auto_declare<double>("online.committed_horizon", online_committed_horizon_);
    auto_declare<double>("online.low_water", online_low_water_);
    auto_declare<std::vector<double>>("online.max_velocity", std::vector<double>());
    auto_declare<std::vector<double>>("online.max_acceleration", std::vector<double>());
    auto_declare<std::vector<double>>("online.max_jerk", std::vector<double>());
  }
  catch (const std::exception & error)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool PassthroughTrajectoryController::CyclePeriodSustainable(
  const double update_period_seconds, const double sample_period_seconds, std::string & reason)
{
  if (!(sample_period_seconds > 0.0) || !std::isfinite(sample_period_seconds))
  {
    reason = "sample_period must be a positive, finite number of seconds";
    return false;
  }
  if (!(update_period_seconds > 0.0) || !std::isfinite(update_period_seconds))
  {
    reason = "the controller manager update_rate must be a positive number of Hz";
    return false;
  }
  const double ratio = update_period_seconds / sample_period_seconds;
  const double nearest = std::round(ratio);
  if (std::fabs(ratio - nearest) > kCycleRatioTolerance * std::max(ratio, 1.0))
  {
    reason = "the update period (" + std::to_string(update_period_seconds) +
      " s) is not a whole multiple of sample_period (" +
      std::to_string(sample_period_seconds) + " s): each transferred point must span a "
      "whole number of firmware samples";
    return false;
  }
  if (nearest < static_cast<double>(kMinSamplesPerCycle))
  {
    reason = "the update period (" + std::to_string(update_period_seconds) +
      " s) must be at least " + std::to_string(kMinSamplesPerCycle) +
      " x sample_period (" + std::to_string(sample_period_seconds) +
      " s): a ROS loop at the firmware rate starves the PVT stream (OUT_OF_FRAMES)";
    return false;
  }
  return true;
}

controller_interface::CallbackReturn PassthroughTrajectoryController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!ReadControllerParameters() || !ReadOnlineParameters())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // the cycle period is fixed here, and an unsustainable one
  // is refused here. update_rate is the controller manager's (or this controller's own
  // override), already resolved by the base class before configure.
  const unsigned int update_rate_hz = get_update_rate();
  const double update_period =
    update_rate_hz == 0 ? 0.0 : 1.0 / static_cast<double>(update_rate_hz);
  std::string cycle_reason;
  if (!CyclePeriodSustainable(update_period, sample_period_, cycle_reason))
  {
    RCLCPP_ERROR(get_node()->get_logger(),
      "on_configure: unsustainable cycle: update_rate=%u Hz, sample_period=%.6f s: %s.",
      update_rate_hz, sample_period_, cycle_reason.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  SizeInterfaceIndexCaches();
  CreateIntakes();

  RCLCPP_INFO(
    get_node()->get_logger(),
    "on_configure: %zu joint(s), gpio '%s', chunk_size=%d (cap %d), interpolation '%s', "
    "action '%s'.",
    joint_names_.size(), gpio_name_.c_str(), chunk_size_, protocol::ChunkCapacity,
    InterpolationName(interpolation_), action_name_.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

bool PassthroughTrajectoryController::ReadControllerParameters()
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  gpio_name_ = get_node()->get_parameter("gpio_name").as_string();
  chunk_size_ = static_cast<int>(get_node()->get_parameter("chunk_size").as_int());
  goal_position_tolerance_ = get_node()->get_parameter("goal_position_tolerance").as_double();
  first_point_tolerance_ = get_node()->get_parameter("first_point_tolerance").as_double();
  action_name_ = get_node()->get_parameter("action_name").as_string();
  sample_period_ = get_node()->get_parameter("sample_period").as_double();
  finalize_last_chunk_ = get_node()->get_parameter("finalize_last_chunk").as_bool();

  const std::string interpolation = get_node()->get_parameter("interpolation").as_string();
  const std::optional<Interpolation> parsed = ParseInterpolation(interpolation);
  if (!parsed)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
      "on_configure: unknown 'interpolation' value '%s' "
      "(expected auto|linear|quadratic|cubic|quintic).",
      interpolation.c_str());
    return false;
  }
  interpolation_ = *parsed;

  // Clamp the chunk size to the channel width the gpio declares (kChunkCapacity).
  chunk_size_ = std::clamp(chunk_size_, 1, static_cast<int>(protocol::ChunkCapacity));

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "on_configure: 'joints' parameter is empty.");
    return false;
  }
  return true;
}

bool PassthroughTrajectoryController::ReadOnlineParameters()
{
  online_enabled_ = get_node()->get_parameter("online.enabled").as_bool();
  online_producer_timeout_ = get_node()->get_parameter("online.producer_timeout").as_double();
  online_committed_horizon_ = get_node()->get_parameter("online.committed_horizon").as_double();
  online_low_water_ = get_node()->get_parameter("online.low_water").as_double();

  const GateBand band =
    GateBandToPoints(online_low_water_, online_committed_horizon_, sample_period_);
  online_low_water_points_ = band.low_water_points;
  online_high_water_points_ = band.high_water_points;
  if (online_committed_horizon_ <= online_low_water_)
  {
    RCLCPP_WARN(get_node()->get_logger(),
      "on_configure: online.committed_horizon (%.3fs) <= online.low_water (%.3fs); "
      "clamped the gate band to [%d, %d] points.",
      online_committed_horizon_, online_low_water_,
      online_low_water_points_, online_high_water_points_);
  }

  const bool limits_ok =
    ResolveJointLimits(get_node()->get_parameter("online.max_velocity").as_double_array(),
      joint_names_.size(), kDefaultOnlineMaxVelocity, online_max_velocity_) &&
    ResolveJointLimits(get_node()->get_parameter("online.max_acceleration").as_double_array(),
      joint_names_.size(), kDefaultOnlineMaxAcceleration, online_max_acceleration_) &&
    ResolveJointLimits(get_node()->get_parameter("online.max_jerk").as_double_array(),
      joint_names_.size(), kDefaultOnlineMaxJerk, online_max_jerk_);
  if (!limits_ok)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
      "on_configure: online.max_velocity/acceleration/jerk must be empty or have exactly "
      "%zu entries (one per managed joint).", joint_names_.size());
    return false;
  }
  return true;
}

void PassthroughTrajectoryController::SizeInterfaceIndexCaches()
{
  const std::size_t kcap = static_cast<std::size_t>(protocol::ChunkCapacity);
  ci_slot_time_.assign(kcap, -1);
  ci_slot_joint_position_.assign(kcap * joint_names_.size(), -1);
  ci_slot_joint_velocity_.assign(kcap * joint_names_.size(), -1);
  ci_slot_joint_acceleration_.assign(kcap * joint_names_.size(), -1);
  ci_slot_joint_jerk_.assign(kcap * joint_names_.size(), -1);
  si_joint_position_.assign(joint_names_.size(), -1);
  feedback_snapshot_.actual_positions = std::vector<std::atomic<double>>(joint_names_.size());
  si_joint_velocity_.assign(joint_names_.size(), -1);
}

void PassthroughTrajectoryController::CreateIntakes()
{
  // FollowJointTrajectory action server -- the same action MoveIt drives a stock
  // JointTrajectoryController with, so MoveIt compatibility is preserved.
  using std::placeholders::_1;
  using std::placeholders::_2;
  action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
    get_node(), action_name_,
    std::bind(&PassthroughTrajectoryController::HandleGoal, this, _1, _2),
    std::bind(&PassthroughTrajectoryController::HandleCancel, this, _1),
    std::bind(&PassthroughTrajectoryController::HandleAccepted, this, _1));

  // Second intake: the online-jog topic. A continuous producer (MoveIt Servo / a jog
  // node) publishes JointTrajectory windows here; the callback stages the newest into
  // rt_incoming_online_ (latest-wins) for update() to stream as an open move.
  if (online_enabled_)
  {
    joint_command_subscriber_ =
      get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "~/joint_trajectory", rclcpp::SystemDefaultsQoS(),
        [this](const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr message) {
          JointTrajectoryTopicCallback(message);
        });
  }

  // Operator fault acknowledgement: after fixing the drive manually (RapidSetupX
  // ClearFaults), this service asks the RT fault branch to emit Cmd::Reset, which
  // clears the hardware's latched error_code. Responds success=false when no fault is
  // latched (the request would be meaningless); the actual clear happens on a later RT
  // cycle -- a repeat call answering "no fault latched" is the confirmation.
  reset_fault_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/reset_fault",
    [this](std_srvs::srv::Trigger::Request::ConstSharedPtr /*request*/,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
      const int error_code = hardware_error_code_.load();
      const bool stop_latched = stop_latched_.load();
      if (error_code == 0 && !stop_latched)
      {
        response->success = false;
        response->message = "no fault latched";
        return;
      }
      reset_requested_.store(true);
      response->success = true;
      response->message = "reset requested (error_code=" + std::to_string(error_code) +
        (stop_latched ? ", stop latch released)" : ")");
      RCLCPP_INFO(get_node()->get_logger(),
        "reset_fault: operator acknowledged fault (error_code=%d); emitting Reset.",
        error_code);
    });

  // Operator stop: latch immediately (both intakes reject from this line on) and ask
  // the RT cycle to abort the pipelines and emit Cmd::Stop -- the hardware decelerates
  // the group to rest at the stop rate. Sticky by design: motion stays rejected until
  // the operator clears the latch through ~/reset_fault (the no-resume obligation).
  // Idempotent: a repeat call while latched answers success.
  stop_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/stop",
    [this](std_srvs::srv::Trigger::Request::ConstSharedPtr /*request*/,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
      const bool was_latched = stop_latched_.exchange(true);
      stop_requested_.store(true);
      response->success = true;
      response->message = was_latched ? "already stopped (latch held)"
                                      : "stop latched; call ~/reset_fault to resume";
      RCLCPP_WARN(get_node()->get_logger(),
        "stop: operator stop%s; goals and jog input are rejected until ~/reset_fault.",
        was_latched ? " (already latched)" : " latched");
    });
}

controller_interface::InterfaceConfiguration
PassthroughTrajectoryController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(protocol::FullName(gpio_name_, protocol::interface_names::CommandToken));
  config.names.push_back(protocol::FullName(gpio_name_, protocol::interface_names::CommandSequence));
  config.names.push_back(protocol::FullName(gpio_name_, protocol::interface_names::TrajectoryId));
  config.names.push_back(protocol::FullName(gpio_name_, protocol::interface_names::TrajectorySize));
  config.names.push_back(protocol::FullName(gpio_name_, protocol::interface_names::ValidFields));
  config.names.push_back(protocol::FullName(gpio_name_, protocol::interface_names::ChunkBaseIndex));
  config.names.push_back(protocol::FullName(gpio_name_, protocol::interface_names::ChunkLen));
  config.names.push_back(protocol::FullName(gpio_name_, protocol::interface_names::ChunkFinal));
  for (std::size_t slot = 0; slot < static_cast<std::size_t>(protocol::ChunkCapacity); ++slot)
  {
    config.names.push_back(protocol::FullName(gpio_name_, protocol::SlotDuration(slot)));
    for (std::size_t index = 0; index < joint_names_.size(); ++index)
    {
      config.names.push_back(protocol::FullName(gpio_name_, protocol::JointSlotPosition(index, slot)));
      config.names.push_back(protocol::FullName(gpio_name_, protocol::JointSlotVelocity(index, slot)));
      config.names.push_back(protocol::FullName(gpio_name_, protocol::JointSlotAcceleration(index, slot)));
      config.names.push_back(protocol::FullName(gpio_name_, protocol::JointSlotJerk(index, slot)));
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration
PassthroughTrajectoryController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(protocol::FullName(gpio_name_, protocol::interface_names::IsMoving));
  config.names.push_back(protocol::FullName(gpio_name_, protocol::interface_names::AckSequence));
  config.names.push_back(protocol::FullName(gpio_name_, protocol::interface_names::AcceptedPointIndex));
  config.names.push_back(protocol::FullName(gpio_name_, protocol::interface_names::ErrorCode));
  config.names.push_back(protocol::FullName(gpio_name_, protocol::interface_names::CompletedTrajectoryId));
  config.names.push_back(protocol::FullName(gpio_name_, protocol::interface_names::CommittedDepthPoints));
  // Joint position+velocity state for the continuity precondition, the goal
  // tolerance check, and action feedback.
  for (const auto & joint : joint_names_)
  {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
  }
  return config;
}

int PassthroughTrajectoryController::FindCommandInterface(const std::string & full_name) const
{
  for (std::size_t idx = 0; idx < command_interfaces_.size(); ++idx)
  {
    if (command_interfaces_[idx].get_name() == full_name) { return static_cast<int>(idx); }
  }
  return -1;
}

int PassthroughTrajectoryController::FindStateInterface(const std::string & full_name) const
{
  for (std::size_t idx = 0; idx < state_interfaces_.size(); ++idx)
  {
    if (state_interfaces_[idx].get_name() == full_name) { return static_cast<int>(idx); }
  }
  return -1;
}

double PassthroughTrajectoryController::ReadStateValue(int index) const
{
  // get_value() is deprecated in this hardware_interface; get_optional() is the
  // replacement. A missing value (lock contention) -> fall back to 0.0; for the
  // FSM scalars that reads as Idle/no-ack, which is the safe default.
  return state_interfaces_[static_cast<std::size_t>(index)].get_optional().value_or(0.0);
}

void PassthroughTrajectoryController::SetCommandValue(int index, double value)
{
  if (!command_interfaces_[static_cast<std::size_t>(index)].set_value(value))
  {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "set_value failed on command interface index %d (stale command this cycle).", index);
  }
}

controller_interface::CallbackReturn PassthroughTrajectoryController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!ResolveInterfaceIndices())
  {
    RCLCPP_ERROR(get_node()->get_logger(),
      "on_activate: could not resolve all trajectory_transfer interfaces -- check the "
      "URDF <gpio name=\"%s\"> block and joint names.", gpio_name_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // update() isn't running yet, so the RT-owned members are safe to touch here.
  ResetActionPipeline();
  PrepareOnlineWorkingSet();
  action_busy_.store(false);
  SetCommandValue(ci_command_, protocol::Encode(static_cast<int>(Cmd::None)));

  StartGoalMonitorTimer();
  OpenInputTrace();

  RCLCPP_INFO(get_node()->get_logger(), "on_activate: passthrough controller ready.");
  return controller_interface::CallbackReturn::SUCCESS;
}

bool PassthroughTrajectoryController::ResolveInterfaceIndices()
{
  bool all_found = true;
  const auto GetCommandIndex = [this, &all_found](const std::string & leaf) {
    const int idx = FindCommandInterface(protocol::FullName(gpio_name_, leaf));
    all_found = all_found && idx >= 0;
    return idx;
  };
  // Resolve every interface to an index once, so update() never searches by name.
  ci_command_ = GetCommandIndex(protocol::interface_names::CommandToken);
  ci_command_sequence_ = GetCommandIndex(protocol::interface_names::CommandSequence);
  ci_trajectory_id_ = GetCommandIndex(protocol::interface_names::TrajectoryId);
  ci_trajectory_size_ = GetCommandIndex(protocol::interface_names::TrajectorySize);
  ci_valid_fields_ = GetCommandIndex(protocol::interface_names::ValidFields);
  ci_chunk_base_ = GetCommandIndex(protocol::interface_names::ChunkBaseIndex);
  ci_chunk_len_ = GetCommandIndex(protocol::interface_names::ChunkLen);
  ci_chunk_final_ = GetCommandIndex(protocol::interface_names::ChunkFinal);

  const auto GetStateIndex = [this, &all_found](const std::string & leaf) {
    const int idx = FindStateInterface(protocol::FullName(gpio_name_, leaf));
    all_found = all_found && idx >= 0;
    return idx;
  };
  si_is_moving_ = GetStateIndex(protocol::interface_names::IsMoving);
  si_ack_sequence_ = GetStateIndex(protocol::interface_names::AckSequence);
  si_accepted_index_ = GetStateIndex(protocol::interface_names::AcceptedPointIndex);
  si_error_code_ = GetStateIndex(protocol::interface_names::ErrorCode);
  si_completed_trajectory_id_ = GetStateIndex(protocol::interface_names::CompletedTrajectoryId);
  si_committed_depth_ = GetStateIndex(protocol::interface_names::CommittedDepthPoints);

  const std::size_t joint_count = joint_names_.size();
  for (std::size_t slot = 0; slot < static_cast<std::size_t>(protocol::ChunkCapacity); ++slot)
  {
    ci_slot_time_[slot] = GetCommandIndex(protocol::SlotDuration(slot));
    for (std::size_t joint = 0; joint < joint_count; ++joint)
    {
      const std::size_t idx = slot * joint_count + joint;
      ci_slot_joint_position_[idx] = GetCommandIndex(protocol::JointSlotPosition(joint, slot));
      ci_slot_joint_velocity_[idx] = GetCommandIndex(protocol::JointSlotVelocity(joint, slot));
      ci_slot_joint_acceleration_[idx] = GetCommandIndex(protocol::JointSlotAcceleration(joint, slot));
      ci_slot_joint_jerk_[idx] = GetCommandIndex(protocol::JointSlotJerk(joint, slot));
    }
  }
  // Joint state interfaces live under the JOINT prefix (e.g. "joint1/position"), not
  // the gpio prefix -- look them up directly, NOT via GetStateIndex (which prepends
  // gpio_name_ and would search for the nonexistent "trajectory_transfer/joint1/...").
  for (std::size_t joint = 0; joint < joint_count; ++joint)
  {
    si_joint_position_[joint] = FindStateInterface(joint_names_[joint] + "/position");
    si_joint_velocity_[joint] = FindStateInterface(joint_names_[joint] + "/velocity");
    all_found = all_found && si_joint_position_[joint] >= 0 && si_joint_velocity_[joint] >= 0;
  }
  return all_found;
}

void PassthroughTrajectoryController::ResetActionPipeline()
{
  // Both sides reset on activate (the hardware zeroes completed_trajectory_id in its
  // on_activate), so ids resync from 1 with no explicit Reset command.
  feeding_.reset();
  pending_.clear();
  inflight_.clear();
  next_trajectory_id_ = 1;
  last_adopted_end_.clear();
  have_last_end_ = false;
  cancel_requested_.store(false);
  // The stop latch is NOT cleared here: a deactivation (or an operator stop) that
  // preceded this activate left the group e-stopped in ERROR, and only ~/reset_fault's
  // Cmd::Reset clears that on the hardware side (a reset is required
  // before new goals are accepted). The stop itself is not re-emitted.
  stop_requested_.store(false);
  stop_command_pending_ = false;
  feedback_head_id_ = 0;
  feedback_head_elapsed_ = 0.0;
  feedback_snapshot_.trajectory_id.store(0, std::memory_order_relaxed);
  { std::lock_guard<std::mutex> lock(incoming_mutex_); incoming_.clear(); }
  { std::lock_guard<std::mutex> lock(monitored_mutex_); monitored_goals_.clear(); }
}

void PassthroughTrajectoryController::PrepareOnlineWorkingSet()
{
  const std::size_t joint_count = joint_names_.size();
  online_committed_pos_.assign(joint_count, 0.0);
  online_committed_vel_.assign(joint_count, 0.0);
  // Reserve the stop-tail buffer to its worst case (the capped duration at this sample
  // grid) so BeginOnlineStopTail refills it in place without an RT heap allocation.
  const double sample_grid = std::max(sample_period_, 1.0e-6);
  const int stop_tail_capacity = std::max(kStopTailReservePoints,
    static_cast<int>(std::ceil(kMaxStopTailDuration / sample_grid)) + 2);
  online_stop_traj_ = std::make_shared<GoalTrajectory>(joint_count, stop_tail_capacity);
  // Splice working set (phase 3): the reusable blend+extension buffer (worst case = the
  // capped blend plus the constant-velocity extension), the velocity-intent scratch,
  // and the all-zeros rest target the stop-tail blends toward.
  const int splice_capacity = static_cast<int>(std::ceil(
    (kMaxStopTailDuration + online_producer_timeout_ + kSpliceExtensionMargin) /
    sample_grid)) + 2;
  online_splice_traj_ = std::make_shared<GoalTrajectory>(joint_count, splice_capacity);
  online_splice_vel_.assign(joint_count, 0.0);
  online_rest_vel_.assign(joint_count, 0.0);
  online_identity_mapping_.resize(joint_count);
  for (std::size_t joint = 0; joint < joint_count; ++joint)
  {
    online_identity_mapping_[joint] = static_cast<int>(joint);
  }
  // Flush any snapshot staged while inactive so it isn't adopted on the first cycle.
  rt_incoming_online_.writeFromNonRT(std::shared_ptr<GoalTrajectory>());
  ResetOnlineState();  // also clears online_busy_
}

void PassthroughTrajectoryController::StartGoalMonitorTimer()
{
  // Periodically flush each in-flight goal's queued result/feedback off the RT update()
  // path (same pattern as JointTrajectoryController, extended to many goals). An entry is
  // pruned once its RT-side `terminal` flag is set AND this tick has flushed it.
  goal_handle_timer_ = get_node()->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(action_monitor_period_.seconds())),
    [this]() {
      // Feedback for the goal the RT snapshot says is executing (0 == none). Built
      // here, off the RT path, then flushed by the same runNonRealtime() call below.
      const std::uint64_t executing_id =
        feedback_snapshot_.trajectory_id.load(std::memory_order_relaxed);
      const double elapsed = feedback_snapshot_.elapsed_seconds.load(std::memory_order_relaxed);
      std::lock_guard<std::mutex> lock(monitored_mutex_);
      for (auto iter = monitored_goals_.begin(); iter != monitored_goals_.end(); )
      {
        if (executing_id != 0 && iter->goal && iter->assigned_id &&
          !(iter->terminal && iter->terminal->load()) &&
          iter->assigned_id->load(std::memory_order_relaxed) == executing_id)
        {
          PublishGoalFeedback(*iter, executing_id, elapsed);
        }
        if (iter->goal) { iter->goal->runNonRealtime(); }  // flush queued feedback/result
        if (iter->terminal && iter->terminal->load()) { iter = monitored_goals_.erase(iter); }
        else { ++iter; }
      }
    });
}

void PassthroughTrajectoryController::OpenInputTrace()
{
  // Optional: dump the raw input trajectory (the sparse FollowJointTrajectory
  // waypoints, before interpolation) to CSV so the commanded profile can be compared
  // offline against the hardware's MovePVT trace. Opt-in via env
  // RAPIDCODE_TRAJ_INPUT_CSV=<path>; off by default (no param/URDF change). Written
  // only in HandleAccepted (executor thread), so this file I/O never touches update().
  if (input_csv_ != nullptr) { return; }
  const char * csv_path = std::getenv("RAPIDCODE_TRAJ_INPUT_CSV");
  if (csv_path == nullptr || csv_path[0] == '\0') { return; }
  input_csv_ = std::fopen(csv_path, "w");
  if (input_csv_ == nullptr)
  {
    RCLCPP_WARN(get_node()->get_logger(),
      "on_activate: could not open RAPIDCODE_TRAJ_INPUT_CSV='%s'", csv_path);
    return;
  }
  input_seq_ = 0;
  input_goal_id_ = 0;
  WriteInputTraceHeader();
  RCLCPP_INFO(
    get_node()->get_logger(), "on_activate: tracing input trajectories to %s", csv_path);
}

controller_interface::CallbackReturn PassthroughTrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  EmitLifecycleStop();  // committed motion is cut, not run out
  TearDownPipelines();
  return controller_interface::CallbackReturn::SUCCESS;
}

void PassthroughTrajectoryController::TearDownPipelines()
{
  if (goal_handle_timer_) { goal_handle_timer_->cancel(); goal_handle_timer_.reset(); }
  AbortAllMonitoredGoals();
  { std::lock_guard<std::mutex> lock(incoming_mutex_); incoming_.clear(); }
  feeding_.reset();
  pending_.clear();
  inflight_.clear();
  have_last_end_ = false;
  ResetOnlineState();  // also clears online_busy_
  action_busy_.store(false);
  if (input_csv_ != nullptr) { std::fclose(input_csv_); input_csv_ = nullptr; }
}

controller_interface::CallbackReturn PassthroughTrajectoryController::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  // Shutdown straight from ACTIVE skips on_deactivate, so the stop is written here.
  // From INACTIVE the group is already at rest and the interfaces are released, so
  // nothing is emitted. The teardown below is idempotent.
  if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    EmitLifecycleStop();
  }
  TearDownPipelines();
  return controller_interface::CallbackReturn::SUCCESS;
}

void PassthroughTrajectoryController::EmitLifecycleStop()
{
  // The command interfaces are still claimed inside the lifecycle callbacks; the
  // controller manager releases them after the callback returns, and the hardware's
  // write() runs later in the same control cycle. The mailbox accepts any sequence
  // above its last ack, so an un-acked chunk in the slot is overwritten -- that motion
  // is being discarded anyway. The hardware's HandleStop then EStop()s the group and
  // clears its FIFO, so nothing queued behind the slot runs either.
  if (ci_command_ < 0 || ci_command_sequence_ < 0) { return; }  // never activated
  EmitCommand(Cmd::Stop, /*emit*/ true);
  stop_latched_.store(true);      // held until ~/reset_fault, like an operator stop
  stop_requested_.store(false);
  stop_command_pending_ = false;  // the Stop is already in the slot
}

void PassthroughTrajectoryController::AbortAllMonitoredGoals()
{
  // Every accepted goal is registered in monitored_goals_, so this covers
  // feeding_/pending_/inflight_ too (their queues are cleared by the caller).
  std::lock_guard<std::mutex> lock(monitored_mutex_);
  for (auto & monitored : monitored_goals_)
  {
    if (monitored.terminal && monitored.terminal->load()) { continue; }  // already reported
    if (monitored.goal)
    {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
      result->error_string = "controller deactivated mid-trajectory";
      monitored.goal->setAborted(result);
      monitored.goal->runNonRealtime();
    }
    if (monitored.terminal) { monitored.terminal->store(true); }
  }
  monitored_goals_.clear();
}

controller_interface::return_type PassthroughTrajectoryController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const HardwareState hardware = ReadHardwareState();
  hardware_error_code_.store(hardware.error_code);  // mirror for the ~/reset_fault service

  // Pull any newly-accepted goals into the RT pending_ queue, then finish every
  // in-flight goal the hardware reports fully executed.
  DrainIncomingGoals();
  RetireCompletedGoals(hardware.completed_id);

  // Hardware fault: abort everything, hold at Cmd::None (or emit the operator's
  // acknowledged Reset) until the error clears.
  if (hardware.error_code != 0)
  {
    HandleHardwareFault(hardware.prev_acked);
    return controller_interface::return_type::OK;
  }
  // Operator stop latch (~/stop): owns every healthy cycle until ~/reset_fault
  // releases it. Ordered after the fault branch (a hardware fault outranks the hold).
  if (ServiceOperatorStop(hardware.prev_acked))
  {
    PublishArbitration();
    return controller_interface::return_type::OK;
  }

  // Healthy cycle: a reset request is meaningful only while faulted -- discard a stale
  // one so it can never emit a Reset (which would drop a queued trajectory tail) later.
  if (reset_requested_.load()) { reset_requested_.store(false); }
  fault_warn_countdown_ = 0;

  // Command to emit this cycle. command_sequence_ is bumped ONLY when emit_command is set
  // (in EmitCommand), so the prev_acked gate stays meaningful (an every-cycle bump would
  // deadlock the handshake).
  Cmd command = Cmd::None;
  bool emit_command = false;

  // --- online (jog) branch ----------------------------------------------------
  // While a jog is streaming it owns the single-slot mailbox; the action pipeline stays
  // idle (they are mutually exclusive). ServiceOnlineStream sets command/emit when it
  // emits a BEGIN/AppendChunk, so a cycle it drove (or a still-active hold) skips the
  // action pipeline below.
  bool online_owns_cycle = false;
  if (online_enabled_)
  {
    // Committed depth (points accepted but not executed, firmware + host FIFO): the
    // signal the online feed gate bounds against its low/high-water band. One read()
    // cycle stale, which the low-water floor absorbs.
    const int committed_depth = protocol::Decode<int>(ReadStateValue(si_committed_depth_));
    ServiceOnlineStream(hardware.prev_acked, period.seconds(), committed_depth,
      command, emit_command);
    // online_active_ covers a HOLD cycle (active, no command); emit_command covers the
    // one cycle the finalizing stop-tail's last chunk goes out and clears online_active_.
    online_owns_cycle = online_active_ || emit_command;
  }

  // --- action branch -----------------------------------------------------------
  // Runs only when the online branch didn't take the mailbox this cycle and the
  // hardware has consumed the previous command (the single-slot handshake).
  if (!online_owns_cycle && hardware.prev_acked)
  {
    ServiceActionStream(command, emit_command);
  }

  EmitCommand(command, emit_command);
  PublishArbitration();

  // FollowJointTrajectory feedback: the RT path only records a lock-free snapshot here.
  // The goal-monitor timer builds and publishes the message (PublishGoalFeedback), so
  // no allocation or mutex is added to update(). An earlier version built the message
  // on this thread and was removed for the jitter it caused.
  UpdateFeedbackSnapshot(period);

  return controller_interface::return_type::OK;
}

PassthroughTrajectoryController::HardwareState
PassthroughTrajectoryController::ReadHardwareState() const
{
  // const bool is_moving = protocol::Decode<bool>(ReadStateValue(si_is_moving_));
  HardwareState hardware;
  hardware.ack_sequence = protocol::Decode<uint64_t>(ReadStateValue(si_ack_sequence_));
  hardware.error_code = protocol::Decode<int>(ReadStateValue(si_error_code_));
  hardware.completed_id =
    protocol::Decode<uint64_t>(ReadStateValue(si_completed_trajectory_id_));
  // Single-slot mailbox handshake: the hardware has consumed the last command we issued
  // once it echoes our sequence back. Gates every BEGIN/AppendChunk so we never overwrite
  // an un-consumed command.
  hardware.prev_acked = (hardware.ack_sequence == command_sequence_);
  return hardware;
}

void PassthroughTrajectoryController::RetireCompletedGoals(const uint64_t completed_id)
{
  // In-flight goals are in ascending-id order, so finish from the front while the
  // hardware's completed id covers them. Only the goal that ends the chain at rest gets
  // the live goal-tolerance check; mid-chain the arm has already moved past its endpoint.
  while (!inflight_.empty() && inflight_.front().trajectory_id != 0 &&
    inflight_.front().trajectory_id <= completed_id)
  {
    const GoalEntry entry = inflight_.front();
    inflight_.pop_front();
    FinishGoal(entry, /*do_tolerance*/ entry.traj->finalize);
  }
}

void PassthroughTrajectoryController::HandleHardwareFault(const bool prev_acked)
{
  // Abort any goals that arrive while faulted so clients don't hang.
  AbortAllGoals(FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED,
    "hardware in error state (e-stop / starvation / bad point); goals not accepted at this time");
  ResetOnlineState();  // the open jog move (if any) is gone with the fault; also clears online_busy_
  action_busy_.store(false);

  // Surface the (otherwise silent) fault hold, throttled by cycle count so the RT
  // branch stays clock-free. The counter re-arms on any healthy cycle.
  if (++fault_warn_countdown_ >= kFaultHoldWarnPeriodCycles)
  {
    fault_warn_countdown_ = 0;
    RCLCPP_WARN(get_node()->get_logger(),
      "holding: hardware error_code=%d; jog/goals ignored. After fixing the drive "
      "(ClearFaults), call ~/reset_fault to clear the latch.",
      hardware_error_code_.load());
  }

  // Operator acknowledgement: emit Cmd::Reset through the normal mailbox handshake.
  // The hardware's HandleReset clears the latched error_code, so the next cycle reads
  // 0 and leaves this branch. An un-acked mailbox leaves the request pending; it
  // retries next cycle.
  if (reset_requested_.load() && prev_acked)
  {
    reset_requested_.store(false);
    // One acknowledgement re-arms everything: a stop latched before (or during) the
    // fault is released too, and its now-moot Cmd::Stop is dropped -- emitting it
    // after this Reset would park the freshly re-armed group in STOPPED.
    stop_latched_.store(false);
    stop_requested_.store(false);
    stop_command_pending_ = false;
    EmitCommand(Cmd::Reset, /*emit*/ true);
    return;
  }
  EmitCommand(Cmd::None, /*emit*/ false);
}

bool PassthroughTrajectoryController::ServiceOperatorStop(const bool prev_acked)
{
  if (!stop_latched_.load()) { return false; }

  // Every latched cycle, like the fault hold: a goal that raced the latch is aborted
  // (not left hanging on a result), the open jog session stays torn down, and any
  // staged jog snapshot is marked seen -- it predates the stop, so it is stale.
  AbortAllGoals(FollowJointTrajectory::Result::INVALID_GOAL,
    "operator stop (~/stop) latched; call ~/reset_fault to accept goals again");
  ResetOnlineState();  // also clears online_busy_
  action_busy_.store(false);
  MarkStagedSnapshotSeen();
  if (stop_requested_.exchange(false)) { stop_command_pending_ = true; }

  // The stop itself: one Cmd::Stop, retried until the mailbox is free. Overwriting is
  // not an option -- EmitCommand's sequence handshake needs the previous ack.
  if (stop_command_pending_ && prev_acked)
  {
    stop_command_pending_ = false;
    EmitCommand(Cmd::Stop, /*emit*/ true);
    return true;
  }
  // Operator release: Cmd::Reset re-arms the hardware (its HandleReset also clears
  // the commanded STOPPED group state), then the latch drops. Gated behind the
  // pending Stop so a release can never overtake the stop itself.
  if (reset_requested_.load() && !stop_command_pending_ && prev_acked)
  {
    reset_requested_.store(false);
    stop_latched_.store(false);
    EmitCommand(Cmd::Reset, /*emit*/ true);
    return true;
  }
  EmitCommand(Cmd::None, /*emit*/ false);
  return true;
}

void PassthroughTrajectoryController::ServiceActionStream(Cmd & command, bool & emit)
{
  if (!feeding_)
  {
    TryAdoptPendingGoal(command, emit);
    return;
  }
  FeedActionChunk(command, emit);
}

bool PassthroughTrajectoryController::FirstPointContinuous(const GoalTrajectory & traj) const
{
  if (traj.num_points == 0) { return false; }
  // For the first goal of an idle chain, check against the live joint state; for a goal
  // queued behind others (arm mid-motion), check against the previous adopted goal's
  // endpoint -- the seam MoveIt planned to.
  const bool use_seam = !inflight_.empty() && have_last_end_;
  const std::size_t joint_count = joint_names_.size();
  for (std::size_t joint = 0; joint < joint_count; ++joint)
  {
    const double reference =
      use_seam ? last_adopted_end_[joint] : ReadStateValue(si_joint_position_[joint]);
    if (std::abs(traj.positions[joint] - reference) > first_point_tolerance_) { return false; }
  }
  return true;
}

void PassthroughTrajectoryController::RecordAdoptedEndSeam(const GoalTrajectory & traj)
{
  const std::size_t joint_count = joint_names_.size();
  last_adopted_end_.assign(joint_count, 0.0);
  const std::size_t last = static_cast<std::size_t>(traj.num_points - 1) * joint_count;
  for (std::size_t joint = 0; joint < joint_count; ++joint)
  {
    last_adopted_end_[joint] = traj.positions[last + joint];
  }
  have_last_end_ = true;
}

void PassthroughTrajectoryController::TryAdoptPendingGoal(Cmd & command, bool & emit)
{
  if (pending_.empty()) { return; }
  GoalEntry entry = std::move(pending_.front());
  pending_.pop_front();

  // First-point continuity precondition: reject (abort the goal), don't e-stop. The
  // cycle that aborts adopts nothing; the next pending goal is tried next cycle.
  if (!FirstPointContinuous(*entry.traj))
  {
    AbortGoal(entry, FollowJointTrajectory::Result::INVALID_GOAL,
      "first trajectory point is not within tolerance of the expected start state");
    return;
  }

  // Adopt: assign a monotonic id and emit BEGIN with size + valid_fields + id, then
  // record this goal's endpoint as the seam for whatever is queued behind it.
  entry.trajectory_id = next_trajectory_id_++;
  if (entry.assigned_id) { entry.assigned_id->store(entry.trajectory_id, std::memory_order_relaxed); }
  entry.next_index = 0;
  SetCommandValue(ci_trajectory_id_, protocol::Encode(entry.trajectory_id));
  SetCommandValue(ci_trajectory_size_,
    protocol::Encode(static_cast<int>(entry.traj->num_points)));
  SetCommandValue(ci_valid_fields_, protocol::Encode(entry.traj->valid_fields));
  RecordAdoptedEndSeam(*entry.traj);

  feeding_ = std::move(entry);
  command = Cmd::Begin;
  emit = true;
}

void PassthroughTrajectoryController::FeedActionChunk(Cmd & command, bool & emit)
{
  const int num_points = static_cast<int>(feeding_->traj->num_points);
  const int base = feeding_->next_index;
  const int len = std::min(chunk_size_, num_points - base);
  const bool is_last = (base + len == num_points);
  // Tell the firmware to finalize the trajectory if this is the last chunk and the
  // goal requested it (the auto queued-behind policy may have withheld finalize).
  const bool final_move = is_last && feeding_->traj->finalize;
  PresentChunk(*feeding_->traj, base, len, final_move);
  command = Cmd::AppendChunk;
  emit = true;
  feeding_->next_index = base + len;
  if (is_last)
  {
    // Every chunk sent AND acked: move to the in-flight set and pick up the next
    // pending goal on a following cycle. Completion is reported by id, not is_moving.
    inflight_.push_back(std::move(*feeding_));
    feeding_.reset();
  }
}

void PassthroughTrajectoryController::EmitCommand(const Cmd command, const bool emit)
{
  if (!emit)
  {
    SetCommandValue(ci_command_, protocol::Encode(static_cast<int>(Cmd::None)));
    return;
  }
  ++command_sequence_;
  SetCommandValue(ci_command_, protocol::Encode(static_cast<int>(command)));
  SetCommandValue(ci_command_sequence_, protocol::Encode(command_sequence_));
}

void PassthroughTrajectoryController::PublishArbitration()
{
  // Lock-free mirrors of the RT-owned pipeline state: the topic callback drops jog
  // messages while action_busy_, HandleGoal rejects action goals while online_busy_.
  action_busy_.store(ActionBusy());
  online_busy_.store(online_active_);
}

void PassthroughTrajectoryController::DrainIncomingGoals()
{
  std::unique_lock<std::mutex> lock(incoming_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) { return; }  // executor is pushing; drain next cycle instead
  while (!incoming_.empty())
  {
    pending_.push_back(std::move(incoming_.front()));
    incoming_.pop_front();
  }
}

bool PassthroughTrajectoryController::WithinGoalTolerance(const GoalTrajectory & traj) const
{
  if (traj.num_points == 0) { return true; }
  const std::size_t joint_count = joint_names_.size();
  const std::size_t last = static_cast<std::size_t>(traj.num_points - 1) * joint_count;
  for (std::size_t joint = 0; joint < joint_count; ++joint)
  {
    const double state_pos = ReadStateValue(si_joint_position_[joint]);
    if (std::abs(traj.positions[last + joint] - state_pos) > goal_position_tolerance_)
    {
      return false;
    }
  }
  return true;
}

void PassthroughTrajectoryController::FinishGoal(const GoalEntry & entry, bool do_tolerance)
{
  if (entry.goal)
  {
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    if (!do_tolerance || (entry.traj && WithinGoalTolerance(*entry.traj)))
    {
      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      entry.goal->setSucceeded(result);
    }
    else
    {
      result->error_code = FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
      result->error_string = "final position outside goal tolerance";
      entry.goal->setAborted(result);
    }
  }
  if (entry.terminal) { entry.terminal->store(true); }
}

void PassthroughTrajectoryController::AbortGoal(
  const GoalEntry & entry, int32_t result_code, const std::string & message)
{
  if (entry.goal)
  {
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = result_code;
    result->error_string = message;
    entry.goal->setAborted(result);
  }
  if (entry.terminal) { entry.terminal->store(true); }
}

void PassthroughTrajectoryController::AbortAllGoals(int32_t result_code, const std::string & message)
{
  if (feeding_) { AbortGoal(*feeding_, result_code, message); feeding_.reset(); }
  for (const auto & entry : inflight_) { AbortGoal(entry, result_code, message); }
  inflight_.clear();
  for (const auto & entry : pending_) { AbortGoal(entry, result_code, message); }
  pending_.clear();
}

void PassthroughTrajectoryController::PresentChunk(
  const GoalTrajectory & traj, int base, int len, bool isFinal)
{
  const std::size_t joint_count = joint_names_.size();
  // valid_fields is per-trajectory (same for every point); the SoA arrays are laid
  // out point-major / joint-minor as [point*n + joint].
  SetCommandValue(ci_valid_fields_, protocol::Encode(traj.valid_fields));
  SetCommandValue(ci_chunk_base_, protocol::Encode(base));
  SetCommandValue(ci_chunk_len_, protocol::Encode(len));
  SetCommandValue(ci_chunk_final_, isFinal ? 1.0 : 0.0);
  for (int slot = 0; slot < len; ++slot)
  {
    const std::size_t point = static_cast<std::size_t>(base + slot);  // global point (SoA source)
    SetCommandValue(ci_slot_time_[static_cast<std::size_t>(slot)], traj.durations[point]);
    for (std::size_t joint = 0; joint < joint_count; ++joint)
    {
      const std::size_t src = point * joint_count + joint;                        // SoA source index
      const std::size_t dest = static_cast<std::size_t>(slot) * joint_count + joint;  // command-slot dest index
      SetCommandValue(ci_slot_joint_position_[dest], traj.positions[src]);
      SetCommandValue(ci_slot_joint_velocity_[dest], traj.velocities[src]);
      SetCommandValue(ci_slot_joint_acceleration_[dest], traj.accelerations[src]);
      SetCommandValue(ci_slot_joint_jerk_[dest], traj.jerks[src]);
    }
  }
}

void PassthroughTrajectoryController::UpdateFeedbackSnapshot(const rclcpp::Duration & period)
{
  // The firmware executes goals in id order, so the executing goal is the oldest one
  // not yet completed: the head of inflight_, or feeding_ when nothing is in flight yet.
  // feeding_ has an id only after its BEGIN was emitted.
  const GoalEntry * head = nullptr;
  if (!inflight_.empty()) { head = &inflight_.front(); }
  else if (feeding_ && feeding_->trajectory_id != 0) { head = &*feeding_; }

  if (head == nullptr)
  {
    feedback_head_id_ = 0;
    feedback_head_elapsed_ = 0.0;
    feedback_snapshot_.trajectory_id.store(0, std::memory_order_relaxed);
    return;
  }
  if (head->trajectory_id != feedback_head_id_)
  {
    // New head: it starts executing when the previous head completes (or at its own
    // BEGIN when the chain was idle). Either way, the clock starts now.
    feedback_head_id_ = head->trajectory_id;
    feedback_head_elapsed_ = 0.0;
  }
  else
  {
    feedback_head_elapsed_ += period.seconds();
  }

  const std::size_t joint_count =
    std::min(joint_names_.size(), feedback_snapshot_.actual_positions.size());
  for (std::size_t joint = 0; joint < joint_count; ++joint)
  {
    feedback_snapshot_.actual_positions[joint].store(
      ReadStateValue(si_joint_position_[joint]), std::memory_order_relaxed);
  }
  feedback_snapshot_.elapsed_seconds.store(feedback_head_elapsed_, std::memory_order_relaxed);
  // Publish the id last so a reader that sees it also sees this cycle's values.
  feedback_snapshot_.trajectory_id.store(feedback_head_id_, std::memory_order_release);
}

void PassthroughTrajectoryController::PublishGoalFeedback(
  const MonitoredGoal & monitored, std::uint64_t /*executing_id*/, double elapsed_seconds)
{
  if (!monitored.goal || !monitored.traj || monitored.traj->num_points == 0) { return; }
  const GoalTrajectory & traj = *monitored.traj;
  const std::size_t joint_count = joint_names_.size();
  if (traj.num_joints != joint_count || feedback_snapshot_.actual_positions.size() != joint_count)
  {
    return;
  }

  // Sample the trajectory at the elapsed time: the last point whose start time is not
  // after `elapsed`. durations[i] is the time spent on point i, so point i starts at the
  // sum of durations[0..i-1]. Clamped to the final point once the motion has overrun.
  std::size_t idx = 0;
  double start = 0.0;
  while (idx + 1 < traj.num_points && start + traj.durations[idx] <= elapsed_seconds)
  {
    start += traj.durations[idx];
    ++idx;
  }
  const std::size_t base = idx * joint_count;

  auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
  feedback->header.stamp = get_node()->now();
  feedback->joint_names = joint_names_;
  feedback->desired.positions.resize(joint_count);
  feedback->actual.positions.resize(joint_count);
  feedback->error.positions.resize(joint_count);
  const auto elapsed = rclcpp::Duration::from_seconds(elapsed_seconds);
  feedback->desired.time_from_start = elapsed;
  feedback->actual.time_from_start = elapsed;
  feedback->error.time_from_start = elapsed;
  for (std::size_t joint = 0; joint < joint_count; ++joint)
  {
    const double actual =
      feedback_snapshot_.actual_positions[joint].load(std::memory_order_relaxed);
    const double want = traj.positions[base + joint];
    feedback->desired.positions[joint] = want;
    feedback->actual.positions[joint] = actual;
    feedback->error.positions[joint] = want - actual;
  }
  // Same thread as runNonRealtime(), and update() no longer touches this handle's
  // mutex, so the try_lock inside setFeedback() succeeds; the return value is ignored.
  monitored.goal->setFeedback(feedback);
}

const char * PassthroughTrajectoryController::IntakeLatchReason() const
{
  if (stop_latched_.load())  // operator or lifecycle stop: sticky until ~/reset_fault
  {
    return "operator stop (~/stop) latched; call ~/reset_fault first";
  }
  if (hardware_error_code_.load() != 0)  // hardware fault: latched until ~/reset_fault
  {
    return "hardware fault latched (error_code != 0); clear the drive, then call ~/reset_fault";
  }
  return nullptr;
}

rclcpp_action::GoalResponse PassthroughTrajectoryController::HandleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
  if (const char * latched = IntakeLatchReason())
  {
    RCLCPP_WARN(get_node()->get_logger(), "Goal rejected: %s.", latched);
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Online XOR action: a jog owns the mailbox exclusively, so reject (don't preempt) a
  // new action goal while one is streaming. online_busy_ is the lock-free mirror of the
  // RT-owned online_active_ (published each update() cycle).
  if (online_busy_.load())
  {
    RCLCPP_WARN(get_node()->get_logger(),
      "Goal rejected: an online jog stream is active on ~/joint_trajectory.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Multi-goal: accept concurrent goals up to a bound (backpressure at the action layer;
  // the hardware FIFO cap is the deeper backstop).
  const std::size_t active = ActiveMonitoredGoalCount();
  if (active >= kMaxInFlightGoals)
  {
    RCLCPP_WARN(get_node()->get_logger(),
      "Goal rejected: %zu goals already in flight (max %zu).", active, kMaxInFlightGoals);
    return rclcpp_action::GoalResponse::REJECT;
  }

  const auto & traj = goal->trajectory;
  if (traj.joint_names.size() != joint_names_.size())
  {
    RCLCPP_WARN(get_node()->get_logger(),
      "Goal rejected: trajectory has %zu joints, controller manages %zu.",
      traj.joint_names.size(), joint_names_.size());
    return rclcpp_action::GoalResponse::REJECT;
  }
  // Structural checks shared with the online intake. (MovePVT still receives
  // synthesized P/V/A/J regardless of the chosen interpolation order.)
  std::string reason;
  if (!ValidateTrajectoryStructure(traj, joint_names_, RequiredFields(interpolation_), reason))
  {
    RCLCPP_WARN(get_node()->get_logger(),
      "Goal rejected ('%s' interpolation): %s.",
      InterpolationName(interpolation_), reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

std::size_t PassthroughTrajectoryController::ActiveMonitoredGoalCount()
{
  // Count only goals not yet reported terminal -- the timer prunes finished ones, so
  // don't prune here (that would drop a result the timer hasn't flushed yet).
  std::lock_guard<std::mutex> lock(monitored_mutex_);
  std::size_t active = 0;
  for (const auto & monitored : monitored_goals_)
  {
    if (!(monitored.terminal && monitored.terminal->load())) { ++active; }
  }
  return active;
}

rclcpp_action::CancelResponse PassthroughTrajectoryController::HandleCancel(
  std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  cancel_requested_.store(true);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PassthroughTrajectoryController::HandleAccepted(std::shared_ptr<GoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto traj = RemapTrajectory(*goal);
  if (!traj)
  {
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = FollowJointTrajectory::Result::INVALID_JOINTS;
    result->error_string = "could not remap trajectory joints";
    goal_handle->abort(result);
    return;
  }
  // Read the flag LIVE (not the on_configure cache) so a chunked/streaming sender can
  // flip the `finalize_last_chunk` parameter between goals: false while more goals
  // follow (the firmware move stays open across the seam -- no rest dwell), true on
  // the chain's last goal (the move closes and the goal-tolerance check runs). This
  // executor-thread read is the per-goal "finalize hint" of the streamed-recipe design.
  traj->finalize = get_node()->get_parameter("finalize_last_chunk").as_bool();
  LogInputTrajectory(*goal);  // opt-in trace; a no-op unless RAPIDCODE_TRAJ_INPUT_CSV is set
  auto rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
  rt_goal->execute();
  auto terminal = std::make_shared<std::atomic<bool>>(false);
  auto assigned_id = std::make_shared<std::atomic<std::uint64_t>>(0);

  GoalEntry entry;
  entry.traj = traj;  // shared with the monitored entry (feedback `desired`)
  entry.goal = rt_goal;
  entry.terminal = terminal;
  entry.assigned_id = assigned_id;

  // Register with the monitor timer FIRST (so the result can always be flushed), then
  // hand the goal to the RT pipeline. The RT thread finishes/aborts via the shared
  // `terminal` flag, and the timer prunes the monitored entry once it is flushed.
  {
    std::lock_guard<std::mutex> lock(monitored_mutex_);
    monitored_goals_.push_back(MonitoredGoal{rt_goal, terminal, traj, assigned_id});
  }
  {
    std::lock_guard<std::mutex> lock(incoming_mutex_);
    incoming_.push_back(std::move(entry));
  }
}

template<std::size_t NumPowers = 5>
constexpr std::array<double, NumPowers + 1> ComputePowers(const double value)
{
  std::array<double, NumPowers + 1> powers;
  powers[0] = 1.0;
  for (std::size_t power = 1; power <= NumPowers; ++power)
  {
    powers[power] = powers[power - 1] * value;
  }
  return powers;
}

// Linear fit through the segment's position endpoints (JTC's linear branch): velocity is
// the constant secant slope, acceleration and jerk are zero. duration[1] is the segment
// length, which SampleTrajectoryPoint has already guaranteed > 0. The unused higher-order
// coefficients are zeroed so the shared degree-5 evaluators below read them as 0.
static void FillLinearCoefficients(
  std::array<double, 6>& coefficients, const std::array<double, 6>& duration,
  const double start_pos, const double end_pos)
{
  coefficients.fill(0.0);
  coefficients[0] = start_pos;
  coefficients[1] = (duration[1] > 0.0) ? (end_pos - start_pos) / duration[1] : 0.0;
}

// Quadratic fit: velocity is interpolated linearly from start_vel to end_vel, so acceleration
// is a single CONSTANT per segment (a trapezoidal profile -- no oscillation, no velocity
// overshoot), discontinuous at the knots. Uses only start_pos + both velocities; it ignores
// the endpoint position and accelerations, so position can drift from the knots at an
// acceleration corner (zero drift where accel is already constant). coefficients[3..5] are
// zeroed so the shared degree-5 evaluators read them as 0.
static void FillQuadraticCoefficients(
  std::array<double, 6>& coefficients, const std::array<double, 6>& duration,
  const double start_pos, const double start_vel, const double end_vel)
{
  coefficients.fill(0.0);
  coefficients[0] = start_pos;
  coefficients[1] = start_vel;
  coefficients[2] = (duration[1] > 0.0) ? 0.5 * (end_vel - start_vel) / duration[1] : 0.0;
}

// Cubic Hermite through the segment's (position, velocity) endpoints -- identical to
// ros2_controllers JointTrajectoryController's cubic branch. Acceleration is linear
// (discontinuous at the knots) and jerk is constant across the segment. coefficients[4..5]
// are zeroed so the shared degree-5 evaluators read them as 0.
static void FillCubicCoefficients(
  std::array<double, 6>& coefficients, const std::array<double, 6>& duration,
  const double start_pos, const double start_vel,
  const double end_pos, const double end_vel)
{
  coefficients.fill(0.0);
  coefficients[0] = start_pos;
  coefficients[1] = start_vel;
  coefficients[2] = (-3.0 * start_pos + 3.0 * end_pos
                      - 2.0 * start_vel * duration[1] - end_vel * duration[1]) / duration[2];
  coefficients[3] = (2.0 * start_pos - 2.0 * end_pos
                      + start_vel * duration[1] + end_vel * duration[1]) / duration[3];
}

static void FillQuinticCoefficients(
  std::array<double, 6>& coefficients, const std::array<double, 6>& duration,
  const double start_pos, const double start_vel, const double start_acc,
  const double end_pos, const double end_vel, const double end_acc)
{
  coefficients[0] = start_pos;
  coefficients[1] = start_vel;
  coefficients[2] = start_acc * 0.5;
  coefficients[3] = (end_acc * duration[2] - 3.0 * start_acc * duration[2]
                      - 8.0 * end_vel * duration[1] - 12.0 * start_vel * duration[1]
                      + 20.0 * end_pos - 20.0 * start_pos) / (2.0 * duration[3]);
  coefficients[4] = (-2.0 * end_acc * duration[2] + 3.0 * start_acc * duration[2]
                      + 14.0 * end_vel * duration[1] + 16.0 * start_vel * duration[1]
                      - 30.0 * end_pos + 30.0 * start_pos) / (2.0 * duration[4]);
  coefficients[5] = (end_acc * duration[2] - start_acc * duration[2]
                      - 6.0 * end_vel * duration[1] - 6.0 * start_vel * duration[1]
                      + 12.0 * end_pos - 12.0 * start_pos) / (2.0 * duration[5]);
}

static double InterpolateQuinticPosition(
  const std::array<double, 6>& coefficients, const std::array<double, 6>& duration)
{
  return coefficients[0] +
    coefficients[1] * duration[1] + coefficients[2] * duration[2] +
    coefficients[3] * duration[3] + coefficients[4] * duration[4] +
    coefficients[5] * duration[5];
}

static double InterpolateQuinticVelocity(
  const std::array<double, 6>& coefficients, const std::array<double, 6>& duration)
{
  return coefficients[1] +
    2.0 * coefficients[2] * duration[1] + 3.0 * coefficients[3] * duration[2] +
    4.0 * coefficients[4] * duration[3] + 5.0 * coefficients[5] * duration[4];
}

static double InterpolateQuinticAcceleration(
  const std::array<double, 6>& coefficients, const std::array<double, 6>& duration)
{
  return 2.0 * coefficients[2] +
    6.0 * coefficients[3] * duration[1] + 12.0 * coefficients[4] * duration[2] +
    20.0 * coefficients[5] * duration[3];
}

static double InterpolateQuinticJerk(
  const std::array<double, 6>& coefficients, const std::array<double, 6>& duration)
{
  return 6.0 * coefficients[3] +
    24.0 * coefficients[4] * duration[1] + 60.0 * coefficients[5] * duration[2];
}

// Resolve Auto to a concrete order from the fields present on both segment endpoints,
// mirroring JTC's has_velocity/has_accel test; forced orders pass through unchanged.
static Interpolation ResolveInterpolation(const Interpolation configured,
  const trajectory_msgs::msg::JointTrajectoryPoint& start,
  const trajectory_msgs::msg::JointTrajectoryPoint& end, const std::size_t num_joints)
{
  if (configured != Interpolation::Auto) { return configured; }
  const bool has_velocity =
    start.velocities.size() == num_joints && end.velocities.size() == num_joints;
  const bool has_acceleration =
    start.accelerations.size() == num_joints && end.accelerations.size() == num_joints;
  if (has_velocity && has_acceleration) { return Interpolation::Quintic; }
  if (has_velocity) { return Interpolation::Cubic; }
  return Interpolation::Linear;
}

// Fit one degree-5-capable polynomial per joint to the segment endpoints, dispatching on the
// already-resolved interpolation order. Shared by SampleTrajectoryPoint (which samples the
// polynomial) and LogInputTrajectory (which traces the coefficients), so the logged
// coefficients always match the sampled ones. duration_powers[k] == duration^k.
static std::vector<std::array<double, 6>> ComputeSegmentCoefficients(
  const trajectory_msgs::msg::JointTrajectoryPoint& start,
  const trajectory_msgs::msg::JointTrajectoryPoint& end,
  const std::array<double, 6>& duration_powers, const Interpolation method,
  const std::vector<int>& joint_mapping)
{
  const std::size_t num_joints = joint_mapping.size();
  std::vector<std::array<double, 6>> joint_coefficients(num_joints);
  for (std::size_t joint = 0; joint < num_joints; ++joint)
  {
    const int mapped_index = joint_mapping[joint];
    switch (method)
    {
      case Interpolation::Linear:
        FillLinearCoefficients(joint_coefficients[joint], duration_powers,
          start.positions[mapped_index], end.positions[mapped_index]);
        break;
      case Interpolation::Quadratic:
        FillQuadraticCoefficients(joint_coefficients[joint], duration_powers,
          start.positions[mapped_index], start.velocities[mapped_index],
          end.velocities[mapped_index]);
        break;
      case Interpolation::Cubic:
        FillCubicCoefficients(joint_coefficients[joint], duration_powers,
          start.positions[mapped_index], start.velocities[mapped_index],
          end.positions[mapped_index], end.velocities[mapped_index]);
        break;
      case Interpolation::Auto:  // caller resolves Auto; fall through to the safe default
      case Interpolation::Quintic:
        FillQuinticCoefficients(
          joint_coefficients[joint], duration_powers,
          start.positions[mapped_index], start.velocities[mapped_index], start.accelerations[mapped_index],
          end.positions[mapped_index], end.velocities[mapped_index], end.accelerations[mapped_index]);
        break;
    }
  }
  return joint_coefficients;
}

static void SampleTrajectoryPoint(const trajectory_msgs::msg::JointTrajectoryPoint& start,
  const trajectory_msgs::msg::JointTrajectoryPoint& end, const double duration,
  const double sample_period, const Interpolation method,
  const std::vector<int>& joint_mapping, GoalTrajectory& output_traj)
{
  if (duration <= 0.0) { return; }  // skip degenerate / zero-length segments
  const std::size_t num_joints = joint_mapping.size();
  const auto duration_powers = ComputePowers<5>(duration);

  // One polynomial per joint, fit to the segment endpoints (shared with the input trace so
  // the logged coefficients match the sampled ones). `method` is already resolved to a
  // concrete order by the caller.
  const std::vector<std::array<double, 6>> joint_coefficients =
    ComputeSegmentCoefficients(start, end, duration_powers, method, joint_mapping);

  // Half-open [0, duration): emit a sample every sample_period. An integer count
  // avoids float-accumulation drift; the segment end (t == duration) is emitted as
  // the NEXT segment's first sample, so there is no duplicate frame. The very last
  // knot has no next segment -> AppendKnot supplies it once after the loop.
  const int sample_count = std::max(1, static_cast<int>(std::ceil(duration / sample_period)));
  for (int sample = 0; sample < sample_count; ++sample)
  {
    const auto powers = ComputePowers<5>(static_cast<double>(sample) * sample_period);
    output_traj.durations.push_back(sample_period);
    for (std::size_t joint = 0; joint < num_joints; ++joint)
    {
      output_traj.positions.push_back(InterpolateQuinticPosition(joint_coefficients[joint], powers));
      output_traj.velocities.push_back(InterpolateQuinticVelocity(joint_coefficients[joint], powers));
      output_traj.accelerations.push_back(InterpolateQuinticAcceleration(joint_coefficients[joint], powers));
      output_traj.jerks.push_back(InterpolateQuinticJerk(joint_coefficients[joint], powers));
    }
  }
}

// Append the trajectory's final knot, which no segment's half-open [0, duration)
// sampling emits. Jerk is 0 (the move ends at the planned endpoint, at rest).
static void AppendKnot(const trajectory_msgs::msg::JointTrajectoryPoint& point,
  const double sample_period, const std::vector<int>& joint_mapping, GoalTrajectory& output_traj)
{
  const std::size_t num_joints = joint_mapping.size();
  // A velocity-only (cubic) or position-only (linear) goal may omit the higher fields on
  // the final knot; treat an absent field as zero rather than indexing an empty vector.
  const bool has_velocity = point.velocities.size() == num_joints;
  const bool has_acceleration = point.accelerations.size() == num_joints;
  output_traj.durations.push_back(sample_period);
  for (std::size_t joint = 0; joint < num_joints; ++joint)
  {
    const int mapped_index = joint_mapping[joint];
    output_traj.positions.push_back(point.positions[mapped_index]);
    output_traj.velocities.push_back(has_velocity ? point.velocities[mapped_index] : 0.0);
    output_traj.accelerations.push_back(has_acceleration ? point.accelerations[mapped_index] : 0.0);
    output_traj.jerks.push_back(0.0);
  }
}

std::vector<int> PassthroughTrajectoryController::ResolveJointMapping(
  const trajectory_msgs::msg::JointTrajectory & trajectory) const
{
  const std::size_t num_joints = joint_names_.size();
  std::vector<int> joint_mapping(num_joints, -1);
  for (std::size_t joint = 0; joint < num_joints; ++joint)
  {
    for (std::size_t name_index = 0; name_index < trajectory.joint_names.size(); ++name_index)
    {
      if (trajectory.joint_names[name_index] == joint_names_[joint])
      {
        joint_mapping[joint] = static_cast<int>(name_index);
        break;
      }
    }
    if (joint_mapping[joint] < 0) { return {}; }  // a managed joint isn't in the trajectory
  }
  return joint_mapping;
}

std::vector<int> PassthroughTrajectoryController::ResolveJointMapping(
  const FollowJointTrajectory::Goal & goal) const
{
  return ResolveJointMapping(goal.trajectory);
}

void PassthroughTrajectoryController::WriteInputTraceHeader()
{
  if (input_csv_ == nullptr) { return; }
  // j<i>_* columns match the hardware trace's joint layout; input carries no jerk
  // (trajectory_msgs::msg::JointTrajectoryPoint has no jerk field).
  std::fprintf(input_csv_, "seq,goal_id,point,t_from_start,has_vel,has_acc");
  for (std::size_t joint = 0; joint < joint_names_.size(); ++joint)
  {
    std::fprintf(input_csv_, ",j%zu_pos,j%zu_vel,j%zu_acc", joint, joint, joint);
  }
  // Per-segment interpolation coefficients: the segment that STARTS at each row's point (the
  // final point has no segment -> empty method / nan). Exposes exactly what each
  // Fill*Coefficients produced so the source of any overshoot is visible. c0..c5 are the
  // degree-5 polynomial coefficients (unused high orders are 0 for cubic/quadratic/linear).
  std::fprintf(input_csv_, ",seg_method,seg_T");
  for (std::size_t joint = 0; joint < joint_names_.size(); ++joint)
  {
    std::fprintf(input_csv_, ",j%zu_c0,j%zu_c1,j%zu_c2,j%zu_c3,j%zu_c4,j%zu_c5", joint, joint, joint, joint, joint, joint);
  }
  std::fprintf(input_csv_, "\n");
}

void PassthroughTrajectoryController::LogInputTrajectory(const FollowJointTrajectory::Goal & goal)
{
  if (input_csv_ == nullptr) { return; }
  const std::vector<int> joint_mapping = ResolveJointMapping(goal);
  if (joint_mapping.empty()) { return; }  // logged only when joints map, as in RemapTrajectory

  const auto & traj = goal.trajectory;
  const std::size_t num_joints = joint_names_.size();
  const double nan_value = std::numeric_limits<double>::quiet_NaN();
  const long goal_id = input_goal_id_++;
  for (std::size_t point = 0; point < traj.points.size(); ++point)
  {
    const auto & traj_point = traj.points[point];
    if (traj_point.positions.size() < traj.joint_names.size()) { continue; }  // malformed; skip row
    // A field is "present" only when it is sized for every joint (matches how
    // SampleTrajectoryPoint/ResolveInterpolation decide field presence); else log NaN.
    const bool has_vel = traj_point.velocities.size() == traj.joint_names.size();
    const bool has_acc = traj_point.accelerations.size() == traj.joint_names.size();
    const double t_from_start = rclcpp::Duration(traj_point.time_from_start).seconds();
    std::fprintf(input_csv_, "%ld,%ld,%zu,%.6f,%d,%d",
      input_seq_++, goal_id, point, t_from_start, has_vel ? 1 : 0, has_acc ? 1 : 0);
    for (std::size_t joint = 0; joint < num_joints; ++joint)
    {
      const std::size_t col = static_cast<std::size_t>(joint_mapping[joint]);
      const double pos = traj_point.positions[col];
      const double vel = has_vel ? traj_point.velocities[col] : nan_value;
      const double acc = has_acc ? traj_point.accelerations[col] : nan_value;
      std::fprintf(input_csv_, ",%.9f,%.9f,%.9f", pos, vel, acc);
    }

    // The coefficients of the segment that starts at this point (to its successor). Computed
    // via the SAME helper the sampler uses, so this is exactly what generated the frames. The
    // last point has no segment; degenerate/short segments log empties too.
    bool logged_segment = false;
    if (point + 1 < traj.points.size())
    {
      const auto & seg_end = traj.points[point + 1];
      const double seg_T = rclcpp::Duration(seg_end.time_from_start).seconds() -
        rclcpp::Duration(traj_point.time_from_start).seconds();
      if (seg_T > 0.0 && seg_end.positions.size() >= traj.joint_names.size())
      {
        const Interpolation method = ResolveInterpolation(interpolation_, traj_point, seg_end, num_joints);
        const auto powers = ComputePowers<5>(seg_T);
        const auto coeffs = ComputeSegmentCoefficients(traj_point, seg_end, powers, method, joint_mapping);
        std::fprintf(input_csv_, ",%s,%.6f", InterpolationName(method), seg_T);
        for (std::size_t joint = 0; joint < num_joints; ++joint)
        {
          const auto & coeff = coeffs[joint];
          std::fprintf(input_csv_, ",%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",
            coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5]);
        }
        logged_segment = true;
      }
    }
    if (!logged_segment)
    {
      std::fprintf(input_csv_, ",,nan");  // seg_method (empty), seg_T = nan
      for (std::size_t joint = 0; joint < num_joints; ++joint)
      {
        std::fprintf(input_csv_, ",nan,nan,nan,nan,nan,nan");
      }
    }
    std::fprintf(input_csv_, "\n");
  }
  std::fflush(input_csv_);  // non-RT and only a few rows; keep the file complete per move
}

// --- online (jog) topic path --------------------------------------------------

// Duration (seconds) of a monotonic quintic velocity blend (vel_from -> vel_to) that
// respects the tightest per-joint accel/jerk limit for the largest per-joint velocity
// change. For this profile peak|accel| = 1.5|dv|/T and peak|jerk| = 6|dv|/T^2, so
// T >= 1.5|dv|/a_max and T >= sqrt(6|dv|/j_max); floored/capped to [min_dur, max_dur].
// The decel-to-rest stop-tail is the vel_to = rest special case. Pure.
static double BlendDuration(const std::vector<double> & vel_from,
  const std::vector<double> & vel_to,
  const std::vector<double> & max_acceleration, const std::vector<double> & max_jerk,
  double min_dur, double max_dur)
{
  double duration = min_dur;
  for (std::size_t joint = 0; joint < vel_from.size(); ++joint)
  {
    const double delta = std::abs(vel_to[joint] - vel_from[joint]);
    if (delta <= 0.0) { continue; }
    if (max_acceleration[joint] > 0.0)
    {
      duration = std::max(duration, 1.5 * delta / max_acceleration[joint]);
    }
    if (max_jerk[joint] > 0.0)
    {
      duration = std::max(duration, std::sqrt(6.0 * delta / max_jerk[joint]));
    }
  }
  return std::min(duration, max_dur);
}

// Empty a trajectory's SoA payload while retaining vector capacity, so a pre-reserved
// buffer (the stop-tail's) can be refilled in place with no RT heap allocation.
static void ClearTrajectoryPayload(GoalTrajectory & traj)
{
  traj.durations.clear();
  traj.positions.clear();
  traj.velocities.clear();
  traj.accelerations.clear();
  traj.jerks.clear();
}

// Build the endpoints for a quintic velocity blend from `position`: the start carries
// (pos, vel_from, accel = 0) -- the accel = 0 seed is the burst-runaway fix -- and the
// end is (pos + 0.5*(vel_from+vel_to)*T, vel_to, 0): advancing by the blend's mean
// velocity keeps the position quintic monotonic (no overshoot). The decel-to-rest
// stop-tail is the vel_to = rest special case (braking distance 0.5*v*T). Writes
// out_start / out_end (time_from_start 0 and T). Pure.
static void MakeBlendEndpoints(const std::vector<double> & position,
  const std::vector<double> & vel_from, const std::vector<double> & vel_to,
  double duration,
  trajectory_msgs::msg::JointTrajectoryPoint & out_start,
  trajectory_msgs::msg::JointTrajectoryPoint & out_end)
{
  const std::size_t joint_count = position.size();
  out_start.positions.assign(position.begin(), position.end());
  out_start.velocities.assign(vel_from.begin(), vel_from.end());
  out_start.accelerations.assign(joint_count, 0.0);
  out_start.time_from_start = rclcpp::Duration::from_seconds(0.0);
  out_end.positions.resize(joint_count);
  out_end.velocities.assign(vel_to.begin(), vel_to.end());
  out_end.accelerations.assign(joint_count, 0.0);
  for (std::size_t joint = 0; joint < joint_count; ++joint)
  {
    out_end.positions[joint] =
      position[joint] + 0.5 * (vel_from[joint] + vel_to[joint]) * duration;
  }
  out_end.time_from_start = rclcpp::Duration::from_seconds(duration);
}

// Read a snapshot's velocity intent -- its LAST point's per-joint velocities (a jog
// window means "keep moving like this"), clamped to the per-joint cap. A snapshot
// without velocities means rest (blend to a stop). Writes out_velocity (pre-sized to
// joint count). Pure.
static void ReadVelocityIntent(const GoalTrajectory & snapshot,
  const std::vector<double> & max_velocity, std::vector<double> & out_velocity)
{
  const std::size_t joint_count = out_velocity.size();
  const bool has_velocity = (snapshot.valid_fields & protocol::FieldMaskVelocity) != 0 &&
    snapshot.velocities.size() >= static_cast<std::size_t>(snapshot.num_points) * joint_count;
  const std::size_t last_base = static_cast<std::size_t>(snapshot.num_points - 1) * joint_count;
  for (std::size_t joint = 0; joint < joint_count; ++joint)
  {
    const double intent = has_velocity ? snapshot.velocities[last_base + joint] : 0.0;
    out_velocity[joint] = std::clamp(intent, -max_velocity[joint], max_velocity[joint]);
  }
}

// Extend a blend end by `duration` seconds of constant velocity: same velocities,
// accel 0, positions advanced by v*duration. With matching velocities and zero accel
// at both ends the fitted quintic degenerates to the straight line, so sampling this
// segment yields exact constant-velocity points. Returns the extension endpoint. Pure.
static trajectory_msgs::msg::JointTrajectoryPoint MakeExtensionEndpoint(
  const trajectory_msgs::msg::JointTrajectoryPoint & blend_end, double duration)
{
  trajectory_msgs::msg::JointTrajectoryPoint extension_end = blend_end;
  for (std::size_t joint = 0; joint < blend_end.positions.size(); ++joint)
  {
    extension_end.positions[joint] += blend_end.velocities[joint] * duration;
  }
  extension_end.time_from_start = rclcpp::Duration::from_seconds(
    rclcpp::Duration(blend_end.time_from_start).seconds() + duration);
  return extension_end;
}

bool PassthroughTrajectoryController::ActionBusy() const
{
  return feeding_.has_value() || !pending_.empty() || !inflight_.empty();
}

void PassthroughTrajectoryController::ResetOnlineState()
{
  online_traj_.reset();
  online_next_index_ = 0;
  online_active_ = false;
  online_stopping_ = false;
  online_refilling_ = true;  // a fresh stream starts at depth 0: refill immediately
  online_opened_ = false;
  online_since_snapshot_sec_ = 0.0;
  // Anything staged before this reset (fault, deactivate) is stale -- same ghost-reopen
  // guard as the stop-tail session end. On_activate flushes the buffer first, so a
  // fresh activation still starts with a clean slate.
  MarkStagedSnapshotSeen();
  online_stop_requested_.store(false);  // a stop request for the torn-down stream is moot
  std::fill(online_committed_pos_.begin(), online_committed_pos_.end(), 0.0);
  std::fill(online_committed_vel_.begin(), online_committed_vel_.end(), 0.0);
  online_busy_.store(false);  // keep the executor-facing mirror in step with online_active_
}

void PassthroughTrajectoryController::CaptureOnlineCommitted(int point_index)
{
  if (online_stopping_ || !online_traj_ || point_index < 0) { return; }
  const std::size_t joint_count = joint_names_.size();
  const std::size_t base = static_cast<std::size_t>(point_index) * joint_count;
  for (std::size_t joint = 0; joint < joint_count; ++joint)
  {
    online_committed_pos_[joint] = online_traj_->positions[base + joint];
    online_committed_vel_[joint] = online_traj_->velocities[base + joint];
  }
}

void PassthroughTrajectoryController::EmitOnlineBegin()
{
  // Open move: id 0 (a jog is not completion-tracked) + size -1. The hardware pushes no
  // completion threshold for a non-positive size, so completed_trajectory_id never
  // advances for a jog; the stop-tail's final chunk is what closes the move.
  SetCommandValue(ci_trajectory_id_, protocol::Encode<uint64_t>(0));
  SetCommandValue(ci_trajectory_size_, protocol::Encode(-1));
  SetCommandValue(ci_valid_fields_, protocol::Encode(online_traj_->valid_fields));
}

void PassthroughTrajectoryController::IngestOnlineSnapshot(
  const std::shared_ptr<GoalTrajectory> & snapshot, bool prev_acked,
  Cmd & command, bool & emit)
{
  if (!online_active_)
  {
    if (!prev_acked) { return; }  // mailbox still busy: retry this snapshot next cycle
    online_last_seen_ = snapshot.get();
    // First window of a jog: anchor the blend at the MEASURED state (no committed
    // stream exists yet) and open a fresh move. Window positions are never commanded
    // -- only its velocity intent -- so there is no continuity gate to fail.
    SeedOnlineAnchorFromState();
    if (!SpliceOnlineSnapshot(*snapshot))
    {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "online jog snapshot ignored: the adopt blend could not be built.");
      return;
    }
    online_active_ = true;
    online_stopping_ = false;
    online_opened_ = false;  // the next feed is this move's opening chunk
    EmitOnlineBegin();
    command = Cmd::Begin;
    emit = true;
    return;
  }
  // Active: velocity-intent splice (latest-wins). The window is re-anchored onto our
  // own committed (P,V), so it is never dropped for position error and never kinks the
  // stream the way the old whole-window replace did. A degenerate build (never in a
  // configured controller) leaves the current stream in place.
  online_last_seen_ = snapshot.get();
  if (!SpliceOnlineSnapshot(*snapshot))
  {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "online jog snapshot ignored: the splice blend could not be built.");
  }
}

void PassthroughTrajectoryController::SeedOnlineAnchorFromState()
{
  const std::size_t joint_count = joint_names_.size();
  for (std::size_t joint = 0; joint < joint_count; ++joint)
  {
    online_committed_pos_[joint] = ReadStateValue(si_joint_position_[joint]);
    // Defensive clamp: measured velocity seeds the blend start; a garbage reading must
    // not command an over-speed profile. At rest (the normal adopt case) this is ~0.
    online_committed_vel_[joint] = std::clamp(ReadStateValue(si_joint_velocity_[joint]),
      -online_max_velocity_[joint], online_max_velocity_[joint]);
  }
}

void PassthroughTrajectoryController::MarkStagedSnapshotSeen()
{
  const auto staged = rt_incoming_online_.readFromRT();
  online_last_seen_ = (staged != nullptr && *staged) ? staged->get() : nullptr;
}

bool PassthroughTrajectoryController::SpliceOnlineSnapshot(const GoalTrajectory & snapshot)
{
  const std::size_t joint_count = joint_names_.size();
  if (joint_count == 0 || !online_splice_traj_ || snapshot.num_points == 0) { return false; }

  // The window's absolute positions are deliberately IGNORED: they are anchored to the
  // producer's stale view of the robot (measured state, up to committed_horizon behind
  // our committed cursor), so blending toward them dips backward. Only the velocity
  // intent is trusted; position integrates forward from our own committed seam.
  ReadVelocityIntent(snapshot, online_max_velocity_, online_splice_vel_);
  const double blend_duration = BlendDuration(online_committed_vel_, online_splice_vel_,
    online_max_acceleration_, online_max_jerk_, kMinStopTailDuration, kMaxStopTailDuration);

  // Blend committed -> intent, then extend at the intent velocity long enough to
  // outlive the producer-silence timeout: without the extension a short window (Servo
  // publishes single-point windows) could drain the firmware buffer to starvation
  // before the stop-tail fires. Producer-rate path (not per-cycle): the endpoint
  // scratch and the sampler's coefficient buffers still allocate small vectors here --
  // a known, bounded prototype cost (the dense payload itself is pre-reserved).
  trajectory_msgs::msg::JointTrajectoryPoint blend_start;
  trajectory_msgs::msg::JointTrajectoryPoint blend_end;
  MakeBlendEndpoints(online_committed_pos_, online_committed_vel_, online_splice_vel_,
    blend_duration, blend_start, blend_end);
  const double extension_duration = online_producer_timeout_ + kSpliceExtensionMargin;
  const trajectory_msgs::msg::JointTrajectoryPoint extension_end =
    MakeExtensionEndpoint(blend_end, extension_duration);

  ClearTrajectoryPayload(*online_splice_traj_);
  online_splice_traj_->num_joints = static_cast<uint32_t>(joint_count);
  online_splice_traj_->valid_fields = protocol::FieldMaskPosition | protocol::FieldMaskVelocity |
    protocol::FieldMaskAcceleration | protocol::FieldMaskJerk;
  online_splice_traj_->finalize = false;  // the spliced stream keeps the move open
  SampleTrajectoryPoint(blend_start, blend_end, blend_duration, sample_period_,
    Interpolation::Quintic, online_identity_mapping_, *online_splice_traj_);
  SampleTrajectoryPoint(blend_end, extension_end, extension_duration, sample_period_,
    Interpolation::Quintic, online_identity_mapping_, *online_splice_traj_);
  AppendKnot(extension_end, sample_period_, online_identity_mapping_, *online_splice_traj_);
  online_splice_traj_->num_points = static_cast<uint32_t>(online_splice_traj_->durations.size());
  if (online_splice_traj_->num_points == 0) { return false; }

  // Swap the rebuilt buffer in as the live stream: cursor to the blend start, committed
  // seed re-captured from it, producer-liveness timer reset. The move stays open -- no
  // new BEGIN -- and the feed gate streams the tail from here.
  online_traj_ = online_splice_traj_;
  online_next_index_ = 0;
  online_since_snapshot_sec_ = 0.0;
  CaptureOnlineCommitted(0);
  return true;
}

void PassthroughTrajectoryController::FeedOnlineChunk(int max_points, Cmd & command, bool & emit)
{
  const int num_points = static_cast<int>(online_traj_->num_points);
  const int base = online_next_index_;
  // Chunk length = min(pipe, payload, policy): chunk_size_ is the mailbox width, the
  // remaining points are what's staged, and max_points is the gate's horizon space --
  // how many more points fit under the high-water mark this cycle. Without the last
  // cap a single full chunk (e.g. 64 points = 256ms at a 4ms grid) would blow through
  // an 80ms lookahead bound in one transaction.
  const int len = std::min(std::min(chunk_size_, num_points - base), max_points);
  if (len <= 0) { return; }  // no horizon space left: hold; the gate retries next cycle
  const bool is_last = (base + len == num_points);
  // finalize=false on the streamed window keeps the move open; only the finalizing
  // stop-tail carries finalize=true, so final_move is true only for its trailing chunk.
  const bool final_move = is_last && online_traj_->finalize;
  PresentChunk(*online_traj_, base, len, final_move);
  command = Cmd::AppendChunk;
  emit = true;
  online_next_index_ = base + len;
  online_opened_ = true;  // the move has its opening chunk; the gate applies from here
  CaptureOnlineCommitted(base + len - 1);
  if (final_move)
  {
    // The decel-to-rest tail's last chunk went out: the firmware move closes to IDLE.
    // End the session; a later snapshot reopens a fresh move via the hardware reopen
    // gate. Any snapshot still parked in the latest-wins buffer arrived before this
    // stop and is stale -- mark it seen so it cannot spawn a ghost reopen.
    online_active_ = false;
    online_stopping_ = false;
    MarkStagedSnapshotSeen();
  }
}

bool PassthroughTrajectoryController::BeginOnlineStopTail()
{
  const std::size_t joint_count = joint_names_.size();
  if (joint_count == 0 || !online_stop_traj_) { return false; }  // backstop: firmware e-stop

  // Clamp the committed seed speed to the configured cap (defensive) in place -- the
  // committed state is not reused once we commit to stopping.
  for (std::size_t joint = 0; joint < joint_count; ++joint)
  {
    online_committed_vel_[joint] = std::clamp(online_committed_vel_[joint],
      -online_max_velocity_[joint], online_max_velocity_[joint]);
  }
  const double duration = BlendDuration(online_committed_vel_, online_rest_vel_,
    online_max_acceleration_, online_max_jerk_, kMinStopTailDuration, kMaxStopTailDuration);

  // Rare one-shot on producer silence (NOT the per-cycle stream path): build the two
  // decel endpoints (a blend to rest) and sample them into the pre-reserved
  // online_stop_traj_ buffer.
  trajectory_msgs::msg::JointTrajectoryPoint start_point;
  trajectory_msgs::msg::JointTrajectoryPoint end_point;
  MakeBlendEndpoints(online_committed_pos_, online_committed_vel_, online_rest_vel_,
    duration, start_point, end_point);

  ClearTrajectoryPayload(*online_stop_traj_);
  online_stop_traj_->num_joints = static_cast<uint32_t>(joint_count);
  online_stop_traj_->valid_fields = protocol::FieldMaskPosition | protocol::FieldMaskVelocity |
    protocol::FieldMaskAcceleration | protocol::FieldMaskJerk;
  online_stop_traj_->finalize = true;  // the tail closes the open move
  SampleTrajectoryPoint(start_point, end_point, duration, sample_period_,
    Interpolation::Quintic, online_identity_mapping_, *online_stop_traj_);
  AppendKnot(end_point, sample_period_, online_identity_mapping_, *online_stop_traj_);
  online_stop_traj_->num_points = static_cast<uint32_t>(online_stop_traj_->durations.size());
  if (online_stop_traj_->num_points == 0) { return false; }

  online_traj_ = online_stop_traj_;
  online_next_index_ = 0;
  online_stopping_ = true;
  return true;
}

// Advance the feed gate's hysteresis phase (see the bounded-lookahead block in the
// header): below the low water -> start refilling; at/above the high water -> stop and
// hold; in between -> keep the current phase (that gap IS the hysteresis, so the gate
// refills in band-sized bursts instead of drip-feeding at a single edge). Pure.
static bool UpdateFillGate(bool refilling, int depth_points, int low_water_points,
  int high_water_points)
{
  if (depth_points < low_water_points) { return true; }
  if (depth_points >= high_water_points) { return false; }
  return refilling;
}

void PassthroughTrajectoryController::ServiceOnlineStream(
  bool prev_acked, double period_seconds, int committed_depth_points,
  Cmd & command, bool & emit)
{
  // 1. Latest-wins ingest. RealtimeBuffer keeps returning its last value, so identity-
  //    compare against online_last_seen_ to detect genuinely new data.
  //    An empty-trajectory stop request is consumed here every cycle, so
  //    a request made while no stream is open cannot stop a later one. It arrived after
  //    any window still staged, so that window is marked seen rather than adopted.
  const auto staged = rt_incoming_online_.readFromRT();
  const bool stop_requested = online_stop_requested_.exchange(false);
  if (stop_requested) { MarkStagedSnapshotSeen(); }
  const bool have_new =
    !stop_requested && staged && *staged && staged->get() != online_last_seen_;
  if (have_new && !ActionBusy() && !online_stopping_)
  {
    IngestOnlineSnapshot(*staged, prev_acked, command, emit);
    if (emit) { return; }  // BEGIN emitted this cycle; feed chunks starting next cycle
  }

  if (!online_active_) { return; }

  // 2. Producer silence, or the client's empty-trajectory stop -> synthesize the
  //    decel-to-rest stop-tail once, then let it feed.
  online_since_snapshot_sec_ += period_seconds;
  if (!online_stopping_ &&
      (stop_requested || online_since_snapshot_sec_ > online_producer_timeout_))
  {
    BeginOnlineStopTail();  // false -> unbuildable; firmware OUT_OF_FRAMES is the backstop
  }

  // 3. Feed gate (bounded lookahead): while refilling, feed the next chunk -- capped to
  //    the horizon space (high water minus committed depth) so one feed can't overshoot
  //    the band -- when acked and points remain. Otherwise HOLD at the staged tail (no
  //    command) until the depth drains, a new snapshot arrives, or the stop-tail fires.
  //    The gate also paces the stop-tail: it appends after the committed frames, so
  //    trickling it in keeps the total committed depth bounded through the stop.
  //    EXCEPTION -- the move's OPENING chunk bypasses the gate entirely: the firmware
  //    arms its out-of-frames watchdog (kEmptyCount = 64 frames = 32 points) the
  //    instant a move opens, and right after a reopen the reported depth still counts
  //    the PREVIOUS move's draining frames, which would cap the opening chunk to a few
  //    points and starve the new move at birth (the live teleop stop/start fault). One
  //    full chunk transiently overshoots the band; the gate reasserts from chunk two.
  online_refilling_ = UpdateFillGate(online_refilling_, committed_depth_points,
    online_low_water_points_, online_high_water_points_);
  const bool opening_chunk = !online_opened_;
  const int max_points = opening_chunk
    ? chunk_size_ : (online_high_water_points_ - committed_depth_points);
  if ((online_refilling_ || opening_chunk) && prev_acked &&
    online_next_index_ < static_cast<int>(online_traj_->num_points))
  {
    FeedOnlineChunk(max_points, command, emit);
  }
}

void PassthroughTrajectoryController::JointTrajectoryTopicCallback(
  const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr message)
{
  if (const char * latched = IntakeLatchReason())
  {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "jog message dropped: %s.", latched);
    return;
  }
  if (action_busy_.load())  // mutual exclusion: an action goal owns the mailbox
  {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "jog message dropped: a FollowJointTrajectory action goal is active.");
    return;
  }
  // JTC semantics: an empty trajectory is a soft-stop request, not a
  // malformed message. Joint names are not consulted; there is nothing to map.
  if (message->points.empty())
  {
    online_stop_requested_.store(true);
    return;
  }
  std::string reason;
  if (!ValidateTrajectoryStructure(*message, joint_names_, RequiredFields(interpolation_), reason))
  {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "jog message rejected: %s.", reason.c_str());
    return;
  }
  auto snapshot = RemapTrajectory(*message);
  if (!snapshot) { return; }  // ValidateTrajectoryStructure already guards the mapping
  snapshot->finalize = false;  // an online stream never finalizes; only the stop-tail does
  rt_incoming_online_.writeFromNonRT(std::move(snapshot));
}

std::shared_ptr<GoalTrajectory> PassthroughTrajectoryController::RemapTrajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory) const
{
  const std::size_t num_joints = joint_names_.size();
  if (trajectory.points.empty()) { return nullptr; }

  // Column of each managed joint within the trajectory's own joint order (shared helper).
  const std::vector<int> joint_mapping = ResolveJointMapping(trajectory);
  if (joint_mapping.empty()) { return nullptr; }

  auto out = std::make_shared<GoalTrajectory>(num_joints, trajectory.points.size());

  // The resampler always synthesizes a full P/V/A/J sample per point -- A and J are the
  // fitted polynomial's derivatives whatever the interpolation order -- so every
  // trajectory advertises all four fields. Plain int masks -- enum class ValidField has
  // no operator|.
  out->valid_fields = protocol::FieldMaskPosition | protocol::FieldMaskVelocity |
    protocol::FieldMaskAcceleration | protocol::FieldMaskJerk;

  for (std::size_t index = 0; index + 1 < trajectory.points.size(); ++index)
  {
    const auto& start = trajectory.points[index];
    const auto& end = trajectory.points[index + 1];
    if (start.positions.size() != trajectory.joint_names.size()) { return nullptr; }
    const double duration = rclcpp::Duration(end.time_from_start).seconds() -
      rclcpp::Duration(start.time_from_start).seconds();
    // Pick the order per segment (Auto -> from field presence, like JTC; else forced).
    const Interpolation method = ResolveInterpolation(interpolation_, start, end, num_joints);
    SampleTrajectoryPoint(start, end, duration, sample_period_, method, joint_mapping, *out);
  }

  // Each segment's half-open sampling emits its start but not its end; append the
  // final knot so the dense grid reaches the trajectory's last point exactly once.
  AppendKnot(trajectory.points.back(), sample_period_, joint_mapping, *out);
  out->num_points = static_cast<uint32_t>(out->durations.size());
  return out;
}

std::shared_ptr<GoalTrajectory> PassthroughTrajectoryController::RemapTrajectory(
  const FollowJointTrajectory::Goal & goal) const
{
  return RemapTrajectory(goal.trajectory);
}

}  // namespace rapidcode_passthrough_trajectory_controller

PLUGINLIB_EXPORT_CLASS(
  rapidcode_passthrough_trajectory_controller::PassthroughTrajectoryController,
  controller_interface::ControllerInterface)

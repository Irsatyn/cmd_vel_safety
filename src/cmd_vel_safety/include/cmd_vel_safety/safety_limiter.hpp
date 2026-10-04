// Copyright 2026 cmd_vel_safety contributors. Apache-2.0.
//
// Stateful velocity-command safety pipeline, deliberately free of any ROS
// dependency so that it can be unit tested with plain gtest: feed it a
// sequence of (command, timestamp) pairs plus a fixed dt and assert on the
// outputs, without a ROS graph or scheduler jitter in the way.

#ifndef CMD_VEL_SAFETY__SAFETY_LIMITER_HPP_
#define CMD_VEL_SAFETY__SAFETY_LIMITER_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace cmd_vel_safety
{

/// Intervention flags. Values MUST stay in sync with the FLAG_* constants in
/// cmd_vel_safety_msgs/msg/SafetyReport.msg.
enum SafetyFlag : uint16_t
{
  FLAG_NONE = 0,
  FLAG_NON_FINITE = 1,
  FLAG_NONHOLONOMIC = 2,
  FLAG_SPIKE_REJECTED = 4,
  FLAG_LINEAR_CLAMPED = 8,
  FLAG_ANGULAR_CLAMPED = 16,
  FLAG_LATERAL_ACCEL_LIMITED = 32,
  FLAG_ACCEL_LIMITED = 64,
  FLAG_DEADBAND_APPLIED = 128,
  FLAG_WATCHDOG_TIMEOUT = 256,
  FLAG_ESTOP = 512,
  FLAG_SAFETY_HOLD = 1024,
};

/// A raw 6-DOF twist exactly as received from the wire.
struct Twist6
{
  double lx = 0.0;
  double ly = 0.0;
  double lz = 0.0;
  double ax = 0.0;
  double ay = 0.0;
  double az = 0.0;
};

/// The two degrees of freedom a differential-drive base can actually execute.
struct Velocity2D
{
  double v = 0.0;  ///< m/s   (maps to linear.x)
  double w = 0.0;  ///< rad/s (maps to angular.z)
};

/// Complete tunable envelope. Mirrored one-to-one by ROS parameters.
struct Limits
{
  // --- kinematic envelope ---
  double max_linear_x = 1.0;        ///< m/s, forward
  double min_linear_x = -0.3;       ///< m/s, reverse (tighter: no rear sensing)
  double max_angular_z = 1.5;       ///< rad/s
  double max_linear_accel = 0.8;    ///< m/s^2
  double max_linear_decel = 1.5;    ///< m/s^2, braking may be harder
  double max_angular_accel = 2.0;   ///< rad/s^2
  double max_angular_decel = 3.0;   ///< rad/s^2
  double max_lateral_accel = 1.2;   ///< m/s^2, bounds |v*w| (tip-over / cargo)

  // --- input validation ---
  bool enforce_nonholonomic = true;
  double nonholonomic_epsilon = 1e-6;
  int max_consecutive_invalid = 5;  ///< invalid frames tolerated before hold

  // --- spike rejection ---
  bool spike_rejection_enabled = true;
  double spike_linear_threshold = 0.6;    ///< m/s jump needing confirmation
  double spike_angular_threshold = 1.2;   ///< rad/s jump needing confirmation
  int spike_confirm_count = 1;            ///< extra frames needed to confirm

  // --- deadband ---
  double linear_deadband = 0.01;
  double angular_deadband = 0.02;

  // --- failsafe ---
  double cmd_timeout = 0.5;            ///< s without input -> controlled brake
  double emergency_decel_factor = 2.0; ///< decel multiplier while failing safe
  bool estop_hard_stop = true;         ///< e-stop bypasses the slew limiter
};

/// Outcome of validating one incoming command.
struct ValidationResult
{
  bool accepted = false;          ///< the target was updated from this frame
  uint16_t flags = FLAG_NONE;
  std::vector<std::string> reasons;
};

/// Outcome of one fixed-rate control cycle.
struct CycleResult
{
  Velocity2D output;              ///< command to publish
  Velocity2D target;              ///< effective target after clamping
  uint16_t flags = FLAG_NONE;
  std::vector<std::string> reasons;
  bool watchdog_active = false;
  bool estop_active = false;
  bool safety_hold = false;
};

/// Two-phase safety pipeline.
///
/// Call submit() for every received command (validation, spike rejection,
/// deadband) and update() at a fixed rate (failsafe arbitration, clamping,
/// slew limiting). Splitting the two is what lets the node keep publishing a
/// controlled brake command while the upstream publisher is silent, and lets
/// runtime parameter changes take effect on the very next output cycle.
class SafetyLimiter
{
public:
  SafetyLimiter() = default;

  void setLimits(const Limits & limits) {limits_ = limits;}
  const Limits & limits() const {return limits_;}

  /// Validate one raw command. `stamp` is seconds on a monotonic clock.
  ValidationResult submit(const Twist6 & raw, double stamp);

  /// Produce the command to publish. `dt` is the nominal cycle period.
  CycleResult update(double now, double dt);

  /// Engage / release the latched emergency stop. Releasing also resets the
  /// target to zero so that clearing an e-stop never resumes the old speed.
  void setEstop(bool engaged);
  bool estop() const {return estop_;}

  /// Drop all history. Used when the clock jumps backwards (bag loop, sim
  /// reset), where accumulated rates and dt would otherwise be nonsense.
  void reset();

  double inputRateHz() const {return input_rate_hz_;}
  double timeSinceLastInput(double now) const;
  bool hasInput() const {return has_input_;}

  const Twist6 & lastRawInput() const {return last_raw_;}
  const Velocity2D & output() const {return output_;}
  const Velocity2D & target() const {return target_;}

  uint64_t totalReceived() const {return total_received_;}
  uint64_t totalRejected() const {return total_rejected_;}
  uint64_t totalModified() const {return total_modified_;}
  uint64_t totalPublished() const {return total_published_;}

private:
  Limits limits_;

  // accepted target (validated, not yet clamped)
  Velocity2D target_;
  Velocity2D output_;

  // spike-rejection candidate/confirm state machine
  Velocity2D pending_;
  bool has_pending_ = false;
  int pending_count_ = 0;

  // input bookkeeping
  Twist6 last_raw_;
  bool has_input_ = false;
  double last_input_time_ = 0.0;
  double interval_ema_ = 0.0;
  double input_rate_hz_ = 0.0;

  // failsafe state
  bool estop_ = false;
  bool watchdog_ = false;
  bool watchdog_latched_ = false;
  bool safety_hold_ = false;
  int consecutive_invalid_ = 0;
  int consecutive_valid_ = 0;

  bool started_ = false;
  double start_time_ = 0.0;
  double last_dt_ = 0.05;  ///< last plausible cycle period, used as a fallback

  uint64_t total_received_ = 0;
  uint64_t total_rejected_ = 0;
  uint64_t total_modified_ = 0;
  uint64_t total_published_ = 0;
};

/// Human-readable names for a flag bitmask, for logs and status strings.
std::vector<std::string> describeFlags(uint16_t flags);

}  // namespace cmd_vel_safety

#endif  // CMD_VEL_SAFETY__SAFETY_LIMITER_HPP_

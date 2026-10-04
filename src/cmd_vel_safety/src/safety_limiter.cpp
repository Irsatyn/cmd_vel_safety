// Copyright 2026 cmd_vel_safety contributors. Apache-2.0.

#include "cmd_vel_safety/safety_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace cmd_vel_safety
{
namespace
{

bool allFinite(const Twist6 & t)
{
  return std::isfinite(t.lx) && std::isfinite(t.ly) && std::isfinite(t.lz) &&
         std::isfinite(t.ax) && std::isfinite(t.ay) && std::isfinite(t.az);
}

std::string fmt(double v)
{
  std::ostringstream os;
  os.precision(3);
  os << std::fixed << v;
  return os.str();
}

/// Move `cur` towards `tgt` by at most one cycle's worth of acceleration.
///
/// The acceleration budget depends on whether the magnitude is growing
/// (accelerating) or shrinking / reversing (braking). Braking gets the larger
/// budget because a robot must always be able to stop faster than it started.
double slew(
  double cur, double tgt, double a_accel, double a_decel, double dt, bool & limited)
{
  const double diff = tgt - cur;
  if (diff == 0.0) {
    return tgt;
  }
  const bool accelerating = (std::fabs(tgt) > std::fabs(cur)) && (tgt * cur >= 0.0);
  const double a = accelerating ? a_accel : a_decel;
  if (!(a > 0.0)) {
    return tgt;  // limiting disabled; parameter validation normally prevents this
  }
  const double max_step = a * dt;
  if (std::fabs(diff) <= max_step) {
    return tgt;
  }
  limited = true;
  return cur + std::copysign(max_step, diff);
}

}  // namespace

ValidationResult SafetyLimiter::submit(const Twist6 & raw, double stamp)
{
  ValidationResult r;
  ++total_received_;
  last_raw_ = raw;

  // Inter-arrival rate estimate (EMA, bounded memory). Implausible intervals
  // are ignored so a single stall does not poison the estimate.
  if (has_input_) {
    const double gap = stamp - last_input_time_;
    if (gap > 0.0 && gap < 10.0) {
      interval_ema_ = (interval_ema_ > 0.0) ? (0.2 * gap + 0.8 * interval_ema_) : gap;
      input_rate_hz_ = (interval_ema_ > 1e-9) ? (1.0 / interval_ema_) : 0.0;
    }
  }
  last_input_time_ = stamp;
  has_input_ = true;

  // ---- S1: finiteness -----------------------------------------------------
  // A NaN means the upstream computation has already broken down (division by
  // zero, uninitialised memory). The remaining components of the same frame
  // are no more trustworthy, so the whole frame is dropped rather than
  // patched component-wise.
  if (!allFinite(raw)) {
    ++consecutive_invalid_;
    consecutive_valid_ = 0;
    ++total_rejected_;
    r.flags |= FLAG_NON_FINITE;
    r.reasons.push_back(
      "non-finite input (NaN/Inf): frame dropped, holding last valid target (" +
      std::to_string(consecutive_invalid_) + " consecutive)");
    if (consecutive_invalid_ >= limits_.max_consecutive_invalid && !safety_hold_) {
      safety_hold_ = true;
      r.flags |= FLAG_SAFETY_HOLD;
      r.reasons.push_back(
        "upstream persistently invalid (>= " +
        std::to_string(limits_.max_consecutive_invalid) +
        " frames): entering safety hold, braking to zero");
    }
    return r;
  }

  consecutive_valid_ = std::min(consecutive_valid_ + 1, 1 << 20);  // capped, no overflow
  consecutive_invalid_ = 0;
  // Symmetric hysteresis: leaving the hold needs as many good frames as it
  // took bad ones to enter it, so an alternating good/bad upstream cannot
  // flap the robot in and out of motion.
  if (safety_hold_ && consecutive_valid_ >= limits_.max_consecutive_invalid) {
    safety_hold_ = false;
    r.reasons.push_back("upstream recovered: safety hold released");
  }

  Velocity2D cand{raw.lx, raw.az};

  // ---- S2: non-holonomic projection --------------------------------------
  // A differential-drive base can only execute linear.x and angular.z.
  // Silently ignoring linear.y would make the robot's motion diverge from the
  // upstream node's intent without anyone noticing, so it is reported.
  if (limits_.enforce_nonholonomic) {
    const double eps = limits_.nonholonomic_epsilon;
    if (std::fabs(raw.ly) > eps || std::fabs(raw.lz) > eps ||
      std::fabs(raw.ax) > eps || std::fabs(raw.ay) > eps)
    {
      r.flags |= FLAG_NONHOLONOMIC;
      r.reasons.push_back(
        "unexecutable DOF requested (ly=" + fmt(raw.ly) + " lz=" + fmt(raw.lz) +
        " ax=" + fmt(raw.ax) + " ay=" + fmt(raw.ay) + "): projected to zero");
    }
  }

  // ---- S3: spike rejection (candidate / confirm) -------------------------
  // A sustained out-of-range command must be clamped and passed through, while
  // a single-frame outlier must be dropped. Their instantaneous jump sizes are
  // indistinguishable, so the discriminator has to be temporal: a suspicious
  // jump is held as a candidate and only adopted once a later frame confirms
  // it. This costs one input period of latency on genuine step commands --
  // invisible in practice because the slew limiter smooths steps anyway -- and
  // unlike a median filter it delays nothing else.
  if (limits_.spike_rejection_enabled) {
    const bool jump =
      std::fabs(cand.v - target_.v) > limits_.spike_linear_threshold ||
      std::fabs(cand.w - target_.w) > limits_.spike_angular_threshold;

    if (!jump) {
      has_pending_ = false;
      pending_count_ = 0;
    } else {
      const bool confirms = has_pending_ &&
        std::fabs(cand.v - pending_.v) <= limits_.spike_linear_threshold &&
        std::fabs(cand.w - pending_.w) <= limits_.spike_angular_threshold;

      if (confirms && ++pending_count_ >= limits_.spike_confirm_count) {
        has_pending_ = false;
        pending_count_ = 0;  // confirmed: fall through and adopt
      } else {
        if (!confirms) {
          pending_ = cand;
          has_pending_ = true;
          pending_count_ = 0;
        }
        ++total_rejected_;
        r.flags |= FLAG_SPIKE_REJECTED;
        r.reasons.push_back(
          "jump to (v=" + fmt(cand.v) + ", w=" + fmt(cand.w) + ") from (v=" +
          fmt(target_.v) + ", w=" + fmt(target_.w) +
          ") exceeds spike threshold: held pending confirmation");
        return r;
      }
    }
  }

  // ---- S4: deadband -------------------------------------------------------
  // Commands below motor stiction only make the drivetrain buzz.
  if (cand.v != 0.0 && std::fabs(cand.v) < limits_.linear_deadband) {
    cand.v = 0.0;
    r.flags |= FLAG_DEADBAND_APPLIED;
    r.reasons.push_back("linear command inside deadband: snapped to zero");
  }
  if (cand.w != 0.0 && std::fabs(cand.w) < limits_.angular_deadband) {
    cand.w = 0.0;
    r.flags |= FLAG_DEADBAND_APPLIED;
    r.reasons.push_back("angular command inside deadband: snapped to zero");
  }

  target_ = cand;
  r.accepted = true;
  if (r.flags != FLAG_NONE) {
    ++total_modified_;
  }
  return r;
}

CycleResult SafetyLimiter::update(double now, double dt)
{
  CycleResult c;
  ++total_published_;

  if (!started_) {
    started_ = true;
    start_time_ = now;
  }
  // A nonsensical dt (clock jump, first cycle, heavily delayed timer) must not
  // turn into an unbounded acceleration budget. Note that std::clamp would
  // propagate a NaN straight through, so the finite check has to come first:
  // fall back to the last plausible period instead of degrading the output.
  if (!std::isfinite(dt) || dt <= 0.0) {
    dt = last_dt_;
  } else {
    dt = std::min(dt, 1.0);
    last_dt_ = dt;
  }

  // ---- U0: failsafe arbitration ------------------------------------------
  const double since = has_input_ ? (now - last_input_time_) : (now - start_time_);
  watchdog_ = since > limits_.cmd_timeout;

  if (watchdog_ && !watchdog_latched_) {
    // Reset the target, not just the output. On recovery the next command is
    // then evaluated relative to standstill, so the robot ramps up from zero
    // instead of jumping back to whatever it was doing before the link died.
    target_ = Velocity2D{};
    has_pending_ = false;
    pending_count_ = 0;
    watchdog_latched_ = true;
  } else if (!watchdog_) {
    watchdog_latched_ = false;
  }

  Velocity2D tgt = target_;
  bool emergency = false;

  if (estop_) {
    tgt = Velocity2D{};
    emergency = true;
    c.flags |= FLAG_ESTOP;
    c.reasons.push_back("emergency stop engaged");
  } else if (safety_hold_) {
    tgt = Velocity2D{};
    emergency = true;
    c.flags |= FLAG_SAFETY_HOLD;
    c.reasons.push_back("safety hold: upstream persistently invalid");
  } else if (watchdog_) {
    tgt = Velocity2D{};
    emergency = true;
    c.flags |= FLAG_WATCHDOG_TIMEOUT;
    c.reasons.push_back(
      "no command for " + fmt(since) + " s (timeout " + fmt(limits_.cmd_timeout) +
      " s): braking to zero");
  } else {
    // ---- U1: magnitude clamp ---------------------------------------------
    // Done here rather than at input time so that lowering a limit with
    // `ros2 param set` takes effect on the command currently being executed.
    const double v0 = tgt.v;
    const double w0 = tgt.w;
    tgt.v = std::clamp(tgt.v, limits_.min_linear_x, limits_.max_linear_x);
    if (tgt.v != v0) {
      c.flags |= FLAG_LINEAR_CLAMPED;
      c.reasons.push_back(
        "linear " + fmt(v0) + " outside [" + fmt(limits_.min_linear_x) + ", " +
        fmt(limits_.max_linear_x) + "]: clamped to " + fmt(tgt.v));
    }
    tgt.w = std::clamp(tgt.w, -limits_.max_angular_z, limits_.max_angular_z);
    if (tgt.w != w0) {
      c.flags |= FLAG_ANGULAR_CLAMPED;
      c.reasons.push_back(
        "angular " + fmt(w0) + " exceeds +-" + fmt(limits_.max_angular_z) +
        ": clamped to " + fmt(tgt.w));
    }

    // ---- U2: centripetal acceleration coupling ---------------------------
    // v and w can each be legal while their product is not: v*w is the lateral
    // acceleration that tips a tall robot over or slides its payload off.
    if (limits_.max_lateral_accel > 0.0 && std::fabs(tgt.v) > 1e-6) {
      const double lat = std::fabs(tgt.v * tgt.w);
      if (lat > limits_.max_lateral_accel) {
        const double w_max = limits_.max_lateral_accel / std::fabs(tgt.v);
        c.reasons.push_back(
          "lateral accel |v*w|=" + fmt(lat) + " exceeds " +
          fmt(limits_.max_lateral_accel) + ": angular reduced from " + fmt(tgt.w) +
          " to " + fmt(std::copysign(w_max, tgt.w)));
        tgt.w = std::copysign(w_max, tgt.w);
        c.flags |= FLAG_LATERAL_ACCEL_LIMITED;
      }
    }
  }
  c.target = tgt;

  // ---- U3 / U4: slew limiting --------------------------------------------
  if (estop_ && limits_.estop_hard_stop) {
    // The only path allowed to ignore the acceleration envelope.
    output_ = Velocity2D{};
  } else {
    const double f = emergency ? std::max(1.0, limits_.emergency_decel_factor) : 1.0;
    bool limited = false;
    output_.v = slew(
      output_.v, tgt.v, limits_.max_linear_accel, limits_.max_linear_decel * f, dt, limited);
    output_.w = slew(
      output_.w, tgt.w, limits_.max_angular_accel, limits_.max_angular_decel * f, dt, limited);
    if (limited) {
      c.flags |= FLAG_ACCEL_LIMITED;
      c.reasons.push_back("acceleration limited towards target");
    }
  }

  // Final unconditional net: whatever happened above, nothing non-finite is
  // ever allowed to reach the drivetrain.
  if (!std::isfinite(output_.v) || !std::isfinite(output_.w)) {
    output_ = Velocity2D{};
    c.flags |= FLAG_NON_FINITE;
    c.reasons.push_back("internal non-finite state detected: output forced to zero");
  }

  c.output = output_;
  c.watchdog_active = watchdog_;
  c.estop_active = estop_;
  c.safety_hold = safety_hold_;
  return c;
}

void SafetyLimiter::setEstop(bool engaged)
{
  if (engaged == estop_) {
    return;
  }
  estop_ = engaged;
  // Both engaging and releasing clear the target: releasing an e-stop must
  // never hand the drivetrain back the speed it had when it was engaged.
  target_ = Velocity2D{};
  has_pending_ = false;
  pending_count_ = 0;
}

void SafetyLimiter::reset()
{
  target_ = Velocity2D{};
  output_ = Velocity2D{};
  pending_ = Velocity2D{};
  has_pending_ = false;
  pending_count_ = 0;
  last_raw_ = Twist6{};
  has_input_ = false;
  last_input_time_ = 0.0;
  interval_ema_ = 0.0;
  input_rate_hz_ = 0.0;
  watchdog_ = false;
  watchdog_latched_ = false;
  safety_hold_ = false;
  consecutive_invalid_ = 0;
  consecutive_valid_ = 0;
  started_ = false;
  start_time_ = 0.0;
  last_dt_ = 0.05;
  // Counters are intentionally preserved: they describe the process lifetime.
}

double SafetyLimiter::timeSinceLastInput(double now) const
{
  if (!has_input_) {
    return std::numeric_limits<double>::infinity();
  }
  return now - last_input_time_;
}

std::vector<std::string> describeFlags(uint16_t flags)
{
  static const struct
  {
    uint16_t bit;
    const char * name;
  } kNames[] = {
    {FLAG_NON_FINITE, "NON_FINITE"},
    {FLAG_NONHOLONOMIC, "NONHOLONOMIC"},
    {FLAG_SPIKE_REJECTED, "SPIKE_REJECTED"},
    {FLAG_LINEAR_CLAMPED, "LINEAR_CLAMPED"},
    {FLAG_ANGULAR_CLAMPED, "ANGULAR_CLAMPED"},
    {FLAG_LATERAL_ACCEL_LIMITED, "LATERAL_ACCEL_LIMITED"},
    {FLAG_ACCEL_LIMITED, "ACCEL_LIMITED"},
    {FLAG_DEADBAND_APPLIED, "DEADBAND"},
    {FLAG_WATCHDOG_TIMEOUT, "WATCHDOG_TIMEOUT"},
    {FLAG_ESTOP, "ESTOP"},
    {FLAG_SAFETY_HOLD, "SAFETY_HOLD"},
  };
  std::vector<std::string> out;
  for (const auto & e : kNames) {
    if (flags & e.bit) {
      out.emplace_back(e.name);
    }
  }
  return out;
}

}  // namespace cmd_vel_safety

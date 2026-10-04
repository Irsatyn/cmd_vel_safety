// Copyright 2026 cmd_vel_safety contributors. Apache-2.0.
//
// velocity_guard: the safety gateway between an untrusted /cmd_vel producer
// and the drivetrain.
//
// Output is driven by a fixed-rate timer rather than by incoming messages.
// That is deliberate:
//   * slew limiting needs a deterministic dt -- input jitter would otherwise
//     corrupt the acceleration computation;
//   * the watchdog has to emit a braking command precisely when there is NO
//     input, which an event-driven design cannot do;
//   * the drivetrain gets a steady command stream, so upstream jitter is not
//     propagated to the actuators.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "cmd_vel_safety/safety_limiter.hpp"
#include "cmd_vel_safety_msgs/msg/safety_report.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace cmd_vel_safety
{

using SafetyReport = cmd_vel_safety_msgs::msg::SafetyReport;

// The library's flag enum and the message constants are two independent
// declarations of the same protocol; keep them provably identical.
static_assert(FLAG_NON_FINITE == SafetyReport::FLAG_NON_FINITE, "flag drift");
static_assert(FLAG_NONHOLONOMIC == SafetyReport::FLAG_NONHOLONOMIC, "flag drift");
static_assert(FLAG_SPIKE_REJECTED == SafetyReport::FLAG_SPIKE_REJECTED, "flag drift");
static_assert(FLAG_LINEAR_CLAMPED == SafetyReport::FLAG_LINEAR_CLAMPED, "flag drift");
static_assert(FLAG_ANGULAR_CLAMPED == SafetyReport::FLAG_ANGULAR_CLAMPED, "flag drift");
static_assert(
  FLAG_LATERAL_ACCEL_LIMITED == SafetyReport::FLAG_LATERAL_ACCEL_LIMITED, "flag drift");
static_assert(FLAG_ACCEL_LIMITED == SafetyReport::FLAG_ACCEL_LIMITED, "flag drift");
static_assert(FLAG_DEADBAND_APPLIED == SafetyReport::FLAG_DEADBAND_APPLIED, "flag drift");
static_assert(FLAG_WATCHDOG_TIMEOUT == SafetyReport::FLAG_WATCHDOG_TIMEOUT, "flag drift");
static_assert(FLAG_ESTOP == SafetyReport::FLAG_ESTOP, "flag drift");
static_assert(FLAG_SAFETY_HOLD == SafetyReport::FLAG_SAFETY_HOLD, "flag drift");

class VelocityGuard : public rclcpp::Node
{
public:
  VelocityGuard()
  : Node("velocity_guard")
  {
    declareParameters();
    limits_ = limitsFromParameters();

    const std::string input_topic = get_parameter("input_topic").as_string();
    const std::string output_topic = get_parameter("output_topic").as_string();
    const std::string report_topic = get_parameter("report_topic").as_string();
    const std::string estop_topic = get_parameter("estop_topic").as_string();
    control_rate_hz_ = get_parameter("control_rate_hz").as_double();

    limiter_.setLimits(limits_);

    // A BestEffort subscription is compatible with both Reliable and
    // BestEffort publishers, whereas a Reliable subscription silently receives
    // nothing from a BestEffort publisher. BestEffort is therefore the safer
    // default for an input we do not control.
    rclcpp::QoS cmd_qos(rclcpp::KeepLast(10));
    if (get_parameter("cmd_vel_qos_reliability").as_string() == "reliable") {
      cmd_qos.reliable();
    } else {
      cmd_qos.best_effort();
    }

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic, cmd_qos,
      std::bind(&VelocityGuard::onCmdVel, this, std::placeholders::_1));

    // TransientLocal by default: if the e-stop was asserted before this node
    // started, we still receive it immediately. Restarting a node must not
    // silently clear an active emergency stop.
    //
    // The cost is a QoS compatibility trap: a Volatile publisher -- which is
    // what a bare `ros2 topic pub` and a plain `ros2 bag play` both are -- is
    // INCOMPATIBLE with a TransientLocal subscription, and DDS resolves that
    // by delivering nothing at all, silently. Safety wins the default, so this
    // is configurable rather than relaxed, and the required publisher-side
    // invocation is spelled out in the README.
    rclcpp::QoS estop_qos(rclcpp::KeepLast(1));
    estop_qos.reliable();
    const std::string estop_durability =
      get_parameter("estop_qos_durability").as_string();
    if (estop_durability == "volatile") {
      estop_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
      RCLCPP_WARN(
        get_logger(),
        "/e_stop subscribed with VOLATILE durability: an e-stop asserted before "
        "this node started will NOT be seen");
    } else {
      estop_qos.transient_local();
    }
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      estop_topic, estop_qos,
      std::bind(&VelocityGuard::onEstop, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      output_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    report_pub_ = create_publisher<SafetyReport>(
      report_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    param_cb_ = add_on_set_parameters_callback(
      std::bind(&VelocityGuard::onSetParameters, this, std::placeholders::_1));

    startTimer();

    RCLCPP_INFO(
      get_logger(),
      "velocity_guard up: %s -> %s @ %.1f Hz | v in [%.2f, %.2f] m/s, |w| <= %.2f rad/s, "
      "accel <= %.2f m/s^2, timeout %.2f s",
      input_topic.c_str(), output_topic.c_str(), control_rate_hz_,
      limits_.min_linear_x, limits_.max_linear_x, limits_.max_angular_z,
      limits_.max_linear_accel, limits_.cmd_timeout);
  }

private:
  // ---------------------------------------------------------------- params --
  void declareDouble(
    const std::string & name, double def, double lo, double hi, const std::string & desc)
  {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = desc;
    rcl_interfaces::msg::FloatingPointRange r;
    r.from_value = lo;
    r.to_value = hi;
    d.floating_point_range.push_back(r);
    declare_parameter(name, def, d);
  }

  void declareInt(
    const std::string & name, int def, int lo, int hi, const std::string & desc)
  {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = desc;
    rcl_interfaces::msg::IntegerRange r;
    r.from_value = lo;
    r.to_value = hi;
    d.integer_range.push_back(r);
    declare_parameter(name, def, d);
  }

  void declareReadOnlyString(
    const std::string & name, const std::string & def, const std::string & desc)
  {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = desc + " (read-only: requires a restart)";
    d.read_only = true;
    declare_parameter(name, def, d);
  }

  void declareParameters()
  {
    declareReadOnlyString("input_topic", "/cmd_vel", "raw command input topic");
    declareReadOnlyString("output_topic", "/cmd_vel_safe", "processed command output topic");
    declareReadOnlyString("report_topic", "~/report", "per-cycle intervention report topic");
    declareReadOnlyString("estop_topic", "/e_stop", "latched emergency-stop topic");
    declareReadOnlyString(
      "cmd_vel_qos_reliability", "best_effort", "input QoS: best_effort | reliable");
    declareReadOnlyString(
      "estop_qos_durability", "transient_local",
      "e-stop QoS durability: transient_local (a late-starting guard learns an "
      "already-asserted e-stop, but Volatile publishers cannot reach it) | "
      "volatile (accepts any publisher, loses the latch)");

    declareDouble("control_rate_hz", 20.0, 1.0, 1000.0, "fixed output rate [Hz]");

    declareDouble("max_linear_x", 1.0, 0.0, 10.0, "forward speed limit [m/s]");
    declareDouble("min_linear_x", -0.3, -10.0, 0.0, "reverse speed limit [m/s], <= 0");
    declareDouble("max_angular_z", 1.5, 0.0, 20.0, "yaw rate limit [rad/s]");
    declareDouble("max_linear_accel", 0.8, 0.01, 50.0, "linear acceleration limit [m/s^2]");
    declareDouble("max_linear_decel", 1.5, 0.01, 50.0, "linear deceleration limit [m/s^2]");
    declareDouble("max_angular_accel", 2.0, 0.01, 100.0, "angular acceleration limit [rad/s^2]");
    declareDouble("max_angular_decel", 3.0, 0.01, 100.0, "angular deceleration limit [rad/s^2]");
    declareDouble(
      "max_lateral_accel", 1.2, 0.0, 50.0,
      "centripetal acceleration limit on |v*w| [m/s^2]; 0 disables");

    declare_parameter(
      "enforce_nonholonomic", true);
    declareDouble(
      "nonholonomic_epsilon", 1e-6, 0.0, 1.0,
      "tolerance below which an unexecutable DOF is not reported");
    declareInt(
      "max_consecutive_invalid", 5, 1, 1000,
      "invalid frames tolerated before entering safety hold");

    declare_parameter("spike_rejection_enabled", true);
    declareDouble(
      "spike_linear_threshold", 0.6, 0.0, 50.0,
      "linear jump [m/s] that must be confirmed by a later frame");
    declareDouble(
      "spike_angular_threshold", 1.2, 0.0, 100.0,
      "angular jump [rad/s] that must be confirmed by a later frame");
    declareInt("spike_confirm_count", 1, 1, 20, "extra frames required to confirm a jump");

    declareDouble("linear_deadband", 0.01, 0.0, 1.0, "linear deadband [m/s]");
    declareDouble("angular_deadband", 0.02, 0.0, 1.0, "angular deadband [rad/s]");

    declareDouble("cmd_timeout", 0.5, 0.01, 60.0, "watchdog timeout [s]");
    declareDouble(
      "emergency_decel_factor", 2.0, 1.0, 20.0,
      "deceleration multiplier while failing safe");
    declare_parameter("estop_hard_stop", true);

    declareDouble(
      "log_throttle_sec", 2.0, 0.0, 60.0, "minimum interval between repeated warnings [s]");
  }

  Limits limitsFromParameters() const
  {
    Limits l;
    l.max_linear_x = get_parameter("max_linear_x").as_double();
    l.min_linear_x = get_parameter("min_linear_x").as_double();
    l.max_angular_z = get_parameter("max_angular_z").as_double();
    l.max_linear_accel = get_parameter("max_linear_accel").as_double();
    l.max_linear_decel = get_parameter("max_linear_decel").as_double();
    l.max_angular_accel = get_parameter("max_angular_accel").as_double();
    l.max_angular_decel = get_parameter("max_angular_decel").as_double();
    l.max_lateral_accel = get_parameter("max_lateral_accel").as_double();
    l.enforce_nonholonomic = get_parameter("enforce_nonholonomic").as_bool();
    l.nonholonomic_epsilon = get_parameter("nonholonomic_epsilon").as_double();
    l.max_consecutive_invalid =
      static_cast<int>(get_parameter("max_consecutive_invalid").as_int());
    l.spike_rejection_enabled = get_parameter("spike_rejection_enabled").as_bool();
    l.spike_linear_threshold = get_parameter("spike_linear_threshold").as_double();
    l.spike_angular_threshold = get_parameter("spike_angular_threshold").as_double();
    l.spike_confirm_count = static_cast<int>(get_parameter("spike_confirm_count").as_int());
    l.linear_deadband = get_parameter("linear_deadband").as_double();
    l.angular_deadband = get_parameter("angular_deadband").as_double();
    l.cmd_timeout = get_parameter("cmd_timeout").as_double();
    l.emergency_decel_factor = get_parameter("emergency_decel_factor").as_double();
    l.estop_hard_stop = get_parameter("estop_hard_stop").as_bool();
    return l;
  }

  /// Parameters are an external input too, so they get validated like one.
  /// A candidate Limits is built from the whole batch and only committed if
  /// the complete set is self-consistent -- a rejected batch leaves the node
  /// running on the previous, known-good configuration.
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;

    Limits cand = limits_;
    double rate = control_rate_hz_;
    double throttle = log_throttle_sec_;

    for (const auto & p : params) {
      const std::string & n = p.get_name();
      if (n == "max_linear_x") {cand.max_linear_x = p.as_double();} else if (
        n == "min_linear_x") {cand.min_linear_x = p.as_double();} else if (
        n == "max_angular_z") {cand.max_angular_z = p.as_double();} else if (
        n == "max_linear_accel") {cand.max_linear_accel = p.as_double();} else if (
        n == "max_linear_decel") {cand.max_linear_decel = p.as_double();} else if (
        n == "max_angular_accel") {cand.max_angular_accel = p.as_double();} else if (
        n == "max_angular_decel") {cand.max_angular_decel = p.as_double();} else if (
        n == "max_lateral_accel") {cand.max_lateral_accel = p.as_double();} else if (
        n == "enforce_nonholonomic") {cand.enforce_nonholonomic = p.as_bool();} else if (
        n == "nonholonomic_epsilon") {cand.nonholonomic_epsilon = p.as_double();} else if (
        n == "max_consecutive_invalid")
      {
        cand.max_consecutive_invalid = static_cast<int>(p.as_int());
      } else if (n == "spike_rejection_enabled") {
        cand.spike_rejection_enabled = p.as_bool();
      } else if (n == "spike_linear_threshold") {
        cand.spike_linear_threshold = p.as_double();
      } else if (n == "spike_angular_threshold") {
        cand.spike_angular_threshold = p.as_double();
      } else if (n == "spike_confirm_count") {
        cand.spike_confirm_count = static_cast<int>(p.as_int());
      } else if (n == "linear_deadband") {
        cand.linear_deadband = p.as_double();
      } else if (n == "angular_deadband") {
        cand.angular_deadband = p.as_double();
      } else if (n == "cmd_timeout") {
        cand.cmd_timeout = p.as_double();
      } else if (n == "emergency_decel_factor") {
        cand.emergency_decel_factor = p.as_double();
      } else if (n == "estop_hard_stop") {
        cand.estop_hard_stop = p.as_bool();
      } else if (n == "control_rate_hz") {
        rate = p.as_double();
      } else if (n == "log_throttle_sec") {
        throttle = p.as_double();
      }
    }

    // Cross-field consistency checks the declared ranges cannot express.
    const char * why = nullptr;
    if (!(cand.max_linear_x > 0.0)) {
      why = "max_linear_x must be > 0 (a robot that cannot move forward is not useful)";
    } else if (cand.min_linear_x > 0.0) {
      why = "min_linear_x must be <= 0 (it is the reverse limit)";
    } else if (!(cand.max_angular_z >= 0.0)) {
      why = "max_angular_z must be >= 0";
    } else if (!(cand.max_linear_accel > 0.0) || !(cand.max_linear_decel > 0.0)) {
      why = "linear accel/decel limits must be > 0";
    } else if (!(cand.max_angular_accel > 0.0) || !(cand.max_angular_decel > 0.0)) {
      why = "angular accel/decel limits must be > 0";
    } else if (!(cand.cmd_timeout > 0.0)) {
      why = "cmd_timeout must be > 0";
    } else if (cand.spike_confirm_count < 1) {
      why = "spike_confirm_count must be >= 1";
    } else if (cand.max_consecutive_invalid < 1) {
      why = "max_consecutive_invalid must be >= 1";
    } else if (!(rate > 0.0)) {
      why = "control_rate_hz must be > 0";
    } else if (cand.cmd_timeout < 2.0 / rate) {
      why = "cmd_timeout must span at least two control cycles, "
        "otherwise the watchdog trips on normal jitter";
    }

    if (why != nullptr) {
      res.successful = false;
      res.reason = why;
      RCLCPP_WARN(get_logger(), "rejected parameter update: %s", why);
      return res;
    }

    limits_ = cand;
    limiter_.setLimits(limits_);
    log_throttle_sec_ = throttle;
    if (rate != control_rate_hz_) {
      control_rate_hz_ = rate;
      startTimer();
      RCLCPP_INFO(get_logger(), "control rate changed to %.1f Hz", control_rate_hz_);
    }
    return res;
  }

  void startTimer()
  {
    if (timer_) {
      timer_->cancel();
    }
    period_ = 1.0 / control_rate_hz_;
    // A ROS-time timer (not a wall timer) so that `use_sim_time` + `ros2 bag
    // play --clock` drives the control loop off the replay clock. With a wall
    // timer, replaying at any rate other than 1.0 would desynchronise dt from
    // the message timestamps and corrupt every acceleration computation.
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration(std::chrono::duration<double>(period_)),
      std::bind(&VelocityGuard::onTimer, this));
  }

  // ------------------------------------------------------------- callbacks --
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    Twist6 raw;
    raw.lx = msg->linear.x;
    raw.ly = msg->linear.y;
    raw.lz = msg->linear.z;
    raw.ax = msg->angular.x;
    raw.ay = msg->angular.y;
    raw.az = msg->angular.z;

    const auto r = limiter_.submit(raw, nowSeconds());

    // Flags are accumulated rather than overwritten: when the input rate
    // exceeds the output rate, several frames share one report and no event
    // may be lost.
    pending_flags_ |= r.flags;
    for (const auto & reason : r.reasons) {
      if (pending_reasons_.size() < kMaxReasons) {
        pending_reasons_.push_back(reason);
      }
    }

    if (r.flags & FLAG_NON_FINITE) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), throttleMs(),
        "non-finite /cmd_vel received (lx=%f ly=%f lz=%f ax=%f ay=%f az=%f): frame dropped",
        raw.lx, raw.ly, raw.lz, raw.ax, raw.ay, raw.az);
    }
    if (r.flags & FLAG_SPIKE_REJECTED) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), throttleMs(),
        "large jump in /cmd_vel (v=%.3f w=%.3f): output held for one frame; it is "
        "adopted if the next frame confirms it, discarded as a spike otherwise",
        raw.lx, raw.az);
    }
    if (r.flags & FLAG_NONHOLONOMIC) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), throttleMs(),
        "/cmd_vel requests unexecutable DOF (ly=%.3f lz=%.3f ax=%.3f ay=%.3f) on a "
        "differential-drive base: projected to zero", raw.ly, raw.lz, raw.ax, raw.ay);
    }
  }

  void onEstop(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data == limiter_.estop()) {
      return;
    }
    limiter_.setEstop(msg->data);
    if (msg->data) {
      RCLCPP_ERROR(get_logger(), "EMERGENCY STOP engaged: output forced to zero");
    } else {
      RCLCPP_WARN(
        get_logger(), "emergency stop released: target reset to zero, "
        "a fresh command is required to move");
    }
  }

  void onTimer()
  {
    const double now = nowSeconds();

    // Detect a backwards clock jump (bag replay looping, sim reset). Carrying
    // stale timestamps across it would make dt and the watchdog meaningless.
    if (have_last_update_ && now < last_update_time_) {
      RCLCPP_WARN(
        get_logger(), "clock jumped backwards (%.3f -> %.3f): resetting safety state",
        last_update_time_, now);
      limiter_.reset();
      have_last_update_ = false;
    }

    const double dt = have_last_update_ ? (now - last_update_time_) : period_;
    last_update_time_ = now;
    have_last_update_ = true;

    const auto c = limiter_.update(now, dt);

    geometry_msgs::msg::Twist out;
    out.linear.x = c.output.v;
    out.angular.z = c.output.w;
    // linear.y/z and angular.x/y stay at zero: a differential-drive base
    // cannot execute them, so publishing anything else would be a lie.
    cmd_pub_->publish(out);

    publishReport(now, c);
    logTransitions(c);
  }

  void publishReport(double now, const CycleResult & c)
  {
    SafetyReport rep;
    rep.header.stamp = this->now();
    rep.header.frame_id = "base_link";

    rep.flags = static_cast<uint16_t>(c.flags | pending_flags_);
    rep.reasons = pending_reasons_;
    rep.reasons.insert(rep.reasons.end(), c.reasons.begin(), c.reasons.end());

    const Twist6 & raw = limiter_.lastRawInput();
    rep.input_cmd.linear.x = raw.lx;
    rep.input_cmd.linear.y = raw.ly;
    rep.input_cmd.linear.z = raw.lz;
    rep.input_cmd.angular.x = raw.ax;
    rep.input_cmd.angular.y = raw.ay;
    rep.input_cmd.angular.z = raw.az;

    rep.target_cmd.linear.x = c.target.v;
    rep.target_cmd.angular.z = c.target.w;
    rep.output_cmd.linear.x = c.output.v;
    rep.output_cmd.angular.z = c.output.w;

    const double since = limiter_.timeSinceLastInput(now);
    rep.input_rate_hz = limiter_.inputRateHz();
    rep.time_since_last_input = std::isfinite(since) ? since : -1.0;
    rep.watchdog_active = c.watchdog_active;
    rep.estop_active = c.estop_active;

    rep.total_received = limiter_.totalReceived();
    rep.total_rejected = limiter_.totalRejected();
    rep.total_modified = limiter_.totalModified();
    rep.total_published = limiter_.totalPublished();

    report_pub_->publish(rep);

    pending_flags_ = FLAG_NONE;
    pending_reasons_.clear();
  }

  /// Edge-triggered logging for the sticky conditions, so a 4-second signal
  /// loss produces two log lines instead of eighty.
  void logTransitions(const CycleResult & c)
  {
    if (c.watchdog_active != was_watchdog_) {
      if (c.watchdog_active) {
        RCLCPP_WARN(
          get_logger(),
          "/cmd_vel timed out after %.2f s: braking to zero and resetting the target",
          limits_.cmd_timeout);
      } else {
        RCLCPP_INFO(
          get_logger(), "/cmd_vel stream recovered: ramping up from standstill");
      }
      was_watchdog_ = c.watchdog_active;
    }
    if (c.safety_hold != was_hold_) {
      if (c.safety_hold) {
        RCLCPP_ERROR(
          get_logger(), "safety hold: %d consecutive invalid commands, braking to zero",
          limits_.max_consecutive_invalid);
      } else {
        RCLCPP_INFO(get_logger(), "safety hold released: upstream is valid again");
      }
      was_hold_ = c.safety_hold;
    }
    const uint16_t clamp_bits =
      FLAG_LINEAR_CLAMPED | FLAG_ANGULAR_CLAMPED | FLAG_LATERAL_ACCEL_LIMITED;
    if (c.flags & clamp_bits) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), throttleMs(),
        "command outside the kinematic envelope: target (v=%.3f w=%.3f) -> output "
        "(v=%.3f w=%.3f)", c.target.v, c.target.w, c.output.v, c.output.w);
    }
  }

  double nowSeconds() const {return this->now().seconds();}
  uint64_t throttleMs() const
  {
    return static_cast<uint64_t>(std::max(0.0, log_throttle_sec_) * 1000.0);
  }

  static constexpr size_t kMaxReasons = 16;

  SafetyLimiter limiter_;
  Limits limits_;
  double control_rate_hz_ = 20.0;
  double period_ = 0.05;
  double log_throttle_sec_ = 2.0;

  uint16_t pending_flags_ = FLAG_NONE;
  std::vector<std::string> pending_reasons_;

  double last_update_time_ = 0.0;
  bool have_last_update_ = false;
  bool was_watchdog_ = false;
  bool was_hold_ = false;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<SafetyReport>::SharedPtr report_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace cmd_vel_safety

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Single-threaded on purpose: all callbacks mutate the same SafetyLimiter,
  // and serialising them removes the need for locking on the control path.
  rclcpp::spin(std::make_shared<cmd_vel_safety::VelocityGuard>());
  rclcpp::shutdown();
  return 0;
}

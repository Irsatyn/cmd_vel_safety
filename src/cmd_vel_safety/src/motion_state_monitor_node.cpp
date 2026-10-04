// Copyright 2026 cmd_vel_safety contributors. Apache-2.0.
//
// motion_state_monitor: classifies what the robot is actually doing and how
// healthy the command link is.
//
// Deliberately kept off the control path. It does windowed statistics, state
// machines and string formatting -- work that must never be able to delay or
// crash the safety gateway. It also measures the /cmd_vel rate itself instead
// of trusting the guard's self-reported figure: a monitor that believes the
// component it monitors is not a monitor.

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <stdexcept>
#include <vector>

#include "cmd_vel_safety/safety_limiter.hpp"
#include "cmd_vel_safety_msgs/msg/motion_state.hpp"
#include "cmd_vel_safety_msgs/msg/safety_report.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace cmd_vel_safety
{

using MotionState = cmd_vel_safety_msgs::msg::MotionState;
using SafetyReport = cmd_vel_safety_msgs::msg::SafetyReport;
using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using KeyValue = diagnostic_msgs::msg::KeyValue;

/// Counts arrivals inside a sliding time window. Preferred over an
/// exponential average because a hard dropout shows up immediately as a
/// falling rate instead of decaying slowly.
class RateWindow
{
public:
  void add(double t)
  {
    stamps_.push_back(t);
    last_ = t;
    have_ = true;
  }

  double hz(double now, double window)
  {
    while (!stamps_.empty() && stamps_.front() < now - window) {
      stamps_.pop_front();
    }
    if (stamps_.size() < 2) {
      return 0.0;
    }
    return static_cast<double>(stamps_.size()) / window;
  }

  bool have() const {return have_;}
  double last() const {return last_;}
  double since(double now) const
  {
    return have_ ? (now - last_) : std::numeric_limits<double>::infinity();
  }

private:
  std::deque<double> stamps_;
  double last_ = 0.0;
  bool have_ = false;
};

class MotionStateMonitor : public rclcpp::Node
{
public:
  MotionStateMonitor()
  : Node("motion_state_monitor")
  {
    declareParameters();
    const auto error = validateParameters({});
    if (!error.empty()) {throw std::invalid_argument(error);}
    loadParameters();

    rclcpp::QoS cmd_qos(rclcpp::KeepLast(10));
    cmd_qos.best_effort();  // compatible with reliable and best-effort producers

    raw_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      get_parameter("cmd_vel_topic").as_string(), cmd_qos,
      std::bind(&MotionStateMonitor::onRaw, this, std::placeholders::_1));
    safe_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      get_parameter("cmd_vel_safe_topic").as_string(), rclcpp::QoS(rclcpp::KeepLast(10)),
      std::bind(&MotionStateMonitor::onSafe, this, std::placeholders::_1));
    report_sub_ = create_subscription<SafetyReport>(
      get_parameter("report_topic").as_string(), rclcpp::QoS(rclcpp::KeepLast(10)),
      std::bind(&MotionStateMonitor::onReport, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), rclcpp::QoS(rclcpp::KeepLast(20)),
      std::bind(&MotionStateMonitor::onOdom, this, std::placeholders::_1));

    state_pub_ = create_publisher<MotionState>(
      get_parameter("state_topic").as_string(), rclcpp::QoS(rclcpp::KeepLast(10)));
    text_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("status_topic").as_string(), rclcpp::QoS(rclcpp::KeepLast(10)));
    diag_pub_ = create_publisher<DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(rclcpp::KeepLast(10)));

    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult r;
        r.reason = validateParameters(params);
        r.successful = r.reason.empty();
        if (r.successful) {reload_pending_ = true;}
        return r;
      });

    restartTimer();

    state_entered_ = nowSeconds();
    RCLCPP_INFO(get_logger(), "motion_state_monitor up @ %.1f Hz", publish_rate_hz_);
  }

private:
  // ---------------------------------------------------------------- params --
  void declareParameters()
  {
    auto ro = [](const std::string & d) {
        rcl_interfaces::msg::ParameterDescriptor pd;
        pd.description = d + " (read-only)";
        pd.read_only = true;
        return pd;
      };
    declare_parameter("cmd_vel_topic", "/cmd_vel", ro("raw command topic to observe"));
    declare_parameter(
      "cmd_vel_safe_topic", "/cmd_vel_safe", ro("processed command topic to observe"));
    declare_parameter(
      "report_topic", "/velocity_guard/report", ro("safety report topic"));
    declare_parameter("odom_topic", "/odom", ro("measured odometry topic"));
    declare_parameter("state_topic", "/robot_motion_state", ro("structured state output"));
    declare_parameter("status_topic", "/robot_status", ro("human-readable status output"));

    declare_parameter("publish_rate_hz", 5.0);
    declare_parameter("idle_linear_threshold", 0.02);
    declare_parameter("idle_angular_threshold", 0.05);
    declare_parameter("turn_in_place_linear_threshold", 0.05);
    declare_parameter("expected_input_rate_hz", 10.0);
    declare_parameter("input_rate_tolerance", 0.5);
    declare_parameter("rate_window", 2.0);
    declare_parameter("signal_lost_timeout", 1.0);
    declare_parameter("frozen_input_timeout", 10.0);
    declare_parameter("tracking_error_warn", 0.15);
    declare_parameter("odom_stale_timeout", 1.0);
  }

  std::string validateParameters(const std::vector<rclcpp::Parameter> & params) const
  {
    const std::vector<std::string> names = {
      "publish_rate_hz", "idle_linear_threshold", "idle_angular_threshold",
      "turn_in_place_linear_threshold", "expected_input_rate_hz", "input_rate_tolerance",
      "rate_window", "signal_lost_timeout", "frozen_input_timeout", "tracking_error_warn",
      "odom_stale_timeout"};
    for (const auto & name : names) {
      double value = get_parameter(name).as_double();
      for (const auto & p : params) {
        if (p.get_name() == name) {
          if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
            return name + " must be a double";
          }
          value = p.as_double();
        }
      }
      const bool zero_allowed = name == "expected_input_rate_hz" ||
        name == "input_rate_tolerance" || name == "tracking_error_warn";
      if (!std::isfinite(value) || (zero_allowed ? value < 0.0 : value <= 0.0)) {
        return name + " must be finite and " + (zero_allowed ? "nonnegative" : "positive");
      }
      if (name == "publish_rate_hz" && (value < 1.0 || value > 1000.0)) {
        return "publish_rate_hz must be in [1, 1000]";
      }
    }
    return "";
  }

  void restartTimer()
  {
    if (timer_) {timer_->cancel();}
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration(std::chrono::duration<double>(1.0 / publish_rate_hz_)),
      std::bind(&MotionStateMonitor::onTimer, this));
  }

  void loadParameters()
  {
    idle_lin_ = get_parameter("idle_linear_threshold").as_double();
    idle_ang_ = get_parameter("idle_angular_threshold").as_double();
    turn_lin_ = get_parameter("turn_in_place_linear_threshold").as_double();
    expected_hz_ = get_parameter("expected_input_rate_hz").as_double();
    rate_tol_ = get_parameter("input_rate_tolerance").as_double();
    rate_window_ = get_parameter("rate_window").as_double();
    signal_lost_timeout_ = get_parameter("signal_lost_timeout").as_double();
    frozen_timeout_ = get_parameter("frozen_input_timeout").as_double();
    tracking_warn_ = get_parameter("tracking_error_warn").as_double();
    odom_stale_timeout_ = get_parameter("odom_stale_timeout").as_double();
  }

  // ------------------------------------------------------------- callbacks --
  void onRaw(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double t = nowSeconds();
    raw_rate_.add(t);

    // Frozen-publisher detection: an upstream node that has hung keeps
    // republishing one buffered value. Steady cruising looks identical, so
    // this only ever raises a warning -- never an intervention.
    const bool same = raw_have_ &&
      msg->linear.x == last_raw_.linear.x && msg->angular.z == last_raw_.angular.z &&
      msg->linear.y == last_raw_.linear.y && msg->linear.z == last_raw_.linear.z &&
      msg->angular.x == last_raw_.angular.x && msg->angular.y == last_raw_.angular.y;
    if (!same) {
      raw_value_changed_at_ = t;
    }
    last_raw_ = *msg;
    raw_have_ = true;
  }

  void onSafe(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {return;}
    safe_rate_.add(nowSeconds());
    last_safe_ = *msg;
    safe_have_ = true;
  }

  void onReport(const SafetyReport::SharedPtr msg)
  {
    last_flags_ = msg->flags;
    // An intervention is any flag beyond the purely informational ones.
    if (msg->flags != 0) {
      ++intervention_count_;
    }
    guard_watchdog_ = msg->watchdog_active;
    guard_estop_ = msg->estop_active;
    guard_hold_ = (msg->flags & FLAG_SAFETY_HOLD) != 0;
    guard_seen_ = true;
    guard_last_ = nowSeconds();
    if (!msg->reasons.empty()) {
      last_reason_ = msg->reasons.back();
    }
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double t = nowSeconds();
    const double v = msg->twist.twist.linear.x;
    const double w = msg->twist.twist.angular.z;
    const auto & q = msg->pose.pose.orientation;
    const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    if (!std::isfinite(v) || !std::isfinite(w) || !std::isfinite(norm) || norm < 1e-12) {
      invalid_odom_ = true;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "invalid odometry feedback ignored");
      return;
    }
    invalid_odom_ = false;

    if (odom_have_ && t > odom_last_) {
      const double dt = t - odom_last_;
      if (dt < 1.0) {  // ignore integration across a dropout
        distance_ += std::fabs(v) * dt;
      }
    }
    odom_v_ = v;
    odom_w_ = w;
    // yaw from the quaternion; roll/pitch are always zero for a ground base
    const double qx = q.x / norm, qy = q.y / norm, qz = q.z / norm, qw = q.w / norm;
    heading_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    odom_last_ = t;
    odom_have_ = true;
  }

  // ----------------------------------------------------------- state logic --
  uint8_t classify(double v, double w, bool signal_lost) const
  {
    // Fault conditions outrank motion description: an operator needs to know
    // *why* the robot is not moving, not merely that it is stopped.
    if (guard_estop_) {return MotionState::STATE_EMERGENCY_STOP;}
    if (guard_hold_) {return MotionState::STATE_SAFETY_HOLD;}
    if (velocity_source_ == MotionState::SOURCE_UNKNOWN) {return MotionState::STATE_UNKNOWN;}
    if (signal_lost) {return MotionState::STATE_SIGNAL_LOST;}

    const bool moving = std::fabs(v) >= idle_lin_ || std::fabs(w) >= idle_ang_;
    if (!moving) {return MotionState::STATE_IDLE;}

    // Commanded to stop but still rolling -> braking.
    const bool cmd_zero = safe_have_ &&
      std::fabs(last_safe_.linear.x) < idle_lin_ &&
      std::fabs(last_safe_.angular.z) < idle_ang_;
    if (cmd_zero) {return MotionState::STATE_BRAKING;}

    const bool turning = std::fabs(w) >= idle_ang_;
    if (turning && std::fabs(v) < turn_lin_) {return MotionState::STATE_TURN_IN_PLACE;}
    if (turning) {
      return (w > 0.0) ? MotionState::STATE_ARC_LEFT : MotionState::STATE_ARC_RIGHT;
    }
    return (v >= 0.0) ? MotionState::STATE_FORWARD : MotionState::STATE_REVERSE;
  }

  static const char * stateName(uint8_t s)
  {
    switch (s) {
      case MotionState::STATE_UNKNOWN: return "UNKNOWN";
      case MotionState::STATE_IDLE: return "IDLE";
      case MotionState::STATE_FORWARD: return "FORWARD";
      case MotionState::STATE_REVERSE: return "REVERSE";
      case MotionState::STATE_TURN_IN_PLACE: return "TURN_IN_PLACE";
      case MotionState::STATE_ARC_LEFT: return "ARC_LEFT";
      case MotionState::STATE_ARC_RIGHT: return "ARC_RIGHT";
      case MotionState::STATE_BRAKING: return "BRAKING";
      case MotionState::STATE_SAFETY_HOLD: return "SAFETY_HOLD";
      case MotionState::STATE_EMERGENCY_STOP: return "EMERGENCY_STOP";
      case MotionState::STATE_SIGNAL_LOST: return "SIGNAL_LOST";
      default: return "UNKNOWN";
    }
  }

  void onTimer()
  {
    if (reload_pending_) {
      loadParameters();
      if (get_parameter("publish_rate_hz").as_double() != publish_rate_hz_) {restartTimer();}
      reload_pending_ = false;
    }

    const double now = nowSeconds();
    if (have_last_tick_ && now < last_tick_) {
      RCLCPP_WARN(get_logger(), "clock jumped backwards: resetting monitor statistics");
      resetStatistics();
    }
    last_tick_ = now;
    have_last_tick_ = true;

    const double raw_hz = raw_rate_.hz(now, rate_window_);
    const double safe_hz = safe_rate_.hz(now, rate_window_);
    const double since_raw = raw_rate_.since(now);
    const bool signal_lost = !raw_rate_.have() || since_raw > signal_lost_timeout_;
    const bool odom_stale = !odom_have_ || (now - odom_last_) > odom_stale_timeout_;

    double v = odom_v_;
    double w = odom_w_;
    velocity_source_ = MotionState::SOURCE_ODOMETRY;
    if (odom_stale) {
      const bool safe_fresh = safe_have_ && safe_rate_.since(now) <= signal_lost_timeout_;
      velocity_source_ = safe_fresh ? MotionState::SOURCE_COMMAND : MotionState::SOURCE_UNKNOWN;
      v = safe_fresh ? last_safe_.linear.x : 0.0;
      w = safe_fresh ? last_safe_.angular.z : 0.0;
    }

    const bool frozen = raw_have_ && !signal_lost &&
      (now - raw_value_changed_at_) > frozen_timeout_ &&
      (std::fabs(last_raw_.linear.x) > idle_lin_ ||
      std::fabs(last_raw_.angular.z) > idle_ang_);

    const uint8_t state = classify(v, w, signal_lost);
    if (state != state_) {
      state_ = state;
      state_entered_ = now;
      RCLCPP_INFO(get_logger(), "motion state -> %s", stateName(state));
    }

    const double track_lin = safe_have_ && !odom_stale ?
      std::fabs(last_safe_.linear.x - odom_v_) : 0.0;
    const double track_ang = safe_have_ && !odom_stale ?
      std::fabs(last_safe_.angular.z - odom_w_) : 0.0;

    // ---- health ----
    uint8_t health = MotionState::HEALTH_OK;
    std::string hmsg = "nominal";
    if (guard_estop_) {
      health = MotionState::HEALTH_ERROR;
      hmsg = "emergency stop engaged";
    } else if (guard_hold_) {
      health = MotionState::HEALTH_ERROR;
      hmsg = "safety hold: upstream persistently invalid";
    } else if (velocity_source_ == MotionState::SOURCE_UNKNOWN) {
      health = MotionState::HEALTH_STALE;
      hmsg = "no fresh odometry or safe command: motion unknown";
    } else if (invalid_odom_) {
      health = MotionState::HEALTH_WARN;
      hmsg = "invalid odometry feedback ignored";
    } else if (signal_lost) {
      health = MotionState::HEALTH_ERROR;
      hmsg = raw_rate_.have() ?
        ("no /cmd_vel for " + fmt(since_raw) + " s") : "no /cmd_vel received yet";
    } else if (!guard_seen_ || (now - guard_last_) > 1.0) {
      health = MotionState::HEALTH_STALE;
      hmsg = "no safety report from velocity_guard: is the guard running?";
    } else if (odom_stale) {
      health = MotionState::HEALTH_STALE;
      hmsg = "no /odom feedback: monitoring commanded velocity only";
    } else if (frozen) {
      health = MotionState::HEALTH_WARN;
      hmsg = "upstream has repeated the same non-zero command for " +
        fmt(now - raw_value_changed_at_) + " s: publisher may be hung";
    } else if (expected_hz_ > 0.0 &&
      std::fabs(raw_hz - expected_hz_) > rate_tol_ * expected_hz_)
    {
      health = MotionState::HEALTH_WARN;
      hmsg = "/cmd_vel rate " + fmt(raw_hz) + " Hz deviates from the expected " +
        fmt(expected_hz_) + " Hz";
    } else if (track_lin > tracking_warn_) {
      health = MotionState::HEALTH_WARN;
      hmsg = "linear tracking error " + fmt(track_lin) + " m/s: the base is not keeping up";
    } else if (last_flags_ != 0) {
      health = MotionState::HEALTH_WARN;
      hmsg = "safety intervention active: " + last_reason_;
    }

    publishState(
      now, state, health, hmsg, v, w, raw_hz, safe_hz, since_raw, track_lin, track_ang,
      frozen);
  }

  void publishState(
    double now, uint8_t state, uint8_t health, const std::string & hmsg,
    double v, double w, double raw_hz, double safe_hz, double since_raw,
    double track_lin, double track_ang, bool frozen)
  {
    MotionState m;
    m.header.stamp = this->now();
    m.header.frame_id = "base_link";
    m.velocity_source = velocity_source_;
    m.state = state;
    m.state_name = stateName(state);
    m.health = health;
    m.health_message = hmsg;
    m.linear_speed = v;
    m.angular_speed = w;
    m.turn_radius = (std::fabs(w) > 1e-6) ?
      (v / w) : std::numeric_limits<double>::infinity();
    m.heading = heading_;
    m.distance_travelled = distance_;
    m.duration_in_state = now - state_entered_;
    m.cmd_input_rate_hz = raw_hz;
    m.safe_cmd_rate_hz = safe_hz;
    m.time_since_last_cmd = std::isfinite(since_raw) ? since_raw : -1.0;
    m.tracking_error_linear = track_lin;
    m.tracking_error_angular = track_ang;
    m.input_frozen = frozen;
    m.intervention_count = intervention_count_;
    m.last_safety_flags = last_flags_;
    state_pub_->publish(m);

    std::ostringstream os;
    os.precision(2);
    os << std::fixed
       << "[" << m.state_name << " " << fmt(m.duration_in_state) << "s] "
       << "v=" << (v >= 0 ? "+" : "") << v << " m/s  "
       << "w=" << (w >= 0 ? "+" : "") << w << " rad/s  ";
    if (std::isfinite(m.turn_radius)) {
      os << "R=" << m.turn_radius << " m  ";
    } else {
      os << "R=inf  ";
    }
    os << "| dist=" << m.distance_travelled << " m "
       << "| cmd_in=" << raw_hz << " Hz safe_out=" << safe_hz << " Hz "
       << "| interventions=" << intervention_count_;
    const auto names = describeFlags(last_flags_);
    if (!names.empty()) {
      os << " [";
      for (size_t i = 0; i < names.size(); ++i) {
        os << (i ? "," : "") << names[i];
      }
      os << "]";
    }
    os << " | " << healthName(health) << ": " << hmsg;

    std_msgs::msg::String s;
    s.data = os.str();
    text_pub_->publish(s);

    publishDiagnostics(m, raw_hz, safe_hz, hmsg);
  }

  void publishDiagnostics(
    const MotionState & m, double raw_hz, double safe_hz, const std::string & hmsg)
  {
    DiagnosticArray arr;
    arr.header.stamp = this->now();

    DiagnosticStatus link;
    link.name = "cmd_vel_safety: command link";
    link.hardware_id = "velocity_guard";
    link.level = toDiagLevel(m.health);
    link.message = hmsg;
    link.values.push_back(kv("raw /cmd_vel rate [Hz]", fmt(raw_hz)));
    link.values.push_back(kv("safe /cmd_vel_safe rate [Hz]", fmt(safe_hz)));
    link.values.push_back(kv("time since last command [s]", fmt(m.time_since_last_cmd)));
    link.values.push_back(kv("interventions", std::to_string(m.intervention_count)));
    link.values.push_back(kv("active flags", joinFlags(m.last_safety_flags)));
    link.values.push_back(kv("input frozen", m.input_frozen ? "true" : "false"));
    arr.status.push_back(link);

    DiagnosticStatus motion;
    motion.name = "cmd_vel_safety: motion";
    motion.hardware_id = "base";
    motion.level = (m.state == MotionState::STATE_EMERGENCY_STOP ||
      m.state == MotionState::STATE_SAFETY_HOLD) ? DiagnosticStatus::ERROR :
      (m.velocity_source == MotionState::SOURCE_UNKNOWN ? DiagnosticStatus::STALE :
      (m.state == MotionState::STATE_SIGNAL_LOST ? DiagnosticStatus::WARN :
      DiagnosticStatus::OK));
    motion.message = m.state_name;
    motion.values.push_back(kv("linear [m/s]", fmt(m.linear_speed)));
    motion.values.push_back(kv("angular [rad/s]", fmt(m.angular_speed)));
    motion.values.push_back(kv("heading [rad]", fmt(m.heading)));
    motion.values.push_back(kv("distance [m]", fmt(m.distance_travelled)));
    motion.values.push_back(kv("time in state [s]", fmt(m.duration_in_state)));
    motion.values.push_back(kv("tracking error [m/s]", fmt(m.tracking_error_linear)));
    arr.status.push_back(motion);

    diag_pub_->publish(arr);
  }

  // ------------------------------------------------------------- utilities --
  static uint8_t toDiagLevel(uint8_t health)
  {
    switch (health) {
      case MotionState::HEALTH_OK: return DiagnosticStatus::OK;
      case MotionState::HEALTH_WARN: return DiagnosticStatus::WARN;
      case MotionState::HEALTH_ERROR: return DiagnosticStatus::ERROR;
      default: return DiagnosticStatus::STALE;
    }
  }

  static const char * healthName(uint8_t h)
  {
    switch (h) {
      case MotionState::HEALTH_OK: return "OK";
      case MotionState::HEALTH_WARN: return "WARN";
      case MotionState::HEALTH_ERROR: return "ERROR";
      default: return "STALE";
    }
  }

  static KeyValue kv(const std::string & k, const std::string & v)
  {
    KeyValue x;
    x.key = k;
    x.value = v;
    return x;
  }

  static std::string joinFlags(uint16_t flags)
  {
    const auto names = describeFlags(flags);
    if (names.empty()) {
      return "none";
    }
    std::string s;
    for (size_t i = 0; i < names.size(); ++i) {
      s += (i ? "," : "") + names[i];
    }
    return s;
  }

  static std::string fmt(double v)
  {
    std::ostringstream os;
    os.precision(3);
    os << std::fixed << v;
    return os.str();
  }

  void resetStatistics()
  {
    raw_rate_ = RateWindow{};
    safe_rate_ = RateWindow{};
    distance_ = 0.0;
    odom_have_ = false;
    invalid_odom_ = false;
    guard_estop_ = guard_hold_ = guard_watchdog_ = false;
    guard_seen_ = false;
    raw_have_ = false;
    safe_have_ = false;
    intervention_count_ = 0;
    last_flags_ = 0;
  }

  double nowSeconds() const {return this->now().seconds();}

  // thresholds
  double idle_lin_ = 0.02, idle_ang_ = 0.05, turn_lin_ = 0.05;
  double expected_hz_ = 10.0, rate_tol_ = 0.5, rate_window_ = 2.0;
  double signal_lost_timeout_ = 1.0, frozen_timeout_ = 10.0;
  double tracking_warn_ = 0.15, odom_stale_timeout_ = 1.0;
  double publish_rate_hz_ = 5.0;
  uint8_t velocity_source_ = MotionState::SOURCE_UNKNOWN;
  bool invalid_odom_ = false;
  bool reload_pending_ = false;

  // observed state
  RateWindow raw_rate_, safe_rate_;
  geometry_msgs::msg::Twist last_raw_, last_safe_;
  bool raw_have_ = false, safe_have_ = false;
  double raw_value_changed_at_ = 0.0;

  double odom_v_ = 0.0, odom_w_ = 0.0, heading_ = 0.0, distance_ = 0.0;
  double odom_last_ = 0.0;
  bool odom_have_ = false;

  uint16_t last_flags_ = 0;
  std::string last_reason_;
  bool guard_seen_ = false, guard_watchdog_ = false, guard_estop_ = false,
    guard_hold_ = false;
  double guard_last_ = 0.0;
  uint32_t intervention_count_ = 0;

  uint8_t state_ = MotionState::STATE_IDLE;
  double state_entered_ = 0.0;
  double last_tick_ = 0.0;
  bool have_last_tick_ = false;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_sub_, safe_sub_;
  rclcpp::Subscription<SafetyReport>::SharedPtr report_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<MotionState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr text_pub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace cmd_vel_safety

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cmd_vel_safety::MotionStateMonitor>());
  rclcpp::shutdown();
  return 0;
}

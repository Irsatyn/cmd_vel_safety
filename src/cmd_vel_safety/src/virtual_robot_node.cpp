// Copyright 2026 cmd_vel_safety contributors. Apache-2.0.
//
// virtual_robot: a minimal differential-drive plant model standing in for a
// real base.
//
// It exists for two reasons beyond "having something to drive":
//   1. it closes the loop, so the monitor can compare commanded against
//      measured velocity and report a tracking error;
//   2. it carries its own, independent command watchdog. If velocity_guard
//      crashes outright, this layer still stops the robot. On real hardware
//      that responsibility belongs to the motor-driver firmware -- defence in
//      depth means never relying on a single node to fail safe.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

namespace cmd_vel_safety
{

class VirtualRobot : public rclcpp::Node
{
public:
  VirtualRobot()
  : Node("virtual_robot")
  {
    declare_parameter("cmd_topic", "/cmd_vel_safe");
    declare_parameter("odom_topic", "/odom");
    declare_parameter("update_rate_hz", 50.0);
    declare_parameter("linear_time_constant", 0.15);
    declare_parameter("angular_time_constant", 0.10);
    declare_parameter("cmd_timeout", 0.5);
    declare_parameter("odom_frame", "odom");
    declare_parameter("base_frame", "base_link");
    declare_parameter("publish_tf", true);

    tau_v_ = std::max(1e-3, get_parameter("linear_time_constant").as_double());
    tau_w_ = std::max(1e-3, get_parameter("angular_time_constant").as_double());
    timeout_ = get_parameter("cmd_timeout").as_double();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      get_parameter("cmd_topic").as_string(), rclcpp::QoS(rclcpp::KeepLast(10)),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Even the "safe" input is validated: a plant model that integrates a
        // NaN poisons its own pose forever and never recovers.
        if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "non-finite command on the safe topic: ignored");
          return;
        }
        cmd_v_ = msg->linear.x;
        cmd_w_ = msg->angular.z;
        last_cmd_ = now().seconds();
        have_cmd_ = true;
      });

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), rclcpp::QoS(rclcpp::KeepLast(20)));
    if (publish_tf_) {
      tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    const double rate = std::max(1.0, get_parameter("update_rate_hz").as_double());
    period_ = 1.0 / rate;
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration(std::chrono::duration<double>(period_)),
      std::bind(&VirtualRobot::step, this));

    RCLCPP_INFO(
      get_logger(),
      "virtual_robot up @ %.0f Hz (tau_v=%.3f s, tau_w=%.3f s, own watchdog %.2f s)",
      rate, tau_v_, tau_w_, timeout_);
  }

private:
  void step()
  {
    const double t = now().seconds();

    if (have_prev_ && t < prev_t_) {
      RCLCPP_WARN(get_logger(), "clock jumped backwards: resetting pose and velocity");
      x_ = y_ = th_ = act_v_ = act_w_ = 0.0;
      have_prev_ = false;
      have_cmd_ = false;
    }
    const double dt = have_prev_ ? (t - prev_t_) : period_;
    prev_t_ = t;
    have_prev_ = true;
    if (!(dt > 0.0) || dt > 1.0) {
      return;  // implausible step: skip integration rather than teleport
    }

    // Independent watchdog: no fresh command means stop, regardless of what
    // the upstream guard believes.
    double tv = cmd_v_;
    double tw = cmd_w_;
    const bool stale = !have_cmd_ || (t - last_cmd_) > timeout_;
    if (stale) {
      tv = 0.0;
      tw = 0.0;
      if (!was_stale_) {
        RCLCPP_WARN(
          get_logger(), "no command on the safe topic for %.2f s: base stopping itself",
          timeout_);
      }
    } else if (was_stale_) {
      RCLCPP_INFO(get_logger(), "command stream restored");
    }
    was_stale_ = stale;

    // First-order actuator lag: a real drivetrain cannot follow a step either,
    // which is exactly why the guard's slew limiting matters.
    const double kv = std::min(1.0, dt / tau_v_);
    const double kw = std::min(1.0, dt / tau_w_);
    act_v_ += (tv - act_v_) * kv;
    act_w_ += (tw - act_w_) * kw;

    // Mid-point heading integration: noticeably better than Euler on arcs.
    const double th_mid = th_ + 0.5 * act_w_ * dt;
    x_ += act_v_ * std::cos(th_mid) * dt;
    y_ += act_v_ * std::sin(th_mid) * dt;
    th_ = std::atan2(std::sin(th_ + act_w_ * dt), std::cos(th_ + act_w_ * dt));

    publishOdom(t);
  }

  void publishOdom(double)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, th_);

    nav_msgs::msg::Odometry o;
    o.header.stamp = now();
    o.header.frame_id = odom_frame_;
    o.child_frame_id = base_frame_;
    o.pose.pose.position.x = x_;
    o.pose.pose.position.y = y_;
    o.pose.pose.orientation.x = q.x();
    o.pose.pose.orientation.y = q.y();
    o.pose.pose.orientation.z = q.z();
    o.pose.pose.orientation.w = q.w();
    o.twist.twist.linear.x = act_v_;
    o.twist.twist.angular.z = act_w_;
    // Only the observable entries; the rest stay zero, which consumers read as
    // "unknown".
    o.pose.covariance[0] = 0.01;
    o.pose.covariance[7] = 0.01;
    o.pose.covariance[35] = 0.02;
    o.twist.covariance[0] = 0.01;
    o.twist.covariance[35] = 0.02;
    odom_pub_->publish(o);

    if (tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = o.header;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = x_;
      tf.transform.translation.y = y_;
      tf.transform.rotation = o.pose.pose.orientation;
      tf_->sendTransform(tf);
    }
  }

  double tau_v_ = 0.15, tau_w_ = 0.10, timeout_ = 0.5, period_ = 0.02;
  std::string odom_frame_, base_frame_;
  bool publish_tf_ = true;

  double cmd_v_ = 0.0, cmd_w_ = 0.0, last_cmd_ = 0.0;
  bool have_cmd_ = false, was_stale_ = false;
  double act_v_ = 0.0, act_w_ = 0.0;
  double x_ = 0.0, y_ = 0.0, th_ = 0.0;
  double prev_t_ = 0.0;
  bool have_prev_ = false;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace cmd_vel_safety

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cmd_vel_safety::VirtualRobot>());
  rclcpp::shutdown();
  return 0;
}

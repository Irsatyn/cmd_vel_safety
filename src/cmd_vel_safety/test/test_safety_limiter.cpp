// Copyright 2026 cmd_vel_safety contributors. Apache-2.0.
//
// Deterministic unit tests for the safety pipeline. Because the pipeline has
// no ROS dependency, each test drives it with an explicit timeline instead of
// relying on a live graph -- no scheduler jitter, no flaky timing.

#include <cmath>
#include <limits>
#include <vector>

#include "cmd_vel_safety/safety_limiter.hpp"
#include "gtest/gtest.h"

using cmd_vel_safety::CycleResult;
using cmd_vel_safety::Limits;
using cmd_vel_safety::SafetyLimiter;
using cmd_vel_safety::Twist6;

namespace
{

constexpr double kPeriod = 0.05;   // 20 Hz control rate
constexpr double kInputDt = 0.1;   // 10 Hz input, as in the supplied bag

Limits defaultLimits()
{
  return Limits{};  // the header defaults mirror config/params.yaml
}

/// Drives the limiter over a command sequence and returns every cycle result.
/// Commands are submitted at `kInputDt`, cycles run at `kPeriod`.
class Harness
{
public:
  explicit Harness(const Limits & l = defaultLimits())
  {
    limiter_.setLimits(l);
  }

  /// Submits `cmd` and then runs cycles for `duration` seconds.
  std::vector<CycleResult> feed(const Twist6 & cmd, double duration)
  {
    std::vector<CycleResult> out;
    const int frames = static_cast<int>(std::round(duration / kInputDt));
    for (int i = 0; i < frames; ++i) {
      limiter_.submit(cmd, t_);
      out.push_back(runCycles(kInputDt));
    }
    return out;
  }

  /// Submits one frame, then advances one input period of cycles.
  CycleResult feedOnce(const Twist6 & cmd)
  {
    limiter_.submit(cmd, t_);
    return runCycles(kInputDt);
  }

  /// Advances time with no input at all.
  std::vector<CycleResult> silence(double duration)
  {
    std::vector<CycleResult> out;
    const int n = static_cast<int>(std::round(duration / kPeriod));
    for (int i = 0; i < n; ++i) {
      t_ += kPeriod;
      out.push_back(limiter_.update(t_, kPeriod));
    }
    return out;
  }

  SafetyLimiter & limiter() {return limiter_;}
  double now() const {return t_;}

private:
  CycleResult runCycles(double duration)
  {
    CycleResult last;
    const int n = static_cast<int>(std::round(duration / kPeriod));
    for (int i = 0; i < n; ++i) {
      t_ += kPeriod;
      last = limiter_.update(t_, kPeriod);
    }
    return last;
  }

  SafetyLimiter limiter_;
  double t_ = 0.0;
};

Twist6 fwd(double v, double w = 0.0)
{
  Twist6 t;
  t.lx = v;
  t.az = w;
  return t;
}

}  // namespace

// ---------------------------------------------------------------------------
// G2: a normal command must pass through essentially unchanged.
// ---------------------------------------------------------------------------
TEST(SafetyLimiter, PassesNormalCommandThrough)
{
  Harness h;
  const auto r = h.feed(fwd(0.3), 3.0);
  EXPECT_NEAR(r.back().output.v, 0.3, 1e-9);
  EXPECT_NEAR(r.back().output.w, 0.0, 1e-9);
  EXPECT_EQ(r.back().flags, cmd_vel_safety::FLAG_NONE);
}

// ---------------------------------------------------------------------------
// Bag frames 220 / 225: NaN and Inf.
// ---------------------------------------------------------------------------
TEST(SafetyLimiter, DropsNonFiniteFrameAndHoldsLastTarget)
{
  Harness h;
  h.feed(fwd(0.25), 1.0);

  Twist6 nan_cmd = fwd(std::numeric_limits<double>::quiet_NaN());
  const auto v = h.limiter().submit(nan_cmd, h.now());
  EXPECT_FALSE(v.accepted);
  EXPECT_TRUE(v.flags & cmd_vel_safety::FLAG_NON_FINITE);

  Twist6 inf_cmd = fwd(0.25, std::numeric_limits<double>::infinity());
  const auto v2 = h.limiter().submit(inf_cmd, h.now());
  EXPECT_FALSE(v2.accepted);
  EXPECT_TRUE(v2.flags & cmd_vel_safety::FLAG_NON_FINITE);

  // The held target is still the last valid one, and the output is finite.
  EXPECT_NEAR(h.limiter().target().v, 0.25, 1e-9);
  const auto c = h.limiter().update(h.now() + kPeriod, kPeriod);
  EXPECT_TRUE(std::isfinite(c.output.v));
  EXPECT_TRUE(std::isfinite(c.output.w));
}

TEST(SafetyLimiter, EntersSafetyHoldOnPersistentlyInvalidUpstream)
{
  Limits l = defaultLimits();
  l.max_consecutive_invalid = 3;
  Harness h(l);
  h.feed(fwd(0.5), 2.0);

  const Twist6 nan_cmd = fwd(std::numeric_limits<double>::quiet_NaN());
  for (int i = 0; i < 3; ++i) {
    h.limiter().submit(nan_cmd, h.now());
  }
  // Isolated bad frames are tolerated; a persistent stream is not.
  const auto c = h.limiter().update(h.now() + kPeriod, kPeriod);
  EXPECT_TRUE(c.safety_hold);
  EXPECT_TRUE(c.flags & cmd_vel_safety::FLAG_SAFETY_HOLD);

  // ... and it brakes all the way to a stop.
  const auto stopped = h.silence(0.0);
  (void)stopped;
  for (int i = 0; i < 100; ++i) {
    h.limiter().submit(nan_cmd, h.now());
    h.feedOnce(nan_cmd);
  }
  EXPECT_NEAR(h.limiter().output().v, 0.0, 1e-6);
}

// ---------------------------------------------------------------------------
// Bag frame 230 vs frames 190-219: the central discrimination problem.
// ---------------------------------------------------------------------------
TEST(SafetyLimiter, RejectsIsolatedSpikeWithoutAffectingOutput)
{
  Harness h;
  h.feed(fwd(0.25), 1.0);
  const double before = h.limiter().output().v;

  // One frame of -2.2, exactly as in the bag.
  const auto spike = h.limiter().submit(fwd(-2.2), h.now());
  EXPECT_FALSE(spike.accepted);
  EXPECT_TRUE(spike.flags & cmd_vel_safety::FLAG_SPIKE_REJECTED);
  EXPECT_NEAR(h.limiter().target().v, 0.25, 1e-9);

  // The next frame returns to normal; the spike never reaches the output.
  const auto after = h.feedOnce(fwd(0.25));
  EXPECT_NEAR(after.output.v, before, 1e-6);
  EXPECT_GT(after.output.v, 0.0);
}

TEST(SafetyLimiter, AdoptsSustainedJumpAfterConfirmation)
{
  Harness h;
  h.feed(fwd(-0.35), 1.0);

  // First frame of the sustained 2.5 m/s run is held...
  const auto first = h.limiter().submit(fwd(2.5), h.now());
  EXPECT_FALSE(first.accepted);
  // ... the second confirms it.
  const auto second = h.limiter().submit(fwd(2.5), h.now() + kInputDt);
  EXPECT_TRUE(second.accepted);
  EXPECT_NEAR(h.limiter().target().v, 2.5, 1e-9);
}

// ---------------------------------------------------------------------------
// Bag frames 190-219: sustained over-limit command.
// ---------------------------------------------------------------------------
TEST(SafetyLimiter, ClampsSustainedOverLimitCommand)
{
  Harness h;
  const auto r = h.feed(fwd(2.5, 3.0), 4.0);
  const Limits l = defaultLimits();

  EXPECT_LE(r.back().output.v, l.max_linear_x + 1e-9);
  EXPECT_LE(std::fabs(r.back().output.w), l.max_angular_z + 1e-9);
  EXPECT_TRUE(r.back().flags & cmd_vel_safety::FLAG_LINEAR_CLAMPED);
  EXPECT_TRUE(r.back().flags & cmd_vel_safety::FLAG_ANGULAR_CLAMPED);
  // v*w must also respect the centripetal budget, which the independent
  // per-axis clamps alone would not guarantee.
  EXPECT_LE(
    std::fabs(r.back().output.v * r.back().output.w), l.max_lateral_accel + 1e-6);
  EXPECT_TRUE(r.back().flags & cmd_vel_safety::FLAG_LATERAL_ACCEL_LIMITED);
}

TEST(SafetyLimiter, AppliesTighterReverseLimit)
{
  Harness h;
  const auto r = h.feed(fwd(-2.0), 6.0);
  EXPECT_NEAR(r.back().output.v, defaultLimits().min_linear_x, 1e-6);
}

// ---------------------------------------------------------------------------
// Slew limiting: the output must never step, whatever the input does.
// ---------------------------------------------------------------------------
TEST(SafetyLimiter, NeverExceedsAccelerationBudget)
{
  Limits l = defaultLimits();
  l.spike_rejection_enabled = false;  // isolate the slew limiter
  Harness h(l);

  double prev = 0.0;
  const double budget = std::max(
    l.max_linear_accel, l.max_linear_decel *
    l.emergency_decel_factor) * kPeriod;

  // Alternate between the extremes every input frame: a worst case no sane
  // upstream would produce, which is exactly why it must be tested.
  for (int i = 0; i < 40; ++i) {
    const auto c = h.feedOnce(fwd((i % 2) ? 1.0 : -0.3));
    EXPECT_LE(std::fabs(c.output.v - prev), budget * (kInputDt / kPeriod) + 1e-9);
    prev = c.output.v;
  }
}

TEST(SafetyLimiter, RampsRatherThanSteppingToTarget)
{
  Limits l = defaultLimits();
  l.spike_rejection_enabled = false;
  Harness h(l);

  const auto first = h.feedOnce(fwd(1.0));
  // 0.8 m/s^2 over 0.1 s cannot reach 1.0 m/s from standstill.
  EXPECT_LT(first.output.v, 0.2);
  EXPECT_TRUE(first.flags & cmd_vel_safety::FLAG_ACCEL_LIMITED);

  const auto later = h.feed(fwd(1.0), 5.0);
  EXPECT_NEAR(later.back().output.v, 1.0, 1e-6);
}

// ---------------------------------------------------------------------------
// Bag frames 240-279: linear.y on a differential-drive base.
// ---------------------------------------------------------------------------
TEST(SafetyLimiter, ProjectsAwayUnexecutableDegreesOfFreedom)
{
  Harness h;
  Twist6 cmd = fwd(0.3);
  cmd.ly = 0.4;

  const auto v = h.limiter().submit(cmd, h.now());
  EXPECT_TRUE(v.accepted);
  EXPECT_TRUE(v.flags & cmd_vel_safety::FLAG_NONHOLONOMIC);
  EXPECT_NEAR(h.limiter().target().v, 0.3, 1e-9);
  EXPECT_NEAR(h.limiter().target().w, 0.0, 1e-9);
}

TEST(SafetyLimiter, HolonomicBasePassesLateralCommandUnflagged)
{
  Limits l = defaultLimits();
  l.enforce_nonholonomic = false;
  Harness h(l);

  Twist6 cmd = fwd(0.3);
  cmd.ly = 0.4;
  const auto v = h.limiter().submit(cmd, h.now());
  EXPECT_FALSE(v.flags & cmd_vel_safety::FLAG_NONHOLONOMIC);
}

// ---------------------------------------------------------------------------
// Bag 27.9 -> 32.0 s: the 4.1 s dropout.
// ---------------------------------------------------------------------------
TEST(SafetyLimiter, BrakesToZeroOnCommandTimeout)
{
  Harness h;
  h.feed(fwd(0.3), 3.0);
  ASSERT_NEAR(h.limiter().output().v, 0.3, 1e-6);

  const auto during = h.silence(4.1);

  EXPECT_TRUE(during.back().watchdog_active);
  EXPECT_TRUE(during.back().flags & cmd_vel_safety::FLAG_WATCHDOG_TIMEOUT);
  EXPECT_NEAR(during.back().output.v, 0.0, 1e-9);
  EXPECT_NEAR(during.back().output.w, 0.0, 1e-9);

  // The brake must be a controlled ramp, not an instantaneous jump to zero:
  // an infinite deceleration means mechanical shock and wheel slip.
  double prev = 0.3;
  int ramp_steps = 0;
  for (const auto & c : during) {
    EXPECT_LE(
      std::fabs(c.output.v - prev),
      defaultLimits().max_linear_decel * defaultLimits().emergency_decel_factor *
      kPeriod + 1e-9);
    if (c.output.v != prev) {++ramp_steps;}
    prev = c.output.v;
  }
  EXPECT_GE(ramp_steps, 2) << "stopping should take more than one cycle";
}

TEST(SafetyLimiter, DoesNotResumeOldSpeedAfterTimeout)
{
  Harness h;
  h.feed(fwd(0.3), 3.0);
  h.silence(4.1);
  ASSERT_NEAR(h.limiter().output().v, 0.0, 1e-9);

  // Recovery, as in the bag: 0.2 m/s arrives after the dropout. The output
  // must ramp up from standstill rather than jump back to 0.3.
  const auto first = h.feedOnce(fwd(0.2));
  EXPECT_FALSE(first.watchdog_active);
  EXPECT_GT(first.output.v, 0.0);
  EXPECT_LT(first.output.v, 0.2) << "must ramp, not jump";

  const auto settled = h.feed(fwd(0.2), 2.0);
  EXPECT_NEAR(settled.back().output.v, 0.2, 1e-6);
}

// ---------------------------------------------------------------------------
// Emergency stop.
// ---------------------------------------------------------------------------
TEST(SafetyLimiter, EstopStopsImmediatelyAndLatches)
{
  Harness h;
  h.feed(fwd(0.5), 3.0);
  ASSERT_GT(h.limiter().output().v, 0.4);

  h.limiter().setEstop(true);
  const auto c = h.limiter().update(h.now() + kPeriod, kPeriod);
  EXPECT_NEAR(c.output.v, 0.0, 1e-9) << "hard stop bypasses the slew limiter";
  EXPECT_TRUE(c.flags & cmd_vel_safety::FLAG_ESTOP);

  // Still latched while commands keep arriving.
  const auto still = h.feed(fwd(0.5), 1.0);
  EXPECT_NEAR(still.back().output.v, 0.0, 1e-9);
  EXPECT_TRUE(still.back().estop_active);
}

TEST(SafetyLimiter, EstopReleaseDoesNotRestoreOldSpeed)
{
  Harness h;
  h.feed(fwd(0.5), 3.0);
  h.limiter().setEstop(true);
  h.feed(fwd(0.5), 1.0);

  h.limiter().setEstop(false);
  const auto c = h.limiter().update(h.now() + kPeriod, kPeriod);
  EXPECT_NEAR(c.output.v, 0.0, 1e-9) << "releasing an e-stop must not resume motion";
}

// ---------------------------------------------------------------------------
// Deadband.
// ---------------------------------------------------------------------------
TEST(SafetyLimiter, SnapsSubDeadbandCommandToZero)
{
  Harness h;
  const auto v = h.limiter().submit(fwd(0.005, 0.01), h.now());
  EXPECT_TRUE(v.flags & cmd_vel_safety::FLAG_DEADBAND_APPLIED);
  EXPECT_NEAR(h.limiter().target().v, 0.0, 1e-12);
  EXPECT_NEAR(h.limiter().target().w, 0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// Robustness of the pipeline itself.
// ---------------------------------------------------------------------------
TEST(SafetyLimiter, SurvivesPathologicalDt)
{
  Harness h;
  h.feed(fwd(0.5), 2.0);
  const double settled = h.limiter().output().v;
  ASSERT_NEAR(settled, 0.5, 1e-6);

  // Zero, negative, huge and NaN dt values must neither produce an unbounded
  // step nor silently degrade a perfectly good output to zero: the cycle falls
  // back to the last plausible period instead.
  for (const double dt : {0.0, -1.0, 1e9, std::numeric_limits<double>::quiet_NaN()}) {
    const auto c = h.limiter().update(h.now(), dt);
    EXPECT_TRUE(std::isfinite(c.output.v)) << "dt=" << dt;
    EXPECT_TRUE(std::isfinite(c.output.w)) << "dt=" << dt;
    EXPECT_LE(c.output.v, defaultLimits().max_linear_x + 1e-9) << "dt=" << dt;
    EXPECT_NEAR(c.output.v, settled, 1e-6) << "dt=" << dt;
  }
}

TEST(SafetyLimiter, ResetClearsMotionState)
{
  Harness h;
  h.feed(fwd(0.8), 3.0);
  ASSERT_GT(h.limiter().output().v, 0.5);

  h.limiter().reset();
  EXPECT_NEAR(h.limiter().output().v, 0.0, 1e-12);
  EXPECT_NEAR(h.limiter().target().v, 0.0, 1e-12);
  EXPECT_FALSE(h.limiter().hasInput());
  // Lifetime counters deliberately survive a reset.
  EXPECT_GT(h.limiter().totalReceived(), 0u);
}

// ---------------------------------------------------------------------------
// The safety invariants from the design document, asserted over the exact
// command sequence contained in the supplied bag.
// ---------------------------------------------------------------------------
TEST(SafetyLimiter, HoldsAllInvariantsOverTheSuppliedBagScenario)
{
  const Limits l = defaultLimits();
  Harness h(l);

  struct Segment
  {
    double v;
    double w;
    double ly;
    double duration;
    bool silent;
  };
  const std::vector<Segment> script = {
    {0.0, 0.0, 0.0, 3.0, false},     // idle
    {0.3, 0.0, 0.0, 5.1, false},     // steady
    {0.8, 0.0, 0.0, 3.9, false},     // ramp end value
    {0.5, 0.6, 0.0, 3.9, false},     // arc
    {-0.35, 0.0, 0.0, 3.0, false},   // reverse
    {2.5, 3.0, 0.0, 3.0, false},     // sustained over-limit
    {0.25, -0.25, 0.0, 2.0, false},  // back to normal
    {0.3, 0.0, 0.4, 3.9, false},     // non-holonomic request
    {0.0, 0.0, 0.0, 4.1, true},      // the dropout
    {0.2, 0.0, 0.0, 3.4, false},     // recovery
    {0.0, 0.0, 0.0, 0.5, false},     // stop
  };

  std::vector<CycleResult> all;
  for (const auto & s : script) {
    if (s.silent) {
      const auto r = h.silence(s.duration);
      all.insert(all.end(), r.begin(), r.end());
    } else {
      Twist6 cmd = fwd(s.v, s.w);
      cmd.ly = s.ly;
      const int frames = static_cast<int>(std::round(s.duration / kInputDt));
      for (int i = 0; i < frames; ++i) {
        all.push_back(h.feedOnce(cmd));
      }
    }
  }

  ASSERT_FALSE(all.empty());
  double prev_v = 0.0;
  double prev_w = 0.0;
  const double v_budget =
    l.max_linear_decel * l.emergency_decel_factor * kInputDt + 1e-6;
  const double w_budget =
    l.max_angular_decel * l.emergency_decel_factor * kInputDt + 1e-6;

  for (size_t i = 0; i < all.size(); ++i) {
    const auto & c = all[i];
    // 1. always finite
    ASSERT_TRUE(std::isfinite(c.output.v)) << "cycle " << i;
    ASSERT_TRUE(std::isfinite(c.output.w)) << "cycle " << i;
    // 2. inside the envelope
    ASSERT_GE(c.output.v, l.min_linear_x - 1e-6) << "cycle " << i;
    ASSERT_LE(c.output.v, l.max_linear_x + 1e-6) << "cycle " << i;
    ASSERT_LE(std::fabs(c.output.w), l.max_angular_z + 1e-6) << "cycle " << i;
    // 3. continuous output
    ASSERT_LE(std::fabs(c.output.v - prev_v), v_budget) << "cycle " << i;
    ASSERT_LE(std::fabs(c.output.w - prev_w), w_budget) << "cycle " << i;
    // 4. centripetal budget
    ASSERT_LE(std::fabs(c.output.v * c.output.w), l.max_lateral_accel + 1e-3)
      << "cycle " << i;
    prev_v = c.output.v;
    prev_w = c.output.w;
  }

  // 5. the dropout really did bring the robot to a halt
  EXPECT_NEAR(all.back().output.v, 0.0, 1e-6);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

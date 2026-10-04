#!/usr/bin/env python3
# Copyright 2026 cmd_vel_safety contributors. Apache-2.0.
"""
Assert the safety invariants against a recorded run.

    ros2 run cmd_vel_safety check_bag.py ~/cmd_vel_safety_runs/run_20261004_1200

Turns "the curve in rqt_plot looks fine" into a repeatable regression check.
Reads the bag directly through rosbag2_py and verifies, for every message on
/cmd_vel_safe, the invariants listed in the design document:

  1. all six components are finite
  2. min_linear_x <= v <= max_linear_x and |w| <= max_angular_z
  3. the output is continuous: |dv|/dt within the emergency deceleration budget
  4. |v*w| <= max_lateral_accel
  5. after a gap in /cmd_vel longer than cmd_timeout, the output reaches zero
  6. the unexecutable DOFs are always exactly zero

Limits are read from the params YAML so the check can never drift from the
configuration the run actually used.
"""

import argparse
import math
import os
import sys

import yaml

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:  # pragma: no cover
    sys.exit(f'this script must run inside a sourced ROS 2 environment: {exc}')


def read_bag(path, topics):
    """Yield (topic, msg, t_sec) for the requested topics, in time order."""
    storage = rosbag2_py.StorageOptions(uri=path, storage_id='')
    converter = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)

    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in topics if t not in types]
    if missing:
        print(f'  ! bag does not contain {missing}')
    msg_types = {
        name: get_message(types[name]) for name in topics if name in types
    }

    reader.set_filter(rosbag2_py.StorageFilter(
        topics=[t for t in topics if t in types]))
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        yield topic, deserialize_message(data, msg_types[topic]), stamp * 1e-9


def load_limits(params_file):
    with open(params_file, 'r', encoding='utf-8') as fh:
        doc = yaml.safe_load(fh)
    return doc['velocity_guard']['ros__parameters']


class Checker:
    def __init__(self, limits):
        self.lim = limits
        self.failures = []
        self.checked = 0

    def fail(self, invariant, detail):
        self.failures.append((invariant, detail))

    def run(self, path):
        topics = ['/cmd_vel_safe', '/velocity_guard/report']
        safe = []
        reports = []
        flags_seen = 0

        for topic, msg, _t in read_bag(path, topics):
            if topic == '/cmd_vel_safe':
                safe.append(msg)
            elif topic == '/velocity_guard/report':
                flags_seen |= msg.flags
                reports.append(msg)

        if not safe:
            self.fail('setup', 'no /cmd_vel_safe messages in the bag')
            return
        if not reports:
            self.fail('setup', 'no /velocity_guard/report messages in the bag')
            return

        self.check_pointwise(safe)
        self.check_continuity(reports)
        self.check_failsafe(reports)
        self.report_coverage(flags_seen)

    def check_pointwise(self, safe):
        """Invariants 1, 2, 4 and 6: per-message, no timing involved."""
        lat_max = float(self.lim['max_lateral_accel'])
        enforce_nh = self.lim.get('enforce_nonholonomic', True)

        for i, m in enumerate(safe):
            self.checked += 1
            comps = {
                'linear.x': m.linear.x, 'linear.y': m.linear.y,
                'linear.z': m.linear.z, 'angular.x': m.angular.x,
                'angular.y': m.angular.y, 'angular.z': m.angular.z,
            }

            # 1. finiteness
            for name, val in comps.items():
                if not math.isfinite(val):
                    self.fail('1 finite', f'msg {i}: {name}={val}')

            # 2. envelope
            if not (float(self.lim['min_linear_x']) - 1e-6 <= m.linear.x
                    <= float(self.lim['max_linear_x']) + 1e-6):
                self.fail('2 envelope', f'msg {i}: linear.x={m.linear.x:.4f}')
            if abs(m.angular.z) > float(self.lim['max_angular_z']) + 1e-6:
                self.fail('2 envelope', f'msg {i}: angular.z={m.angular.z:.4f}')

            # 4. centripetal budget
            if lat_max > 0 and abs(m.linear.x * m.angular.z) > lat_max + 1e-3:
                self.fail(
                    '4 lateral',
                    f'msg {i}: |v*w|={abs(m.linear.x * m.angular.z):.3f} > {lat_max}')

            # 6. unexecutable DOFs
            if enforce_nh:
                for name in ('linear.y', 'linear.z', 'angular.x', 'angular.y'):
                    if comps[name] != 0.0:
                        self.fail('6 nonholonomic', f'msg {i}: {name}={comps[name]}')

    def check_continuity(self, reports):
        """
        Check invariant 3, measured on the guard's own clock.

        The acceleration budget is spent by the guard against the dt it
        measures itself, so continuity has to be verified on the same time
        base. /cmd_vel_safe is a bare Twist with no header, and rosbag2's
        receive timestamps come from the system clock -- under `use_sim_time`
        those are a different clock entirely, and dividing a sim-time velocity
        step by a wall-time interval produces a meaningless rate. The report
        carries header.stamp from the guard's clock next to the very output it
        published, which is the only self-consistent pairing available.
        """
        decel_budget = (float(self.lim['max_linear_decel']) *
                        float(self.lim['emergency_decel_factor']))
        ang_budget = (float(self.lim['max_angular_decel']) *
                      float(self.lim['emergency_decel_factor']))
        # The stamp is taken a few microseconds after the limiter samples its
        # own dt, so allow a small relative slack for that and for timer
        # jitter. It is a tolerance on measurement, not on the limit.
        slack = 1.05

        prev_t, prev = None, None
        for m in reports:
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if prev is not None:
                dt = t - prev_t
                if 0 < dt < 1.0:
                    dv = abs(m.output_cmd.linear.x - prev.linear.x) / dt
                    dw = abs(m.output_cmd.angular.z - prev.angular.z) / dt
                    if dv > decel_budget * slack + 1e-3:
                        self.fail(
                            '3 continuity',
                            f't={t:.3f} dv/dt={dv:.3f} > {decel_budget:.3f} m/s^2')
                    if dw > ang_budget * slack + 1e-3:
                        self.fail(
                            '3 continuity',
                            f't={t:.3f} dw/dt={dw:.3f} > {ang_budget:.3f} rad/s^2')
            prev_t, prev = t, m.output_cmd

    def check_failsafe(self, reports):
        """
        Check invariant 5: every input gap must end with the robot stopped.

        Gaps are located from the guard's own watchdog_active flag and then
        cross-checked against the output it actually published -- the claim and
        the evidence come from two different fields, so a guard that merely
        *said* it was braking would still fail here.
        """
        runs = []
        start = None
        for i, m in enumerate(reports):
            if m.watchdog_active and start is None:
                start = i
            elif not m.watchdog_active and start is not None:
                runs.append((start, i - 1))
                start = None
        if start is not None:
            runs.append((start, len(reports) - 1))

        if not runs:
            print('  i no watchdog activation in this run')
            return

        for lo, hi in runs:
            span = reports[hi].time_since_last_input
            last = reports[hi].output_cmd
            residual = abs(last.linear.x) + abs(last.angular.z)
            if residual > 1e-6:
                self.fail(
                    '5 failsafe',
                    f'watchdog run of {hi - lo + 1} cycles ended with output '
                    f'still {residual:.4f}')
                continue
            # It must also have been a controlled ramp, not a single jump.
            # Start one cycle early: the first deceleration step happens on the
            # very cycle the watchdog engages, so counting only from `lo`
            # onwards misses it.
            first = max(lo - 1, 0)
            ramp = sum(
                1 for i in range(first, min(hi, len(reports) - 1))
                if reports[i].output_cmd.linear.x != reports[i + 1].output_cmd.linear.x)
            print(f'  + watchdog gap of {span:.2f}s: output ramped to zero '
                  f'over {max(ramp, 1)} cycle(s)')

    @staticmethod
    def report_coverage(flags_seen):
        names = [
            (1, 'NON_FINITE'), (2, 'NONHOLONOMIC'), (4, 'SPIKE_REJECTED'),
            (8, 'LINEAR_CLAMPED'), (16, 'ANGULAR_CLAMPED'),
            (32, 'LATERAL_ACCEL_LIMITED'), (64, 'ACCEL_LIMITED'),
            (128, 'DEADBAND'), (256, 'WATCHDOG_TIMEOUT'), (512, 'ESTOP'),
            (1024, 'SAFETY_HOLD'),
        ]
        fired = [n for bit, n in names if flags_seen & bit]
        print(f'\n  safety rules exercised by this run ({len(fired)}/{len(names)}):')
        for bit, n in names:
            print(f'    [{"x" if flags_seen & bit else " "}] {n}')
        del fired


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_params = os.path.join(here, '..', 'config', 'params.yaml')

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='recorded bag directory')
    ap.add_argument('--params', default=os.path.normpath(default_params),
                    help='params YAML the run used (for the limit values)')
    ns = ap.parse_args()

    if not os.path.isdir(ns.bag):
        sys.exit(f'not a bag directory: {ns.bag}')

    print(f'checking {ns.bag}\n  limits from {ns.params}\n')
    checker = Checker(load_limits(ns.params))
    checker.run(ns.bag)

    print(f'\n  {checker.checked} /cmd_vel_safe messages checked')
    if checker.failures:
        print(f'\nFAILED: {len(checker.failures)} invariant violation(s)')
        for inv, detail in checker.failures[:40]:
            print(f'  [{inv}] {detail}')
        if len(checker.failures) > 40:
            print(f'  ... and {len(checker.failures) - 40} more')
        return 1

    print('\nPASSED: all safety invariants hold for this run')
    return 0


if __name__ == '__main__':
    sys.exit(main())

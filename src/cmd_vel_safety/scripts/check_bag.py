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
  3. acceleration and braking obey their respective budgets (except hard e-stop)
  4. |v*w| <= max_lateral_accel
  5. after a gap in /cmd_vel longer than cmd_timeout, the output reaches zero
  6. the unexecutable DOFs are always exactly zero

Pass the params YAML used during the recording; runs with dynamic parameter
changes require separate segments with the corresponding configuration.
"""

import argparse
from bisect import bisect_right
import math
import os
import sys

import yaml

try:
    from ament_index_python.packages import get_package_share_directory
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
    def __init__(self, limits, strict=False):
        self.lim = limits
        self.strict = strict
        self.unchecked = []
        self.failures = []
        self.checked = 0

    def fail(self, invariant, detail):
        self.failures.append((invariant, detail))

    def run(self, path, reference_bag=None):
        topics = ['/cmd_vel', '/cmd_vel_safe', '/velocity_guard/report']
        safe, reports, raw, report_receipts = [], [], [], []
        flags_seen = 0
        for topic, msg, stamp in read_bag(path, topics):
            if topic == '/cmd_vel_safe':
                safe.append(msg)
            elif topic == '/cmd_vel':
                raw.append((stamp, msg))
            else:
                reports.append(msg)
                report_receipts.append(stamp)
                flags_seen |= msg.flags
        if not safe or not reports:
            self.fail('setup', 'bag requires /cmd_vel_safe and /velocity_guard/report')
            return
        self.check_pointwise(safe)
        self.check_continuity(reports)
        self.check_pairs(safe, reports)
        if self.strict:
            self.check_report_sequence(reports)
        aligned = all(abs(t - self.stamp(m)) <= 0.1
                      for t, m in zip(report_receipts, reports))
        if aligned and raw:
            self.check_input_gaps(raw, reports)
        else:
            detail = 'independent input-gap audit unavailable: missing raw input or mixed clocks'
            if self.strict:
                self.fail('timing', detail + '; record with --use-sim-time')
            else:
                self.unchecked.append(detail)
                self.check_failsafe(reports)
        if reference_bag:
            original = [m for _, m, _ in read_bag(reference_bag, ['/cmd_vel'])]
            if not original:
                self.fail('reference', 'reference bag has no /cmd_vel messages')
            self.check_reference([m for _, m in raw], original)
        else:
            self.unchecked.append('source-input completeness (no --reference-bag supplied)')
        self.report_coverage(flags_seen)

    @staticmethod
    def stamp(message):
        return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    @staticmethod
    def components(message):
        return (message.linear.x, message.linear.y, message.linear.z,
                message.angular.x, message.angular.y, message.angular.z)

    @classmethod
    def same_twist(cls, left, right):
        return all(a == b or (math.isnan(a) and math.isnan(b))
                   for a, b in zip(cls.components(left), cls.components(right)))

    def check_pairs(self, safe, reports):
        if len(safe) != len(reports):
            self.fail('pairing', f'{len(safe)} outputs versus {len(reports)} reports')
        for i, (out, rep) in enumerate(zip(safe, reports)):
            if not self.same_twist(out, rep.output_cmd):
                self.fail('pairing', f'output/report mismatch at index {i}')

    def check_reference(self, recorded, original):
        if len(recorded) != len(original):
            self.fail('reference',
                      f'{len(recorded)} input frames recorded; expected {len(original)}')
        for i, (left, right) in enumerate(zip(recorded, original)):
            if not self.same_twist(left, right):
                self.fail('reference', f'input content/order differs at frame {i}')
                break

    def check_report_sequence(self, reports):
        max_interval = max(1.0, 2.0 / float(self.lim['control_rate_hz']))
        for prev, curr in zip(reports, reports[1:]):
            if curr.total_published != prev.total_published + 1:
                self.fail('sequence', 'missing, duplicated or reordered control report')
            dt = self.stamp(curr) - self.stamp(prev)
            if dt == 0 or dt > max_interval + 1e-6:
                self.fail('sequence', f'unverifiable control interval {dt:.6f}s')

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
            if lat_max > 0 and abs(m.linear.x * m.angular.z) > lat_max + 1e-6:
                self.fail(
                    '4 lateral',
                    f'msg {i}: |v*w|={abs(m.linear.x * m.angular.z):.3f} > {lat_max}')

            # 6. unexecutable DOFs
            if enforce_nh:
                for name in ('linear.y', 'linear.z', 'angular.x', 'angular.y'):
                    if comps[name] != 0.0:
                        self.fail('6 nonholonomic', f'msg {i}: {name}={comps[name]}')

    def check_continuity(self, reports):
        previous = None
        for message in reports:
            out = message.output_cmd
            if not all(math.isfinite(x) for x in self.components(out)):
                self.fail('3 continuity', 'non-finite output in report')
                previous = None
                continue
            hard_stop = message.estop_active and self.lim.get('estop_hard_stop', True)
            if hard_stop:
                if any(x != 0.0 for x in self.components(out)):
                    self.fail('estop', 'hard emergency stop must output zero')
            elif previous is not None:
                dt = self.stamp(message) - self.stamp(previous)
                if dt > 0.0:
                    # The limiter caps a delayed cycle's integration budget at one second.
                    dt = min(dt, 1.0)
                    emergency = (message.watchdog_active or message.estop_active or
                                 bool(message.flags & message.FLAG_SAFETY_HOLD))
                    factor = float(self.lim['emergency_decel_factor']) if emergency else 1.0
                    for axis, old, new in (
                        ('linear', previous.output_cmd.linear.x, out.linear.x),
                        ('angular', previous.output_cmd.angular.z, out.angular.z),
                    ):
                        accel = float(self.lim[f'max_{axis}_accel'])
                        decel = float(self.lim[f'max_{axis}_decel']) * factor
                        if old * new < 0.0:
                            required = abs(old) / decel + abs(new) / accel
                        elif abs(new) > abs(old):
                            required = (abs(new) - abs(old)) / accel
                        else:
                            required = (abs(old) - abs(new)) / decel
                        if required > dt + 1e-6:
                            self.fail('3 continuity', f'{axis} change requires {required:.6f}s, '
                                      f'available {dt:.6f}s')
            previous = message

    def check_failsafe(self, reports):
        """Fallback for historical mixed-clock recordings, not independent evidence."""
        start = None
        for i, message in enumerate(reports):
            if not message.watchdog_active:
                start = None
                continue
            if start is None:
                start = max(0, i - 1)
            initial = reports[start].output_cmd
            factor = float(self.lim['emergency_decel_factor'])
            stopping_time = max(
                abs(initial.linear.x) / (float(self.lim['max_linear_decel']) * factor),
                abs(initial.angular.z) / (float(self.lim['max_angular_decel']) * factor))
            elapsed = self.stamp(message) - self.stamp(reports[start])
            if elapsed >= stopping_time + 0.1:
                if abs(message.output_cmd.linear.x) + abs(message.output_cmd.angular.z) > 1e-6:
                    self.fail('5 failsafe',
                              'sustained watchdog did not stop within braking budget')

    def check_input_gaps(self, raw, reports):
        """Check stale input independently of the guard's watchdog declaration."""
        stamps = [t for t, _ in raw]
        if any(b < a for a, b in zip(stamps, stamps[1:])) or any(
                self.stamp(b) < self.stamp(a) for a, b in zip(reports, reports[1:])):
            # Pair segments by acquisition order only when segment boundaries
            # are unambiguous in both streams; otherwise fail the strict audit.
            detail = 'clock reset: independent gap audit requires separate per-loop recordings'
            if self.strict:
                self.fail('timing', detail)
            else:
                self.unchecked.append(detail)
            return
        timeout = float(self.lim['cmd_timeout'])
        slack = max(0.02, 1.0 / float(self.lim['control_rate_hz']))
        coverage_slack = 1.0 / float(self.lim['control_rate_hz']) + 0.02
        if (self.stamp(reports[0]) > stamps[0] + coverage_slack or
                self.stamp(reports[-1]) < stamps[-1] - coverage_slack):
            detail = 'control reports do not cover the recorded raw input period'
            if self.strict:
                self.fail('timing', detail)
            else:
                self.unchecked.append(detail)
        factor = float(self.lim['emergency_decel_factor'])
        stale_index, stop_deadline = None, None
        for message in reports:
            t = self.stamp(message)
            idx = bisect_right(stamps, t) - 1
            if idx < 0:
                continue
            if idx != stale_index:
                stale_index, stop_deadline = idx, None
            age = t - stamps[idx]
            if age > timeout and stop_deadline is None:
                stop_budget = max(
                    abs(message.output_cmd.linear.x) /
                    (float(self.lim['max_linear_decel']) * factor),
                    abs(message.output_cmd.angular.z) /
                    (float(self.lim['max_angular_decel']) * factor))
                # Anchor to measured motion at the first stale control sample,
                # with bounded recording/control delay, independent of watchdog flags.
                stop_deadline = t + stop_budget + 2 * slack
            if age > timeout + slack:
                if not message.watchdog_active:
                    self.fail('5 failsafe', f'input stale for {age:.3f}s without watchdog')
                if stop_deadline is not None and t > stop_deadline:
                    if abs(message.output_cmd.linear.x) + abs(message.output_cmd.angular.z) > 1e-6:
                        self.fail('5 failsafe', 'output still moving beyond stopping deadline')
            elif age < timeout - slack and message.watchdog_active:
                self.fail('5 failsafe', 'watchdog asserted while raw input is fresh')

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
    default_params = os.path.join(
        get_package_share_directory('cmd_vel_safety'), 'config', 'params.yaml')

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='recorded bag directory')
    ap.add_argument('--params', default=os.path.normpath(default_params),
                    help='params YAML the run used (for the limit values)')
    ap.add_argument('--strict', action='store_true',
                    help='require complete, clock-aligned audit data')
    ap.add_argument('--reference-bag',
                    help='source bag used to check input completeness and order')
    ns = ap.parse_args()

    if not os.path.isdir(ns.bag):
        sys.exit(f'not a bag directory: {ns.bag}')

    print(f'checking {ns.bag}\n  limits from {ns.params}\n')
    checker = Checker(load_limits(ns.params), strict=ns.strict)
    checker.run(ns.bag, reference_bag=ns.reference_bag)
    for item in checker.unchecked:
        print(f"  UNCHECKED: {item}")

    print(f'\n  {checker.checked} /cmd_vel_safe messages checked')
    if checker.failures:
        print(f'\nFAILED: {len(checker.failures)} invariant violation(s)')
        for inv, detail in checker.failures[:40]:
            print(f'  [{inv}] {detail}')
        if len(checker.failures) > 40:
            print(f'  ... and {len(checker.failures) - 40} more')
        return 1

    print('\nPASSED: performed checks passed (see UNCHECKED items and coverage above)')
    return 0


if __name__ == '__main__':
    sys.exit(main())

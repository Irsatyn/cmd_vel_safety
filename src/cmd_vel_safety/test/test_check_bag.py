# Copyright 2026 cmd_vel_safety contributors. Apache-2.0.
"""Regression tests for independently auditing recorded output."""
import importlib.util
import math
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from cmd_vel_safety_msgs.msg import SafetyReport
from geometry_msgs.msg import Twist

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('check_bag', ROOT / 'scripts/check_bag.py')
checker_module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(checker_module)


def report(t, v, w=0.0, watchdog=False, estop=False, count=1):
    m = SafetyReport()
    ns = round(t * 1e9)
    m.header.stamp.sec, m.header.stamp.nanosec = divmod(ns, 1000000000)
    m.output_cmd.linear.x = v
    m.output_cmd.angular.z = w
    m.watchdog_active = watchdog
    m.estop_active = estop
    m.total_published = count
    if watchdog:
        m.flags |= m.FLAG_WATCHDOG_TIMEOUT
    if estop:
        m.flags |= m.FLAG_ESTOP
    return m


class TestChecker(unittest.TestCase):
    def setUp(self):
        self.limits = checker_module.load_limits(ROOT / 'config/params.yaml')
        self.checker = checker_module.Checker(self.limits)

    def test_small_lateral_violation_is_not_hidden_by_slack(self):
        out = Twist()
        out.linear.x = 1.0
        out.angular.z = 1.2005
        self.checker.check_pointwise([out])
        self.assertTrue(self.checker.failures)

    def test_normal_acceleration_violation(self):
        self.checker.check_continuity([report(10, 0.0), report(10.05, 0.1)])
        self.assertTrue(self.checker.failures)

    def test_hard_estop_is_allowed(self):
        self.checker.check_continuity([report(10, 1.0), report(10.05, 0.0, estop=True)])
        self.assertFalse(self.checker.failures)

    def test_hard_estop_requires_zero(self):
        self.checker.check_continuity([report(10, 0.1), report(10.05, 0.1, estop=True)])
        self.assertTrue(self.checker.failures)

    def test_reverse_uses_two_budgets(self):
        self.checker.check_continuity([report(10, 0.01), report(10.05, -0.065)])
        self.assertTrue(self.checker.failures)

    def test_emergency_deceleration(self):
        self.checker.check_continuity([report(10, 0.3), report(10.05, 0.15, watchdog=True)])
        self.assertFalse(self.checker.failures)

    def test_nonfinite_report_is_rejected(self):
        self.checker.check_continuity([report(10, 0.0), report(10.05, math.nan)])
        self.assertTrue(self.checker.failures)

    def test_short_watchdog_does_not_require_instant_stop(self):
        m = report(10, 0.15, watchdog=True)
        m.time_since_last_input = 0.55
        self.checker.check_failsafe([m, report(10.05, 0.15)])
        self.assertFalse(self.checker.failures)

    def test_missing_watchdog_is_detected_independently(self):
        raw = [(10.0, Twist())]
        reports = [report(10.0 + i * .05, .2, count=i+1) for i in range(25)]
        self.checker.check_input_gaps(raw, reports)
        self.assertTrue(self.checker.failures)

    def test_short_input_gap_allows_braking_then_recovery(self):
        raw = [(10.0, Twist()), (10.56, Twist())]
        reports = [report(10.5, .6), report(10.55, .45, watchdog=True), report(10.6, .45)]
        self.checker.check_input_gaps(raw, reports)
        self.assertFalse(self.checker.failures)

    def test_stopping_deadline_uses_actual_motion(self):
        raw = [(10.0, Twist()), (10.9, Twist())]
        reports = [report(10 + i * .05, .2, watchdog=11 <= i <= 17, count=i+1)
                   for i in range(19)]
        self.checker.check_input_gaps(raw, reports)
        self.assertTrue(self.checker.failures)

    def test_strict_requires_reports_cover_input_period(self):
        checker = checker_module.Checker(self.limits, strict=True)
        first, last = Twist(), Twist()
        messages = [('/cmd_vel', first, 10.0),
                    ('/cmd_vel_safe', Twist(), 10.0),
                    ('/velocity_guard/report', report(10.0, 0., count=1), 10.0),
                    ('/cmd_vel_safe', Twist(), 10.05),
                    ('/velocity_guard/report', report(10.05, 0., count=2), 10.05),
                    ('/cmd_vel', last, 12.0)]
        reference = [('/cmd_vel', first, 10.0), ('/cmd_vel', last, 12.0)]
        with mock.patch.object(checker_module, 'read_bag',
                               side_effect=[iter(messages), iter(reference)]):
            checker.run('recording', reference_bag='source')
        self.assertTrue(checker.failures)

    def test_one_hz_control_interval_is_valid(self):
        checker = checker_module.Checker(dict(self.limits, control_rate_hz=1., cmd_timeout=2.),
                                         strict=True)
        checker.check_report_sequence([report(10, 0., count=1), report(11.01, 0., count=2)])
        self.assertFalse(checker.failures)

    def test_output_report_mismatch(self):
        self.checker.check_pairs([Twist()], [report(10, .2)])
        self.assertTrue(self.checker.failures)

    def test_reference_detects_lost_frames(self):
        self.checker.check_reference([Twist()] * 310, [Twist()] * 320)
        self.assertTrue(self.checker.failures)

    def test_reference_compares_order_and_nan(self):
        a, b = Twist(), Twist()
        a.linear.x = math.nan
        b.linear.x = math.nan
        self.checker.check_reference([a], [b])
        self.assertFalse(self.checker.failures)
        b.angular.z = 1.0
        self.checker.check_reference([a], [b])
        self.assertTrue(self.checker.failures)

    def test_strict_detects_missing_cycle(self):
        reports = [report(10, 0., count=1), report(10.1, 0., count=3)]
        self.checker.check_report_sequence(reports)
        self.assertTrue(self.checker.failures)

    def test_backward_clock_splits_continuity(self):
        self.checker.check_continuity([report(10, .4), report(1, 0.), report(1.05, .04)])
        self.assertFalse(self.checker.failures)

    def test_record_path_normalization_and_collision(self):
        spec = importlib.util.spec_from_file_location(
            'record_launch', ROOT / 'launch/record_bag.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertTrue(Path(module.prepare_output_path('new_relative_run')).is_absolute())
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(FileExistsError):
                module.prepare_output_path(directory)


if __name__ == '__main__':
    unittest.main()

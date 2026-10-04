# Copyright 2026 cmd_vel_safety contributors. Apache-2.0.
"""Live-node contracts; run through launch_testing or as a unittest script."""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import launch
import launch_testing
import launch_testing.util
import rclpy
from ament_index_python.packages import get_package_prefix
from cmd_vel_safety_msgs.msg import MotionState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.parameter import Parameter


# Each invocation owns its DDS domain and log directory.
os.environ['ROS_DOMAIN_ID'] = os.environ.get('CMD_VEL_TEST_DOMAIN_ID', '173')
os.environ.setdefault('ROS_LOG_DIR', tempfile.mkdtemp(prefix='safety-node-logs-'))


def generate_test_description():
    return launch.LaunchDescription([
        launch_testing.util.KeepAliveProc(), launch_testing.actions.ReadyToTest()])


class TestNodeContracts(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('contract_probe')
        self.processes = []
        self.logs = []
        self.states = []
        self.node.create_subscription(MotionState, '/robot_motion_state',
                                      self.states.append, 100)

    def tearDown(self):
        for proc in self.processes:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=3)
        self.node.destroy_node()
        rclpy.shutdown()
        for log in self.logs:
            log.close()

    def start(self, *args):
        log = tempfile.TemporaryFile(mode='w+')
        proc = subprocess.Popen(args, stdout=log, stderr=log, start_new_session=True)
        self.processes.append(proc)
        self.logs.append(log)
        return proc, log

    def node_process(self, executable, *args):
        path = Path(get_package_prefix('cmd_vel_safety')) / 'lib/cmd_vel_safety'
        return self.start(str(path / executable), '--ros-args', *args)

    def spin(self, seconds, publish=None):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            if publish:
                publish()
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait(self, predicate, seconds=6, publish=None):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            if predicate():
                return
            self.spin(0.05, publish)
        self.assertTrue(predicate(), 'condition not reached before deadline')

    def set_params(self, name, values):
        cli = self.node.create_client(SetParametersAtomically,
                                      f'/{name}/set_parameters_atomically')
        self.assertTrue(cli.wait_for_service(timeout_sec=5))
        req = SetParametersAtomically.Request()
        req.parameters = [Parameter(k, value=v).to_parameter_msg()
                          for k, v in values.items()]
        future = cli.call_async(req)
        self.wait(future.done)
        result = future.result().result
        self.node.destroy_client(cli)
        return result

    def get_double(self, name, key):
        cli = self.node.create_client(GetParameters, f'/{name}/get_parameters')
        self.assertTrue(cli.wait_for_service(timeout_sec=5))
        request = GetParameters.Request(names=[key])
        future = cli.call_async(request)
        self.wait(future.done)
        result = future.result().values[0].double_value
        self.node.destroy_client(cli)
        return result

    def test_startup_rejects_invalid_combination_and_qos(self):
        for args, reason in [
            (['-p', 'cmd_timeout:=0.01'], 'two control cycles'),
            (['-p', 'estop_qos_durability:=typo'], 'estop_qos_durability'),
        ]:
            proc, log = self.node_process('velocity_guard', *args)
            self.wait(lambda: proc.poll() is not None, seconds=3)
            self.assertNotEqual(proc.returncode, 0)
            log.seek(0)
            self.assertIn(reason, log.read())

    def test_dynamic_limit_rejection_preserves_configuration(self):
        self.node_process('velocity_guard')
        pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        outputs = []
        self.node.create_subscription(Twist, '/cmd_vel_safe', outputs.append, 100)
        command = Twist()
        command.linear.x = 1.0
        self.wait(lambda: outputs and outputs[-1].linear.x > 0.9,
                  publish=lambda: pub.publish(command))
        result = self.set_params('velocity_guard', {'max_linear_x': 0.4})
        self.assertFalse(result.successful)
        self.assertEqual(self.get_double('velocity_guard', 'max_linear_x'), 1.0)
        self.assertFalse(self.set_params('velocity_guard', {'cmd_timeout': 0.01}).successful)
        command.linear.x = 0.0
        self.wait(lambda: outputs[-1].linear.x == 0.0, publish=lambda: pub.publish(command))
        self.assertTrue(self.set_params('velocity_guard', {'max_linear_x': 0.4}).successful)

    def check_topics(self, raw_topic, safe_topic):
        self.start('ros2', 'launch', 'cmd_vel_safety', 'bringup.launch.py',
                   f'cmd_vel_topic:={raw_topic}', f'cmd_vel_safe_topic:={safe_topic}')
        pub = self.node.create_publisher(Twist, raw_topic, 10)
        outputs = []
        self.node.create_subscription(Twist, safe_topic, outputs.append, 100)
        command = Twist()
        command.linear.x = 0.2
        self.wait(lambda: self.states and self.states[-1].linear_speed > 0.1 and
                  self.states[-1].cmd_input_rate_hz > 0 and outputs,
                  publish=lambda: pub.publish(command))

    def test_default_topics(self):
        self.check_topics('/cmd_vel', '/cmd_vel_safe')

    def test_remapped_topics(self):
        self.check_topics('/test/raw', '/test/safe')

    def test_monitor_rejects_invalid_feedback_and_recovers(self):
        self.node_process('motion_state_monitor')
        pub = self.node.create_publisher(Odometry, '/odom', 10)
        odom = Odometry()
        odom.pose.pose.orientation.w = 2.0  # finite, normalizable
        odom.twist.twist.linear.x = 0.2
        self.wait(lambda: self.states and self.states[-1].distance_travelled > 0.02,
                  publish=lambda: pub.publish(odom))
        distance = self.states[-1].distance_travelled
        for bad in (float('nan'), float('inf')):
            odom.twist.twist.linear.x = bad
            self.spin(0.3, lambda: pub.publish(odom))
            self.assertTrue(math.isfinite(self.states[-1].distance_travelled))
        odom.twist.twist.linear.x = 0.2
        odom.pose.pose.orientation.w = 0.0
        self.spin(1.2, lambda: pub.publish(odom))
        self.assertEqual(self.states[-1].velocity_source, MotionState.SOURCE_UNKNOWN)
        odom.pose.pose.orientation.w = float('nan')
        self.spin(0.3, lambda: pub.publish(odom))
        self.assertTrue(math.isfinite(self.states[-1].heading))
        odom.pose.pose.orientation.w = 1.0
        self.wait(lambda: self.states[-1].distance_travelled > distance + 0.03,
                  publish=lambda: pub.publish(odom))

    def test_monitor_sources_and_dynamic_rate(self):
        diagnostics = []
        self.node.create_subscription(DiagnosticArray, '/diagnostics', diagnostics.append, 100)
        self.node_process('motion_state_monitor')
        self.wait(lambda: bool(self.states) and bool(diagnostics))
        self.assertEqual(self.states[-1].velocity_source, MotionState.SOURCE_UNKNOWN)
        self.assertEqual(self.states[-1].state, MotionState.STATE_UNKNOWN)
        motion = next(s for s in diagnostics[-1].status if s.name.endswith(': motion'))
        self.assertEqual(motion.level, DiagnosticStatus.STALE)
        safe = self.node.create_publisher(Twist, '/cmd_vel_safe', 10)
        command = Twist()
        command.linear.x = 0.2
        self.wait(lambda: self.states[-1].velocity_source == MotionState.SOURCE_COMMAND,
                  publish=lambda: safe.publish(command))
        odom_pub = self.node.create_publisher(Odometry, '/odom', 10)
        odom = Odometry()
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = 0.1
        self.wait(lambda: self.states[-1].velocity_source == MotionState.SOURCE_ODOMETRY,
                  publish=lambda: odom_pub.publish(odom))
        self.assertFalse(self.set_params('motion_state_monitor', {'rate_window': 0.0}).successful)
        result = self.set_params('motion_state_monitor', {'publish_rate_hz': 0.0})
        self.assertFalse(result.successful)
        result = self.set_params('motion_state_monitor', {'publish_rate_hz': 10.0})
        self.assertTrue(result.successful)
        self.spin(0.4)
        before = len(self.states)
        self.spin(1.2)
        self.assertGreaterEqual(len(self.states) - before, 9)
        self.assertLessEqual(len(self.states) - before, 15)
        self.assertEqual(self.states[-1].velocity_source, MotionState.SOURCE_UNKNOWN)
        self.assertEqual(self.states[-1].state, MotionState.STATE_UNKNOWN)


if __name__ == '__main__':
    unittest.main()

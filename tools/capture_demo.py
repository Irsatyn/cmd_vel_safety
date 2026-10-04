#!/usr/bin/env python3
"""Capture a real ROS 2 Humble demonstration window and its runtime evidence."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

import cv2
import numpy as np
from PyQt5.QtCore import QObject, QTimer
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen
from PyQt5.QtWidgets import QApplication, QLabel, QPlainTextEdit, QVBoxLayout, QWidget
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from cmd_vel_safety_msgs.msg import MotionState, SafetyReport
from rqt_graph.ros_graph import RosGraph


# End times in wall seconds, stage, command (v, w, y), e-stop state.
PHASES = [
    (4, 'Startup / zero command', (0., 0., 0.), False),
    (10, 'Normal motion', (.3, 0., 0.), False),
    (16, 'Over-limit command + unexecutable DOF', (1.8, 2., .2), False),
    (21, 'Reverse with controlled cross-zero transition', (-.2, -.8, 0.), False),
    (26, 'Input dropout: independent watchdog braking', None, False),
    (30, 'Resume with fresh command', (.2, 0., 0.), False),
    (34, 'NaN / Inf: persistent-invalid safety hold', (math.nan, 0., 0.), False),
    (38, 'Valid-input recovery', (.3, .5, 0.), False),
    (42, 'Emergency stop (Reliable + TransientLocal)', (.5, .5, 0.), True),
    (45, 'Release e-stop; withhold fresh command', None, False),
    (50, 'Fresh command after e-stop release', (.2, -.6, 0.), False),
    (54, 'Deadband command / final stop', (.005, .01, 0.), False),
    (58, 'Shutdown / verified results', None, False),
]
TOPICS = ['/cmd_vel', '/cmd_vel_safe', '/e_stop', '/velocity_guard/report',
          '/odom', '/robot_motion_state', '/robot_status', '/diagnostics']


def json_number(value):
    return value if math.isfinite(value) else str(value)


class GraphContext(QObject):
    """Minimal embedding context for the installed Humble rqt_graph plugin."""
    def __init__(self, node):
        super().__init__()
        self.node = node

    def serial_number(self):
        return 1

    def add_widget(self, widget):
        self.widget = widget


class Plot(QWidget):
    def __init__(self):
        super().__init__()
        self.points = []
        self.setMinimumHeight(185)

    def paintEvent(self, event):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor('#f8fafc'))
        p.setFont(QFont('DejaVu Sans', 10))
        p.setPen(QColor('#24354b'))
        p.drawText(12, 19, 'Linear velocity (m/s): raw [blue], target [orange], output [green]')
        left, top, width, height = 65, 32, self.width()-90, self.height()-62
        for value in (-.3, 0., 1., 1.8):
            y = top + height * (2.-value)/2.5
            p.setPen(QPen(QColor('#cbd5e1'), 1))
            p.drawLine(left, int(y), left+width, int(y))
            p.setPen(QColor('#24354b'))
            p.drawText(8, int(y)+4, f'{value:.1f}')
        for sec in (0, 10, 20, 30, 40, 50):
            x = left + width * sec/58
            p.drawText(int(x), self.height()-7, f'{sec}s')
        for column, color in ((1, '#2563eb'), (2, '#ea580c'), (3, '#15803d')):
            p.setPen(QPen(QColor(color), 2))
            previous = None
            for row in self.points:
                value = row[column]
                if not math.isfinite(value):
                    previous = None
                    continue
                point = (left+width*row[0]/58, top+height*(2.-value)/2.5)
                if previous:
                    p.drawLine(int(previous[0]), int(previous[1]), int(point[0]), int(point[1]))
                previous = point


class Demo(QWidget):
    def __init__(self, output):
        super().__init__()
        self.output = output
        self.setWindowTitle('cmd_vel_safety | actual ROS 2 runtime recording')
        self.setFixedSize(1280, 940)
        self.setStyleSheet('QWidget { background: white; color: #24354b; }')
        self.layout = QVBoxLayout(self)
        self.title = QLabel('ROS 2 / C++ velocity safety system — live runtime')
        self.title.setFont(QFont('DejaVu Sans', 18, QFont.Bold))
        self.stage_label = QLabel()
        self.stage_label.setFont(QFont('DejaVu Sans', 13, QFont.Bold))
        self.values = QLabel('Waiting for real node messages…')
        self.values.setFont(QFont('DejaVu Sans Mono', 10))
        self.plot = Plot()
        self.layout.addWidget(self.title)
        self.layout.addWidget(self.stage_label)
        self.layout.addWidget(self.values)
        self.layout.addWidget(self.plot)
        self.node = rclpy.create_node('demo_evidence_driver')
        self.raw_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.estop_pub = self.node.create_publisher(Bool, '/e_stop', qos)
        self.node.create_subscription(SafetyReport, '/velocity_guard/report', self.on_report, 100)
        self.node.create_subscription(Odometry, '/odom', self.on_odom, 100)
        self.node.create_subscription(MotionState, '/robot_motion_state', self.on_state, 100)
        self.graph_context = GraphContext(self.node)
        self.graph = RosGraph(self.graph_context)
        widget = self.graph_context.widget
        widget.graph_type_combo_box.setCurrentIndex(2)
        widget.filter_line_edit.setText('/')
        widget.topic_filter_line_edit.setText(','.join(TOPICS))
        widget.dead_sinks_check_box.setChecked(False)
        widget.leaf_topics_check_box.setChecked(False)
        widget.quiet_check_box.setChecked(True)
        widget.namespace_cluster_spin_box.setValue(0)
        self.graph.initialized = True
        self.layout.addWidget(QLabel('Observed ROS topology — installed rqt_graph plugin'))
        self.layout.addWidget(widget, 1)
        self.logs = QPlainTextEdit()
        self.logs.setReadOnly(True)
        self.logs.setMaximumHeight(95)
        self.logs.setFont(QFont('DejaVu Sans Mono', 8))
        self.layout.addWidget(QLabel('Actual ros2 launch stdout / stderr'))
        self.layout.addWidget(self.logs)
        self.footer = QLabel(
            'Independent ROS domain | virtual base | no audio | only this window captured')
        self.layout.addWidget(self.footer)
        self.writer = cv2.VideoWriter(str(output/'full_run.mp4'),
                                      cv2.VideoWriter_fourcc(*'avc1'), 10, (1280, 940))
        if not self.writer.isOpened():
            raise RuntimeError('OpenCV H.264 encoder unavailable')
        self.trace = (output/'reports.jsonl').open('w')
        self.launch_log = (output/'launch.log').open('w')
        self.launch = None
        self.samples = []
        self.flags = 0
        self.report, self.odom, self.state = None, None, None
        self.phase = 0
        self.estop = None
        self.last_pub = -1.
        self.frames = 0
        self.last_frame = None
        self.graph_saved = False
        self.finished = False
        self.result = None
        self.started = time.monotonic()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)
        QTimer.singleShot(200, self.start_pipeline)

    def start_pipeline(self):
        self.launch = subprocess.Popen(
            ['ros2', 'launch', 'cmd_vel_safety', 'bringup.launch.py'],
            stdout=self.launch_log, stderr=subprocess.STDOUT, start_new_session=True)
        self.logs.setPlainText('$ ros2 launch cmd_vel_safety bringup.launch.py')

    def on_odom(self, message):
        self.odom = message

    def on_state(self, message):
        self.state = message

    def on_report(self, message):
        elapsed = time.monotonic()-self.started
        self.report = message
        self.flags |= message.flags
        self.samples.append((self.phase, message))
        self.plot.points.append((elapsed, message.input_cmd.linear.x,
                                 message.target_cmd.linear.x, message.output_cmd.linear.x))
        row = dict(elapsed=elapsed, phase=self.phase,
                   stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                   input=[json_number(message.input_cmd.linear.x),
                          json_number(message.input_cmd.angular.z)],
                   target=[message.target_cmd.linear.x, message.target_cmd.angular.z],
                   output=[message.output_cmd.linear.x, message.output_cmd.angular.z],
                   flags=message.flags, estop=message.estop_active,
                   watchdog=message.watchdog_active, total_published=message.total_published)
        self.trace.write(json.dumps(row, allow_nan=False)+'\n')

    def capture(self, elapsed):
        image = self.grab().toImage().convertToFormat(QImage.Format_RGBA8888)
        ptr = image.bits()
        ptr.setsize(image.byteCount())
        rgba = np.frombuffer(ptr, np.uint8).reshape(image.height(), image.width(), 4)
        frame = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
        # Retain wall-time duration if an rqt_graph refresh blocks the event loop.
        target = int(elapsed*10)+1
        while self.frames < target-1:
            self.writer.write(self.last_frame if self.last_frame is not None else frame)
            self.frames += 1
        if self.frames < target:
            self.writer.write(frame)
            self.frames += 1
        self.last_frame = frame

    def save_graph(self):
        self.graph._update_rosgraph()
        self.graph._fit_in_view()
        self.graph_context.widget.grab().save(str(self.output/'rqt_graph.png'))
        (self.output/'rqt_graph.dot').write_text(self.graph._current_dotcode or '')
        observed = {}
        for topic in TOPICS:
            observed[topic] = {
                'publishers': [x.node_namespace.rstrip('/')+'/'+x.node_name
                               for x in self.node.get_publishers_info_by_topic(topic)],
                'subscribers': [x.node_namespace.rstrip('/')+'/'+x.node_name
                                for x in self.node.get_subscriptions_info_by_topic(topic)]}
        (self.output/'runtime_graph.json').write_text(json.dumps(observed, indent=2))
        self.graph_saved = True

    def evaluate(self):
        def rows(phase):
            return [m for p, m in self.samples if p == phase]

        def zero(message):
            return abs(message.output_cmd.linear.x)+abs(message.output_cmd.angular.z) < 1e-9

        checks = {
            'normal_motion': any(abs(m.output_cmd.linear.x-.3) < .01 for m in rows(1)),
            'over_limit_intervention': any(
                m.flags & 8 and m.flags & 16 and m.flags & 32 for m in rows(2)),
            'reverse_motion': any(m.output_cmd.linear.x < -.1 for m in rows(3)),
            'watchdog_stops': any(m.watchdog_active and zero(m) for m in rows(4)),
            'invalid_input_hold_stops': any(m.flags & 1024 and zero(m) for m in rows(6)),
            'hard_estop_zero': (any(m.estop_active for m in rows(8)) and
                                all(zero(m) for m in rows(8) if m.estop_active)),
            'release_waits_for_fresh_command': any(
                not m.estop_active and zero(m) and m.target_cmd.linear.x == 0 for m in rows(9)),
            'fresh_command_resumes': any(
                m.output_cmd.linear.x > .1 and m.output_cmd.angular.z < -.3 for m in rows(10)),
            'deadband_final_stop': any(m.flags & 128 and zero(m) for m in rows(11)),
            'all_11_safety_flags_observed': self.flags & 2047 == 2047,
            'all_outputs_within_envelope': bool(self.samples) and all(
                math.isfinite(m.output_cmd.linear.x) and math.isfinite(m.output_cmd.angular.z)
                and -.3-1e-9 <= m.output_cmd.linear.x <= 1.+1e-9
                and abs(m.output_cmd.angular.z) <= 1.5+1e-9
                and abs(m.output_cmd.linear.x*m.output_cmd.angular.z) <= 1.2+1e-9
                for _, m in self.samples),
            'motion_and_odometry_received': self.state is not None and self.odom is not None,
        }
        self.result = dict(checks=checks, passed=all(checks.values()),
                           report_count=len(self.samples), flags_seen=self.flags,
                           phases=[dict(end=end, title=title) for end, title, _, _ in PHASES],
                           source_commit=subprocess.check_output(
                               ['git', 'rev-parse', 'HEAD'], text=True).strip(),
                           ros_domain_id=os.environ.get('ROS_DOMAIN_ID'))
        (self.output/'results.json').write_text(json.dumps(self.result, indent=2))
        return self.result['passed']

    def tick(self):
        for _ in range(12):
            rclpy.spin_once(self.node, timeout_sec=0)
        elapsed = time.monotonic()-self.started
        self.phase = next((i for i, p in enumerate(PHASES) if elapsed < p[0]), len(PHASES)-1)
        _, title, command, estop = PHASES[self.phase]
        if elapsed >= 54 and not self.finished:
            ok = self.evaluate()
            self.footer.setText(
                'Scenario checks: '+('ALL PASSED' if ok else 'FAILED — see results.json'))
            self.grab().save(str(self.output/'final_state.png'))
            self.finished = True
            self.stop_pipeline()
        if not self.finished and elapsed-self.last_pub >= .1:
            if self.estop is None or estop != self.estop:
                self.estop_pub.publish(Bool(data=estop))
                self.estop = estop
            if command is not None:
                message = Twist()
                message.linear.x, message.angular.z, message.linear.y = command
                if self.phase == 6 and int(elapsed*10) % 2:
                    message.linear.x = math.inf
                self.raw_pub.publish(message)
            self.last_pub = elapsed
        if elapsed >= 7 and not self.graph_saved:
            self.save_graph()
        self.stage_label.setText(f'{elapsed:05.1f}s / 58s    {title}')
        if self.report:
            m = self.report
            motion = self.state.state_name if self.state else 'waiting'
            src = (('UNKNOWN', 'ODOMETRY', 'COMMAND')[self.state.velocity_source]
                   if self.state else '?')
            ov = self.odom.twist.twist.linear.x if self.odom else 0.
            self.values.setText(
                f'input v/w: {m.input_cmd.linear.x:+.3f} / {m.input_cmd.angular.z:+.3f}    '
                f'target: {m.target_cmd.linear.x:+.3f} / {m.target_cmd.angular.z:+.3f}\n'
                f'output:    {m.output_cmd.linear.x:+.3f} / {m.output_cmd.angular.z:+.3f}    '
                f'odom v: {ov:+.3f}    flags: 0x{m.flags:03x}\n'
                f'state: {motion}    source: {src}    watchdog: {m.watchdog_active}    '
                f'estop: {m.estop_active}    reports: {len(self.samples)}')
        self.plot.update()
        self.launch_log.flush()
        lines = (self.output/'launch.log').read_text(errors='replace').splitlines()[-5:]
        self.logs.setPlainText(re.sub(r'\x1b\[[0-9;]*m', '', '\n'.join(lines)))
        self.capture(elapsed)
        if elapsed >= 58:
            self.close()

    def stop_pipeline(self):
        if self.launch and self.launch.poll() is None:
            os.killpg(self.launch.pid, signal.SIGINT)

    def closeEvent(self, event):
        self.timer.stop()
        self.stop_pipeline()
        if self.launch:
            try:
                self.launch.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.launch.pid, signal.SIGKILL)
                self.launch.wait(timeout=3)
        self.writer.release()
        self.trace.close()
        self.launch_log.close()
        self.node.destroy_node()
        event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='new directory for video and evidence')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.setdefault('ROS_DOMAIN_ID', '181')
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    os.environ.setdefault('ROS_LOG_DIR', str(args.output/'ros_logs'))
    app = QApplication([])
    app.setStyle('Fusion')
    rclpy.init()
    demo = Demo(args.output)
    demo.show()
    app.exec_()
    rclpy.shutdown()
    print(json.dumps(demo.result, indent=2))
    return 0 if demo.result and demo.result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())

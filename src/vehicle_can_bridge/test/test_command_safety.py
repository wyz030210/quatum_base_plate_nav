"""Exercise the running command-entry executable through ROS messages only.

No CAN/serial bridge is launched.  The dedicated localhost DDS domain contains
only the command entry and this fake upstream/odometry source.
"""

import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = '173'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import pytest
import rclpy
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger
from vehicle_can_bridge.msg import Frame


class CommandBoundary:
    def __init__(self, **parameters):
        rclpy.init()
        self.node = rclpy.create_node('command_safety_test_driver')
        self.frames = []
        self.states = []
        self.node.create_subscription(Frame, '/can_tx', self.on_frame, 100)
        self.node.create_subscription(String, '/task_control/state', self.on_state, 100)
        self.command = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.feedback = self.node.create_publisher(Odometry, '/odometry/filtered_odom', 10)
        self.stop = self.node.create_publisher(Bool, '/task_control/stop', 10)
        self.heartbeat = self.node.create_publisher(Empty, '/task_control/heartbeat', 10)
        self.emergency = self.node.create_publisher(Bool, '/task_control/emergency_stop', 10)
        self.legacy_emergency = self.node.create_publisher(Bool, '/coverage/emergency_stop', 10)
        self.manual = self.node.create_publisher(Bool, '/vehicle/remote_manual_override', 10)
        self.reset = self.node.create_client(Trigger, '/task_control/reset_stop')
        defaults = dict(command_timeout_sec=0.25, feedback_timeout_sec=0.25,
                        heartbeat_timeout_sec=0.25, stopped_hold_sec=0.15,
                        stopping_timeout_sec=0.8, safety_publish_period_sec=0.02)
        defaults.update(parameters)
        executable = os.environ.get(
            'COMMAND_ENTRY_EXECUTABLE',
            str(Path(__file__).parents[3] / 'build/vehicle_can_bridge/cmd_vel_to_can'))
        args = [executable, '--ros-args']
        for key, value in defaults.items():
            args += ['-p', f'{key}:={str(value).lower()}']
        self.log = tempfile.TemporaryFile(mode='w+')
        self.process = subprocess.Popen(args, stdout=self.log, stderr=self.log,
                                        env=os.environ.copy())
        self.until(lambda: self.command.get_subscription_count() > 0)

    def on_frame(self, msg):
        if msg.id == 6 and msg.dlc == 8 and msg.data[0] == 1:
            erpm = int.from_bytes(bytes(msg.data[1:5]), 'big', signed=True)
            servo = int.from_bytes(bytes(msg.data[5:7]), 'big', signed=True)
            self.frames.append((time.monotonic(), erpm, servo))

    def on_state(self, msg):
        self.states.append(json.loads(msg.data, parse_constant=self.reject_constant))

    @staticmethod
    def reject_constant(value):
        raise AssertionError(f'Non-finite JSON state: {value}')

    def pump(self, duration, feedback=True, command=None, heartbeat=False,
             linear=0.0, angular=0.0, stamp_offset=0.0, stamp=None, yaw=0.0):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if feedback:
                odom = Odometry()
                odom.header.stamp = stamp or self.node.get_clock().now().to_msg()
                if stamp_offset:
                    odom.header.stamp.sec += stamp_offset
                odom.twist.twist.linear.x = linear
                odom.twist.twist.angular.z = angular
                self.feedback.publish(odom)
            if command is not None:
                msg = Twist()
                msg.linear.x = command
                msg.angular.z = yaw
                self.command.publish(msg)
            if heartbeat:
                self.heartbeat.publish(Empty())
            rclpy.spin_once(self.node, timeout_sec=0.01)
            for _ in range(10):
                rclpy.spin_once(self.node, timeout_sec=0.0)
            time.sleep(0.01)

    def until(self, predicate, timeout=3.0, **pump_args):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            assert self.process.poll() is None, self.output()
            self.pump(0.02, **pump_args)
        assert predicate(), self.output()

    def output(self):
        self.log.flush()
        self.log.seek(0)
        return self.log.read()

    def set_bool(self, publisher, value, **pump_args):
        self.until(lambda: publisher.get_subscription_count() > 0, **pump_args)
        publisher.publish(Bool(data=value))
        self.pump(0.06, **pump_args)

    def reset_result(self, **pump_args):
        self.until(lambda: self.reset.service_is_ready(), **pump_args)
        future = self.reset.call_async(Trigger.Request())
        self.until(future.done, **pump_args)
        return future.result()

    def close(self):
        self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=2)
        self.node.destroy_node()
        rclpy.shutdown()
        self.log.close()


@pytest.fixture
def boundary():
    instances = []

    def create(**parameters):
        instance = CommandBoundary(**parameters)
        instances.append(instance)
        return instance

    yield create
    for instance in reversed(instances):
        instance.close()


def test_legacy_navigation_stops_on_command_loss_then_accepts_a_fresh_goal(boundary):
    io = boundary()
    io.until(lambda: any(frame[1] > 0 for frame in io.frames), command=0.2)
    lost_at = time.monotonic()
    io.pump(0.45)
    assert any(t > lost_at and erpm == 0 and servo == 90
               for t, erpm, servo in io.frames), 'Upstream exit must produce CAN stop'
    io.pump(0.3)
    assert io.frames[-1][1:] == (0, 90), 'No cached motion may resume while idle'
    second_goal_at = time.monotonic()
    io.until(lambda: any(t > second_goal_at and erpm > 0 for t, erpm, _ in io.frames),
             command=0.15)
    # A normal Nav2 goal ends by publishing zero, then can remain idle indefinitely.
    io.pump(0.05, command=0.0)
    io.pump(0.45)
    third_goal_at = time.monotonic()
    io.until(lambda: any(t > third_goal_at and erpm > 0 for t, erpm, _ in io.frames),
             command=0.1)


def test_task_stop_dominates_commands_and_release_needs_a_fresh_command(boundary):
    io = boundary(task_control_required=True)
    io.pump(0.15, command=0.2, heartbeat=True)
    assert not any(erpm for _, erpm, _ in io.frames), 'Task startup must inhibit motion'
    io.until(lambda: io.states and io.states[-1]['ready'], heartbeat=True)
    assert io.states[-1]['stop_requested']
    io.stop.publish(Bool(data=False))
    io.pump(0.08, heartbeat=True)
    assert io.frames[-1][1] == 0, 'Authorization must not replay pre-release velocity'
    moving_at = time.monotonic()
    io.until(lambda: any(t > moving_at and erpm > 0 for t, erpm, _ in io.frames),
             command=0.2, heartbeat=True)
    io.stop.publish(Bool(data=True))
    io.pump(0.1, command=0.2, heartbeat=True)
    assert io.frames[-1][1:] == (0, 90), 'Stop must dominate ongoing velocity input'
    io.stop.publish(Bool(data=False))
    io.pump(0.08, heartbeat=True)
    assert io.frames[-1][1] == 0, 'Blocked commands cannot be cached for replay'
    moving_at = time.monotonic()
    io.until(lambda: any(t > moving_at and erpm > 0 for t, erpm, _ in io.frames),
             command=0.1, heartbeat=True)


def test_stopped_requires_new_continuous_feedback_and_expires(boundary):
    io = boundary()
    io.until(lambda: io.states)
    # One zero sample cannot become parked merely because wall time passes.
    io.pump(0.03)
    io.pump(0.18, feedback=False)
    assert not io.states[-1]['stopped']
    io.until(lambda: io.states[-1]['stopped'])

    assert io.states[-1]['feedback_valid']
    assert io.states[-1]['feedback_stamp_sec'] > 0
    io.pump(0.35, feedback=False)
    assert not io.states[-1]['stopped']
    assert not io.states[-1]['feedback_valid']
    # Repeated old zero messages are received now but cannot confirm a stop.
    io.pump(0.3, stamp_offset=-5)
    assert not io.states[-1]['stopped']
    assert not io.states[-1]['ready']
    io.until(lambda: io.states[-1]['stopped'])
    frozen_stamp = io.node.get_clock().now().to_msg()
    io.pump(0.3, stamp=frozen_stamp)
    assert not io.states[-1]['stopped'], 'Duplicate source stamps cannot accumulate rest'
    io.pump(0.2, linear=float('nan'))
    assert not io.states[-1]['stopped']
    io.until(lambda: io.states[-1]['stopped'])
    # Old stationary samples must not carry into the next stopping episode.
    io.until(lambda: io.frames[-1][1] > 0, command=0.2)
    io.stop.publish(Bool(data=True))
    io.pump(0.05, feedback=False)
    assert not io.states[-1]['stopped']
    io.until(lambda: io.states[-1]['stopped'])


def test_emergency_aliases_are_or_latched_and_reset_cannot_authorize_motion(boundary):
    io = boundary()
    io.until(lambda: io.frames and io.frames[-1][1] > 0, command=0.2)
    io.set_bool(io.emergency, True, command=0.2)
    assert io.frames[-1][1:] == (0, 90)
    io.set_bool(io.legacy_emergency, True)
    io.set_bool(io.emergency, False)
    io.until(lambda: io.states[-1]['stopped'])
    assert io.states[-1]['emergency_stop']
    assert not io.reset_result().success, 'Legacy alias still asserted'
    io.set_bool(io.legacy_emergency, False, command=0.2)
    assert io.states[-1]['emergency_stop'], 'Deassertion must preserve the hold'
    io.set_bool(io.stop, False, command=0.2)
    assert io.frames[-1][1] == 0, 'New run cannot reset emergency hold'
    io.pump(0.3, feedback=False)
    assert not io.reset_result(feedback=False).success, 'Stale feedback cannot permit reset'
    io.until(lambda: io.states[-1]['stopped'])
    assert io.reset_result().success
    io.pump(0.08, command=0.2)
    assert not io.states[-1]['emergency_stop']
    assert io.states[-1]['stop_requested']
    assert io.frames[-1][1] == 0, 'Reset alone cannot authorize motion'
    io.set_bool(io.stop, False)
    io.pump(0.06)
    assert io.frames[-1][1] == 0
    io.until(lambda: io.frames[-1][1] > 0, command=0.1)


def test_manual_takeover_blocks_auto_and_return_requires_explicit_new_run(boundary):
    io = boundary()
    io.until(lambda: io.frames and io.frames[-1][1] > 0, command=0.2)
    io.set_bool(io.manual, True, command=0.2)
    assert io.states[-1]['manual_override']
    assert not io.states[-1]['ready']
    assert io.frames[-1][1] == 0
    io.until(lambda: io.states[-1]['stopped'])
    assert not io.reset_result().success
    io.set_bool(io.manual, False, command=0.2)
    assert io.states[-1]['stop_requested']
    assert io.frames[-1][1] == 0
    io.set_bool(io.stop, False)
    assert io.frames[-1][1] == 0
    io.until(lambda: io.frames[-1][1] > 0, command=0.1)


def test_task_heartbeat_loss_latches_and_cannot_resume_cached_or_new_velocity(boundary):
    io = boundary(task_control_required=True)
    io.pump(0.3)
    assert not io.states[-1]['ready'], 'Missing supervisor must inhibit startup'
    io.until(lambda: io.states[-1]['ready'], heartbeat=True)
    io.set_bool(io.stop, False, heartbeat=True)
    io.until(lambda: io.frames[-1][1] > 0, command=0.2, heartbeat=True)
    io.pump(0.4, command=0.2)
    assert io.frames[-1][1] == 0
    assert io.states[-1]['fault'] == 'heartbeat_timeout'
    assert not io.reset_result().success, 'Missing heartbeat cannot permit reset'
    io.pump(0.2, command=0.2, heartbeat=True)
    io.set_bool(io.stop, False, command=0.2, heartbeat=True)
    assert io.frames[-1][1] == 0, 'Restored heartbeat cannot clear the fault'
    io.until(lambda: io.states[-1]['stopped'], heartbeat=True)
    assert io.reset_result(heartbeat=True).success
    io.pump(0.08, command=0.2, heartbeat=True)
    assert io.frames[-1][1] == 0
    io.set_bool(io.stop, False, heartbeat=True)
    io.until(lambda: io.frames[-1][1] > 0, command=0.1, heartbeat=True)


def test_nonfinite_velocity_stops_and_requires_explicit_recovery(boundary):
    io = boundary()
    io.until(lambda: io.frames and io.frames[-1][1] > 0, command=0.2)
    io.pump(0.1, command=float('nan'))
    assert io.frames[-1][1:] == (0, 90), 'Invalid velocity must produce a CAN stop'
    assert io.states[-1]['fault'] == 'invalid_command'
    io.pump(0.15, command=0.2)
    assert io.frames[-1][1] == 0
    io.until(lambda: io.states[-1]['stopped'])
    assert io.reset_result().success
    io.set_bool(io.stop, False)
    io.until(lambda: io.frames[-1][1] > 0, command=0.1)


def test_invalid_feedback_timestamp_stops_without_crashing(boundary):
    io = boundary()
    io.until(lambda: io.frames and io.frames[-1][1] > 0, command=0.2)
    io.pump(0.1, stamp=Time(sec=-1), command=0.2)
    io.until(lambda: io.states[-1]['fault'] == 'feedback_timeout_or_invalid', feedback=False)
    assert io.frames[-1][1:] == (0, 90)
    assert not io.states[-1]['feedback_valid']
    assert not io.states[-1]['stopped']


def test_task_command_loss_latches_until_reset_and_new_run(boundary):
    io = boundary(task_control_required=True)
    io.until(lambda: io.states and io.states[-1]['ready'], heartbeat=True)
    io.set_bool(io.stop, False, heartbeat=True)
    io.until(lambda: io.frames[-1][1] > 0, command=0.2, heartbeat=True)
    io.pump(0.4, heartbeat=True)
    assert io.frames[-1][1:] == (0, 90)
    assert io.states[-1]['fault'] == 'command_timeout'
    io.set_bool(io.stop, False, command=0.2, heartbeat=True)
    assert io.frames[-1][1] == 0
    io.until(lambda: io.states[-1]['stopped'], heartbeat=True)
    assert io.reset_result(heartbeat=True).success
    io.set_bool(io.stop, False, heartbeat=True)
    assert io.frames[-1][1] == 0
    io.until(lambda: io.frames[-1][1] > 0, command=0.1, heartbeat=True)


def test_feedback_loss_during_motion_stops_and_blocks_new_commands(boundary):
    io = boundary()
    io.until(lambda: io.frames and io.frames[-1][1] > 0, command=0.2)
    io.pump(0.4, feedback=False, command=0.2)
    assert io.frames[-1][1:] == (0, 90)
    assert io.states[-1]['fault'] == 'feedback_timeout_or_invalid'
    assert not io.states[-1]['stopped']
    assert not io.reset_result(feedback=False).success
    io.pump(0.2, command=0.2)
    assert io.frames[-1][1] == 0, 'Fresh feedback cannot itself restart automatic motion'
    io.until(lambda: io.states[-1]['stopped'])
    assert io.reset_result().success
    io.set_bool(io.stop, False)
    io.until(lambda: io.frames[-1][1] > 0, command=0.1)


def test_zero_command_timeout_is_rejected_at_startup():
    executable = os.environ.get(
        'COMMAND_ENTRY_EXECUTABLE',
        str(Path(__file__).parents[3] / 'build/vehicle_can_bridge/cmd_vel_to_can'))
    process = subprocess.Popen([executable, '--ros-args', '-p', 'command_timeout_sec:=0.0'],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, env=os.environ.copy())
    try:
        output, _ = process.communicate(timeout=1.0)
        assert process.returncode != 0, output
        assert 'command_timeout_sec' in output
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=2)


def test_stopping_timeout_remains_blocked_until_fresh_stationary_feedback(boundary):
    io = boundary(stopping_timeout_sec=0.3)
    io.until(lambda: io.frames and io.frames[-1][1] > 0, command=0.2, linear=0.2)
    io.set_bool(io.stop, True, linear=0.2)
    io.pump(0.4, command=0.2, linear=0.2)
    assert io.frames[-1][1:] == (0, 90)
    assert io.states[-1]['fault'] == 'stopping_timeout'
    assert not io.states[-1]['stopped']
    assert not io.reset_result(linear=0.2).success
    io.until(lambda: io.states[-1]['stopped'])
    assert io.reset_result().success
    io.pump(0.1, command=0.2)
    assert io.frames[-1][1] == 0


def test_command_entry_preserves_speed_limits_and_ackermann_steering(boundary):
    io = boundary(max_forward_speed=0.3, max_reverse_speed=0.1)
    io.until(lambda: io.frames and io.frames[-1][1:] == (337, 40), command=10.0, yaw=10.0)
    io.until(lambda: io.frames[-1][1:] == (-112, 190), command=-10.0, yaw=10.0)
    io.until(lambda: io.frames[-1][1:] == (0, 90), command=0.0, yaw=10.0)


@pytest.mark.parametrize('missing', ['heartbeat', 'feedback'])
def test_late_safety_sample_cannot_hide_an_expired_active_run(boundary, missing):
    # Deliberately slow the periodic checker to exercise recovery at the input
    # boundary before its next tick, without relying on scheduler timing races.
    io = boundary(task_control_required=True, safety_publish_period_sec=10.0,
                  command_timeout_sec=2.0,
                  heartbeat_timeout_sec=0.2 if missing == 'heartbeat' else 1.0,
                  feedback_timeout_sec=0.2 if missing == 'feedback' else 1.0)
    io.pump(0.2, heartbeat=True)
    io.set_bool(io.stop, False, heartbeat=True)
    io.until(lambda: io.frames[-1][1] > 0, command=0.2, heartbeat=True)
    io.pump(0.3, feedback=missing != 'feedback', heartbeat=missing != 'heartbeat')
    io.pump(0.04, heartbeat=True)
    io.pump(0.04, command=0.2, heartbeat=True)
    assert io.frames[-1][1:] == (0, 90), 'A late sample cannot silently renew an expired run'

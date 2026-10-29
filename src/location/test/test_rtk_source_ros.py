"""Real RTK executable's public status, driven through a pseudo terminal."""
import json
import os
import pty
import signal
import subprocess
import tempfile
import time

import pytest
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def nmea(body):
    checksum = 0
    for value in body.encode():
        checksum ^= value
    return f'${body}*{checksum:02X}\r\n'.encode()


def gga(utc, quality=4):
    return nmea(f'GNGGA,{utc},2929.39408860,N,10634.05546410,E,{quality},25,1.0,256.5,M,-38.0,M,,')


@pytest.fixture
def source_driver(tmp_path):
    rclpy.init()
    node = rclpy.create_node('rtk_provenance_observer')
    states, poses = [], []
    node.create_subscription(String, '/localization/rtk_source_state',
                             lambda msg: states.append(json.loads(msg.data)), 100)
    node.create_subscription(Odometry, '/odometry/gps', poses.append, 100)
    master, slave = pty.openpty()
    link = tmp_path / 'rtk-by-id'
    link.symlink_to(os.ttyname(slave))
    executable = os.environ['RTK_EXECUTABLE']
    log = tempfile.TemporaryFile(mode='w+')
    process = subprocess.Popen([executable, '--ros-args', '-p', f'serial_port:={link}'],
                               stdout=log, stderr=log, env=os.environ.copy())

    def pump(duration=0.05, messages=()):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            for message in messages:
                os.write(master, message)
            rclpy.spin_once(node, timeout_sec=0.01)

    def until(predicate, messages=(), timeout=3.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            assert process.poll() is None
            pump(messages=messages)
        assert predicate(), states[-3:]

    until(lambda: states and states[-1]['connected'])
    yield states, poses, pump, until
    process.send_signal(signal.SIGINT)
    process.wait(timeout=4)
    os.close(master)
    os.close(slave)
    log.close()
    node.destroy_node()
    rclpy.shutdown()


def test_status_exposes_independent_gga_heading_and_published_pose_evidence(source_driver):
    states, poses, pump, until = source_driver
    pump(messages=[gga('080158.30')])
    assert not states[-1]['ready']
    pump(messages=[gga('080158.40')])
    until(lambda: states[-1]['position']['sequence'] >= 2)
    pump(0.15)  # Finish queued legacy position publications before heading arrives.
    assert not states[-1]['ready'], 'Position alone has no fresh heading'
    until(lambda: states[-1]['ready'], messages=[nmea('GNHDT,123.4,T')])
    assert states[-1]['published_heading_seq'] == 0, 'Heading arrived after the last pose'
    until(lambda: states[-1]['published_heading_seq'] > 0, messages=[gga('080158.50')])
    assert poses
    position_seq = states[-1]['position']['sequence']
    # Numeric formatting and periodic driver publications cannot renew a GNSS epoch.
    pump(1.15, messages=[gga('080158.5000'), nmea('GNHDT,123.4,T')])
    assert states[-1]['position']['sequence'] == position_seq
    assert not states[-1]['ready']
    assert not states[-1]['quality']['valid']
    # RMC positions cannot restore independent fixed GGA quality.
    pump(messages=[nmea('GNRMC,080159.00,A,2929.39408860,N,10634.05546410,E,0,0,291026,,,A')])
    assert not states[-1]['ready']
    until(lambda: states[-1]['ready'], messages=[gga('080159.10'), nmea('GNHDT,123.4,T')])
    until(lambda: not states[-1]['heading']['valid'], messages=[nmea('GNHDT,,T')])
    assert not states[-1]['ready']
    until(lambda: states[-1]['ready'], messages=[gga('080159.20'), nmea('GNHDT,123.4,T')])
    until(lambda: not states[-1]['quality']['valid'], messages=[gga('080159.20', 5)])
    assert not states[-1]['ready']

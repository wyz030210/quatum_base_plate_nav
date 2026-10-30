"""Provider's public sensor-to-health boundary; no navigation or hardware."""
import json
import time

import pytest
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from nav2_mission.localization_provider_node import LocalizationProvider, map_digest


class Sensors:
    def __init__(self, node):
        self.node = node
        self.source = 'amcl'
        self.sequence = 0
        self.rtk_sequence = 0
        self.rtk_sequences = dict(position=0, quality=0, heading=0)
        self.rtk_freeze = set()
        self.rtk_status = None
        self.rtk_received = time.monotonic()
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose = node.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', durable)
        self.map = node.create_publisher(OccupancyGrid, '/map', durable)
        self.raw = node.create_publisher(Odometry, '/Odometry', 10)
        self.local = node.create_publisher(Odometry, '/odometry/filtered_odom', 10)
        self.global_odom = node.create_publisher(Odometry, '/odometry/filtered_map', 10)
        self.scan = node.create_publisher(LaserScan, '/scan', 10)
        self.tf = node.create_publisher(TFMessage, '/tf', 10)
        self.rtk = node.create_publisher(String, '/localization/rtk_source_state', 10)
        self.active = True
        self.lifecycle = node.create_service(GetState, '/amcl/get_state', self.get_state)
        self.grid = OccupancyGrid()
        self.grid.header.frame_id = 'map'
        self.grid.info.resolution = 1.0
        self.grid.info.width = self.grid.info.height = 2
        self.grid.info.origin.orientation.w = 1.0
        self.grid.data = [0, 0, 0, 0]
        self.states = []
        node.create_subscription(String, '/task_control/localization',
                                 lambda msg: self.states.append(json.loads(msg.data)), 10)

    def get_state(self, request, response):
        response.current_state.id = 3 if self.active else 2
        response.current_state.label = 'active' if self.active else 'inactive'
        return response

    def established(self):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.pose.orientation.w = 1.0
        self.pose.publish(pose)
        self.map.publish(self.grid)

    def pump(self, *, raw=True, scan=True, edge=True, frozen=None, local=True, base_tf=True,
             linear=0.0, feedback_stamp=None, global_odom=True):
        stamp = frozen or self.node.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.orientation.w = 1.0
        if raw:
            self.raw.publish(odom)
        odom.twist.twist.linear.x = linear
        if local:
            odom.header.stamp = feedback_stamp or stamp
            self.local.publish(odom)
            odom.header.stamp = stamp
        if self.source == 'rtk_ekf':
            odom.header.frame_id = 'map'
            if global_odom:
                self.global_odom.publish(odom)
            if self.rtk_status is None:
                self.rtk_sequence += 1
                state = dict(schema_version=1, driver_instance_id='test-driver',
                             connection_generation=1, discontinuity_sequence=1,
                             connected=True, ready=True, frame_id='map', base_frame_id='base_link',
                             calibration_id='east_clockwise_ant0.319',
                             datum_lat=29.4898045, datum_lon=106.5676103, published_generation=1)
                for name in ('position', 'quality', 'heading'):
                    if name not in self.rtk_freeze:
                        self.rtk_sequences[name] = self.rtk_sequence
                    sequence = self.rtk_sequences[name]
                    state[name] = dict(valid=True, sequence=sequence, age_sec=0.0)
                    state[f'published_{name}_seq'] = sequence
                    state[f'published_{name}_age_sec'] = 0.0
            else:
                state = self.rtk_status
            self.sequence += 1
            self.rtk.publish(String(data=json.dumps(dict(state, status_seq=self.sequence))))
        laser = LaserScan()
        laser.header.stamp = stamp
        laser.header.frame_id = 'base_link'
        laser.angle_increment = 1.0
        laser.range_min, laser.range_max = 0.1, 20.0
        laser.ranges = [2.0]
        if scan:
            self.scan.publish(laser)
        transforms = []
        if edge:
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = 'map', 'odom'
            transform.header.stamp = Time(
                sec=stamp.sec + (self.source == 'amcl'), nanosec=stamp.nanosec)
            #  # configured AMCL transform_tolerance
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'odom', 'base_link'
        transform.header.stamp = stamp
        transform.transform.translation.x = 1.25
        transform.transform.rotation.w = 1.0
        if base_tf:
            transforms.append(transform)
        self.tf.publish(TFMessage(transforms=transforms))


@pytest.fixture
def provider_system(request):
    rclpy.init()
    node = rclpy.create_node('amcl')
    for key, value in [('base_frame_id', 'base_link'), ('odom_frame_id', 'odom'),
                       ('global_frame_id', 'map'), ('transform_tolerance', 1.0)]:
        node.declare_parameter(key, value)
    sensors = Sensors(node)
    sensors.source = getattr(request, 'param', 'amcl')
    # Durable pose exists before the adapter; no pose is published during tasks.
    sensors.established()
    provider = LocalizationProvider(parameter_overrides=[
        Parameter('source', value=sensors.source), Parameter('source_timeout_sec', value=0.25),
        Parameter('lifecycle_timeout_sec', value=0.25),
        Parameter('map_id', value=map_digest(sensors.grid)),
        Parameter('localization_profile', value='test-map-amcl'),
        Parameter('base_frame_id', value='base_link'),
        Parameter('scan_frame_id', value='base_link'),
        Parameter('expected_amcl_node', value='/amcl')])
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(provider)

    def pump(duration=0.05, **kwargs):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            sensors.pump(**kwargs)
            for _ in range(12):
                executor.spin_once(timeout_sec=0.001)

    def until(predicate, **kwargs):
        deadline = time.monotonic() + 3.0
        while not predicate() and time.monotonic() < deadline:
            pump(**kwargs)
        assert predicate(), sensors.states[-3:]

    yield sensors, provider, pump, until
    executor.shutdown()
    provider.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


def test_stationary_amcl_keeps_established_pose_with_scan_processing(provider_system):
    sensors, provider, pump, until = provider_system
    until(lambda: sensors.states and sensors.states[-1]['valid'])
    pump(0.7)
    assert sensors.states[-1]['valid'], sensors.states[-1]
    assert sensors.states[-1]['x'] == pytest.approx(1.25)
    assert sensors.states[-1]['source_age_sec'] < 0.25


@pytest.mark.parametrize('missing', ['raw', 'scan', 'edge'])
def test_downstream_ekf_and_tf_cannot_hide_original_source_loss(provider_system, missing):
    sensors, provider, pump, until = provider_system
    until(lambda: sensors.states and sensors.states[-1]['valid'])
    pump(0.45, **{missing: False})
    assert not sensors.states[-1]['valid'], sensors.states[-1]
    until(lambda: sensors.states[-1]['valid'])


def test_fresh_local_odometry_cannot_hide_base_transform_loss(provider_system):
    sensors, provider, pump, until = provider_system
    until(lambda: sensors.states and sensors.states[-1]['valid'])
    pump(0.45, base_tf=False)
    assert not sensors.states[-1]['valid'], sensors.states[-1]


def test_amcl_live_frame_configuration_must_match_selected_profile(provider_system):
    sensors, provider, pump, until = provider_system
    until(lambda: sensors.states and sensors.states[-1]['valid'])
    sensors.node.set_parameters([Parameter('transform_tolerance', value=2.0)])
    pump(0.5)
    assert not sensors.states[-1]['valid']


def test_future_pose_cannot_establish_localization(provider_system):
    sensors, provider, pump, until = provider_system
    until(lambda: sensors.states and sensors.states[-1]['valid'])
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = sensors.node.get_clock().now().to_msg()
    pose.header.stamp.sec += 10
    pose.pose.pose.orientation.w = 1.0
    sensors.pose.publish(pose)
    pump(0.2)
    assert not sensors.states[-1]['valid']


@pytest.mark.parametrize('provider_system', ['rtk_ekf'], indirect=True)
def test_republication_cannot_refresh_unchanged_rtk_measurements(provider_system):
    sensors, provider, pump, until = provider_system
    until(lambda: sensors.states and sensors.states[-1]['valid'])
    # Periodic status sequence/publication time are not new measurements.
    sensors.rtk_status = dict(
        schema_version=1, driver_instance_id='test-driver',
        connection_generation=1, discontinuity_sequence=1, connected=True, ready=True,
        frame_id='map', base_frame_id='base_link', calibration_id='east_clockwise_ant0.319',
        datum_lat=29.4898045, datum_lon=106.5676103, published_generation=1)
    for name in ('position', 'quality', 'heading'):
        sensors.rtk_status[name] = dict(valid=True, sequence=sensors.rtk_sequence, age_sec=0.0)
        sensors.rtk_status[f'published_{name}_seq'] = sensors.rtk_sequence
        sensors.rtk_status[f'published_{name}_age_sec'] = 0.0
    pump(1.1)
    assert not sensors.states[-1]['valid'], sensors.states[-1]


def test_lifecycle_reset_requires_reestablished_pose(provider_system):
    sensors, provider, pump, until = provider_system
    until(lambda: sensors.states and sensors.states[-1]['valid'])
    sensors.active = False
    pump(0.2)
    assert not sensors.states[-1]['valid']
    sensors.active = True
    pump(0.3)
    assert not sensors.states[-1]['valid'], 'A prior active period cannot establish a new one'
    sensors.established()
    until(lambda: sensors.states[-1]['valid'])


def test_map_change_and_restore_cannot_reuse_previous_pose_witness(provider_system):
    sensors, provider, pump, until = provider_system
    until(lambda: sensors.states and sensors.states[-1]['valid'])
    sensors.grid.data[0] = 100
    sensors.map.publish(sensors.grid)
    pump(0.2)
    assert not sensors.states[-1]['valid']
    sensors.grid.data[0] = 0
    sensors.map.publish(sensors.grid)
    pump(0.2)
    assert not sensors.states[-1]['valid']
    sensors.established()
    until(lambda: sensors.states[-1]['valid'])


def test_restamped_pose_and_frozen_scan_epoch_do_not_prove_scan_processing(provider_system):
    sensors, provider, pump, until = provider_system
    until(lambda: sensors.states and sensors.states[-1]['valid'])
    frozen = sensors.node.get_clock().now().to_msg()
    pump(0.45, frozen=frozen)
    sensors.established()
    pump(0.1, frozen=frozen)
    assert not sensors.states[-1]['valid']
    until(lambda: sensors.states[-1]['valid'])


@pytest.mark.parametrize('provider_system', ['rtk_ekf'], indirect=True)
def test_unrepresentable_source_age_is_invalid_without_crashing_provider(provider_system):
    sensors, provider, pump, until = provider_system
    until(lambda: sensors.states and sensors.states[-1]['valid'])
    sensors.rtk_status = dict(
        schema_version=1, driver_instance_id='test-driver', connection_generation=1,
        discontinuity_sequence=1, connected=True, ready=True, frame_id='map',
        base_frame_id='base_link', calibration_id='east_clockwise_ant0.319',
        datum_lat=29.4898045, datum_lon=106.5676103, published_generation=1)
    for name in ('position', 'quality', 'heading'):
        sensors.rtk_status[name] = dict(valid=True, sequence=sensors.rtk_sequence + 1, age_sec=0.0)
        sensors.rtk_status[f'published_{name}_seq'] = sensors.rtk_sequence + 1
        sensors.rtk_status[f'published_{name}_age_sec'] = 0.0
    sensors.rtk_status['position']['age_sec'] = 10 ** 400
    pump(0.15)
    assert not sensors.states[-1]['valid']
    sensors.rtk_status = None
    sensors.rtk_sequence += 2
    until(lambda: sensors.states[-1]['valid'])

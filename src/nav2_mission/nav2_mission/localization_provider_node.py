"""Localization health from original measurements, with one common task pose.

No estimator, pose reset, TF publication, or task-specific initialization lives here.
The deployed profile owns the AMCL/RTK producers and frame configuration. Humble's
Python subscription API cannot attribute each TF edge to a writer; endpoint
checks are topology checks, not authentication of individual TF messages.
"""
from collections import deque
import hashlib
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

from .task_ros_adapters import decode_json, finite_number


def stamp_ns(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000:
        raise ValueError('invalid source timestamp')
    return stamp.sec * 1000000000 + stamp.nanosec


def quaternion_yaw(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('nonfinite quaternion')
    norm = sum(value * value for value in values)
    if abs(norm - 1.0) > 0.01:
        raise ValueError('invalid quaternion')
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def map_digest(grid):
    """Map identity excludes publication/load timestamps, includes geometry/data."""
    origin = grid.info.origin
    values = [grid.header.frame_id, grid.info.width, grid.info.height,
              grid.info.resolution, origin.position.x, origin.position.y,
              origin.position.z, origin.orientation.x, origin.orientation.y,
              origin.orientation.z, origin.orientation.w, list(grid.data)]
    payload = json.dumps(values, separators=(',', ':'), allow_nan=False)
    return hashlib.sha256(payload.encode()).hexdigest()


class LocalizationProvider(Node):
    def __init__(self, **kwargs):
        super().__init__('localization_provider', **kwargs)
        defaults = {
            'source': 'amcl', 'global_frame_id': 'map', 'odom_frame_id': 'odom',
            'base_frame_id': 'base_footprint', 'raw_base_frame_id': 'base_link',
            'scan_frame_id': 'base_footprint', 'map_id': '',
            'localization_profile': '', 'source_timeout_sec': 1.0,
            'future_tolerance_sec': 0.1, 'lifecycle_timeout_sec': 1.0,
            'amcl_transform_tolerance_sec': 1.0, 'expected_amcl_node': '/amcl',
            'rtk_position_timeout_sec': 1.0, 'rtk_quality_timeout_sec': 1.0,
            'rtk_heading_timeout_sec': 1.0, 'rtk_calibration_id': 'east_clockwise_ant0.319',
            'rtk_datum_lat': 29.4898045, 'rtk_datum_lon': 106.5676103,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.config = {key: self.get_parameter(key).value for key in defaults}
        if self.config['source'] not in ('amcl', 'rtk_ekf'):
            raise ValueError('source must be amcl or rtk_ekf')
        for key in ('source_timeout_sec', 'lifecycle_timeout_sec',
                    'rtk_position_timeout_sec', 'rtk_quality_timeout_sec',
                    'rtk_heading_timeout_sec'):
            if not finite_number(self.config[key]) or self.config[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        for key in ('future_tolerance_sec', 'amcl_transform_tolerance_sec'):
            if not finite_number(self.config[key]) or self.config[key] < 0:
                raise ValueError(f'{key} must be finite and nonnegative')
        self.evidence = {}
        self.generation = 0
        self.pose_witness = None
        self.pose_writer = None
        self.pose_writer_seen = False
        self.map_ok = False
        self.map_seen = None
        self.lifecycle_active = False
        self.lifecycle_received = -math.inf
        self.lifecycle_future = None
        self.parameters_future = None
        self.parameters_received = -math.inf
        self.parameters_match = False
        self.scans = deque(maxlen=100)
        self.edges = deque(maxlen=100)
        self.rtk = None
        self.rtk_received = -math.inf
        self.rtk_identity = None
        self.rtk_status_seq = -1
        self.rtk_measurements = {}
        self.retired_rtk_identities = set()
        self.last_clock_ns = self.get_clock().now().nanoseconds
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(String, '/task_control/localization', 10)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, durable)
        self.create_subscription(
            Odometry, '/Odometry', lambda msg: self.on_odom('raw_odom', msg),
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/odometry/filtered_odom', lambda msg: self.on_odom('local_odom', msg),
            qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        if self.config['source'] == 'amcl':
            self.create_subscription(
                PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, durable)
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
            self.lifecycle = self.create_client(
                GetState, self.config['expected_amcl_node'] + '/get_state')
            self.parameters_client = self.create_client(
                GetParameters, self.config['expected_amcl_node'] + '/get_parameters')
        else:
            self.create_subscription(String, '/localization/rtk_source_state', self.on_rtk, 10)
            self.create_subscription(
                Odometry, '/odometry/filtered_map', lambda msg: self.on_odom('global_odom', msg),
                qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info('Localization measurement policy: ' + json.dumps(dict(
            self.config, use_sim_time=self.get_parameter('use_sim_time').value), sort_keys=True))

    def age(self, evidence):
        stamp, received = evidence
        return max(time.monotonic() - received,
                   (self.get_clock().now().nanoseconds - stamp) / 1e9)

    def record(self, name, stamp):
        now = time.monotonic()
        old = self.evidence.get(name)
        source_age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if source_age < -self.config['future_tolerance_sec'] or stamp <= 0:
            self.evidence.pop(name, None)
            self.generation += 1
            return
        if old:
            if stamp < old[0]:
                self.evidence.pop(name, None)
                self.generation += 1
                return
            if stamp == old[0]:
                return
            if self.age(old) > self.config['source_timeout_sec']:
                self.generation += 1
        self.evidence[name] = (stamp, now)

    def on_map(self, msg):
        try:
            digest = map_digest(msg)
            valid = (msg.header.frame_id == self.config['global_frame_id'] and
                     digest == self.config['map_id'])
            if self.map_seen is not None and digest != self.map_seen:
                self.pose_witness = None
                self.evidence.clear()
                self.generation += 1
            self.map_seen = digest
            self.map_ok = valid
        except (ValueError, TypeError):
            self.map_ok = False

    def on_odom(self, name, msg):
        try:
            expected = self.config['global_frame_id' if name == 'global_odom' else 'odom_frame_id']
            if (msg.header.frame_id != expected or
                    msg.child_frame_id != self.config['raw_base_frame_id']):
                raise ValueError('odometry frame mismatch')
            pose = msg.pose.pose
            if not all(math.isfinite(value) for value in (
                    pose.position.x, pose.position.y, pose.position.z)):
                raise ValueError('nonfinite pose')
            quaternion_yaw(pose.orientation)
            self.record(name, stamp_ns(msg.header.stamp))
        except ValueError:
            self.evidence.pop(name, None)
            self.generation += 1

    def on_pose(self, msg):
        try:
            if msg.header.frame_id != self.config['global_frame_id']:
                raise ValueError('AMCL pose frame mismatch')
            pose = msg.pose.pose
            quaternion_yaw(pose.orientation)
            if not all(math.isfinite(value) for value in (
                    pose.position.x, pose.position.y, *msg.pose.covariance)):
                raise ValueError('invalid AMCL pose')
            if any(msg.pose.covariance[index] < 0 for index in (0, 7, 35)):
                raise ValueError('invalid AMCL covariance')
            stamp = stamp_ns(msg.header.stamp)
            future = (stamp - self.get_clock().now().nanoseconds) / 1e9
            if stamp <= 0 or future > self.config['future_tolerance_sec']:
                raise ValueError('invalid AMCL witness timestamp')
            self.pose_witness = stamp
        except ValueError:
            self.pose_witness = None

    def on_scan(self, msg):
        try:
            if msg.header.frame_id != self.config['scan_frame_id']:
                raise ValueError('scan frame mismatch')
            stamp = stamp_ns(msg.header.stamp)
            self.scans.append((stamp, time.monotonic()))
            self.match_scans()
        except ValueError:
            self.evidence.pop('scan_processing', None)
            self.generation += 1

    def on_tf(self, msg):
        for transform in msg.transforms:
            if (transform.header.frame_id == self.config['global_frame_id'] and
                    transform.child_frame_id == self.config['odom_frame_id']):
                try:
                    quaternion_yaw(transform.transform.rotation)
                    if not all(math.isfinite(value) for value in (
                            transform.transform.translation.x, transform.transform.translation.y,
                            transform.transform.translation.z)):
                        raise ValueError('invalid transform')
                    stamp = stamp_ns(transform.header.stamp)
                    if self.config['source'] == 'amcl':
                        tolerance = round(self.config['amcl_transform_tolerance_sec'] * 1e9)
                        implied_scan = stamp - tolerance
                        self.edges.append((implied_scan, time.monotonic()))
                        self.match_scans()
                    else:
                        self.record('map_odom', stamp)
                except ValueError:
                    self.evidence.pop('scan_processing', None)
                    self.evidence.pop('map_odom', None)
                    self.generation += 1

    def match_scans(self):
        # Either DDS callback order is legal. Only individual dynamic map->odom
        # observations correlated to a newly acquired scan renew this proof.
        matches = [(scan, min(sr, er)) for scan, sr in self.scans for edge, er in self.edges
                   if abs(scan - edge) <= 1000]
        if matches:
            stamp, received = max(matches)
            previous = self.evidence.get('scan_processing')
            self.record('scan_processing', stamp)
            if not previous or stamp > previous[0]:
                self.evidence['scan_processing'] = (stamp, received)

    def on_rtk(self, msg):
        try:
            data = decode_json(msg.data)
            if type(data.get('ready')) is not bool or type(data.get('connected')) is not bool:
                raise ValueError('invalid RTK readiness')
            identity = (str(data['driver_instance_id']), int(data['connection_generation']))
            sequence = int(data['status_seq'])
            if data.get('schema_version') != 1 or identity in self.retired_rtk_identities:
                raise ValueError('unsupported or retired RTK source identity')
            if identity == self.rtk_identity and sequence <= self.rtk_status_seq:
                return
            if identity != self.rtk_identity:
                self.generation += 1
                if self.rtk_identity is not None:
                    self.retired_rtk_identities.add(self.rtk_identity)
                self.rtk_measurements.clear()
            elif self.rtk and data['discontinuity_sequence'] != self.rtk['discontinuity_sequence']:
                self.generation += 1
            self.rtk_identity, self.rtk_status_seq = identity, sequence
            for name in ('position', 'quality', 'heading'):
                value = data[name]
                if type(value['valid']) is not bool or type(value['sequence']) is not int:
                    raise ValueError('invalid RTK source evidence')
                if value['valid'] and (
                        not finite_number(value['age_sec']) or value['age_sec'] < 0):
                    raise ValueError('invalid RTK source age')
                measurements = [(name, value['sequence'], value['age_sec']),
                                ('published_' + name, data.get(f'published_{name}_seq'),
                                 data.get(f'published_{name}_age_sec'))]
                for key, seq, age in measurements:
                    if type(seq) is not int or seq < 0:
                        raise ValueError('invalid RTK measurement sequence')
                    previous = self.rtk_measurements.get(key)
                    if previous and seq < previous[0]:
                        raise ValueError('RTK measurement sequence regressed')
                    if not previous or seq > previous[0]:
                        if (previous and name == key and finite_number(previous[2]) and
                                previous[2] + time.monotonic() - previous[1] >
                                self.config[f'rtk_{name}_timeout_sec']):
                            self.generation += 1
                        self.rtk_measurements[key] = (seq, time.monotonic(), age)
            self.rtk, self.rtk_received = data, time.monotonic()
        except (ValueError, KeyError, TypeError):
            self.rtk = None
            self.generation += 1

    def amcl_error(self):
        publishers = self.get_publishers_info_by_topic('/amcl_pose')
        expected = self.config['expected_amcl_node']
        writers = [tuple(endpoint.endpoint_gid) for endpoint in publishers
                   if (endpoint.node_namespace.rstrip('/') + '/' + endpoint.node_name) == expected]
        if len(writers) != 1 or len(publishers) != 1:
            return 'amcl_pose_writer_unavailable_or_ambiguous'
        writer = writers[0]
        if self.pose_writer_seen and writer != self.pose_writer:
            self.pose_witness = None
            self.generation += 1
        self.pose_writer, self.pose_writer_seen = writer, True
        if self.pose_witness is None:
            return 'amcl_established_pose_unavailable'
        if (not self.lifecycle_active or time.monotonic() - self.lifecycle_received >
                self.config['lifecycle_timeout_sec']):
            return 'amcl_lifecycle_inactive_or_expired'
        if (not self.parameters_match or time.monotonic() - self.parameters_received >
                self.config['lifecycle_timeout_sec']):
            return 'amcl_frame_or_transform_tolerance_unverified'
        return ''

    def rtk_error(self):
        data = self.rtk
        if not data:
            return 'rtk_source_state_unavailable', []
        elapsed = time.monotonic() - self.rtk_received
        if elapsed > self.config['source_timeout_sec']:
            return 'rtk_source_state_expired', []
        if not data.get('ready') or not data.get('connected'):
            return 'rtk_measurements_invalid_or_not_progressing', []
        if (data.get('frame_id') != self.config['global_frame_id'] or
                data.get('base_frame_id') != self.config['raw_base_frame_id'] or
                data.get('calibration_id') != self.config['rtk_calibration_id'] or
                data.get('datum_lat') != self.config['rtk_datum_lat'] or
                data.get('datum_lon') != self.config['rtk_datum_lon']):
            return 'rtk_frame_or_datum_calibration_mismatch', []
        ages = []
        for name in ('position', 'quality', 'heading'):
            evidence = data[name]
            anchor = self.rtk_measurements[name]
            age = max((evidence['age_sec'] or 0) + elapsed,
                      (anchor[2] or 0) + time.monotonic() - anchor[1])
            if not evidence['valid'] or age > self.config[f'rtk_{name}_timeout_sec']:
                return f'rtk_{name}_invalid_or_expired', []
            published_age = data.get(f'published_{name}_age_sec')
            anchor = self.rtk_measurements['published_' + name]
            if finite_number(published_age):
                published_age = max(published_age + elapsed,
                                    (anchor[2] or 0) + time.monotonic() - anchor[1])
            if (data.get('published_generation') != data['connection_generation'] or
                    data.get(f'published_{name}_seq', 0) <= 0 or
                    not finite_number(published_age) or published_age < 0 or
                    published_age > self.config[f'rtk_{name}_timeout_sec']):
                return f'rtk_{name}_not_in_fresh_published_pose', []
            ages.extend((age, published_age))
        return '', ages

    def lifecycle_tick(self):
        if self.parameters_future is not None and self.parameters_future.done():
            try:
                values = self.parameters_future.result().values
                self.parameters_match = (
                    len(values) == 4 and [value.string_value for value in values[:3]] == [
                        self.config['base_frame_id'], self.config['odom_frame_id'],
                        self.config['global_frame_id']] and abs(
                            values[3].double_value -
                            self.config['amcl_transform_tolerance_sec']) < 1e-9)
                if not self.parameters_match:
                    self.pose_witness = None
                    self.generation += 1
                self.parameters_received = time.monotonic()
            except Exception:
                self.parameters_match = False
            self.parameters_future = None
        if self.parameters_future is None and self.parameters_client.service_is_ready():
            self.parameters_future = self.parameters_client.call_async(GetParameters.Request(
                names=['base_frame_id', 'odom_frame_id', 'global_frame_id',
                       'transform_tolerance']))
        if self.lifecycle_future is not None and self.lifecycle_future.done():
            try:
                active = self.lifecycle_future.result().current_state.id == 3
                if not active and self.lifecycle_active:
                    self.pose_witness = None
                    self.generation += 1
                self.lifecycle_active = active
                self.lifecycle_received = time.monotonic()
            except Exception:
                self.lifecycle_active = False
            self.lifecycle_future = None
        if self.lifecycle_future is None and self.lifecycle.service_is_ready():
            self.lifecycle_future = self.lifecycle.call_async(GetState.Request())

    def tick(self):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns < self.last_clock_ns:
            self.evidence.clear()
            self.pose_witness = None
            self.generation += 1
        self.last_clock_ns = now_ns
        if self.config['source'] == 'amcl':
            self.lifecycle_tick()
        data = dict(schema_version=1, valid=False, source=self.config['source'],
                    frame_id=self.config['global_frame_id'],
                    base_frame_id=self.config['base_frame_id'],
                    odom_frame_id=self.config['odom_frame_id'],
                    map_id=self.config['map_id'],
                    localization_profile=self.config['localization_profile'],
                    observed_map_id=self.map_seen,
                    evidence_generation=self.generation, source_age_sec=None,
                    source_stamp_sec=None, pose_stamp_sec=None, x=None, y=None, yaw=None, error='')
        try:
            if (not self.config['map_id'] or not self.config['localization_profile'] or
                    not self.map_ok):
                raise ValueError('map_or_localization_profile_unverified')
            required = ['raw_odom', 'local_odom']
            source_ages = []
            if self.config['source'] == 'amcl':
                error = self.amcl_error()
                required.append('scan_processing')
            else:
                error, source_ages = self.rtk_error()
                required += ['global_odom', 'map_odom']
            if error:
                raise ValueError(error)
            for name in required:
                if name not in self.evidence:
                    raise ValueError(f'{name}_unavailable')
                age = self.age(self.evidence[name])
                if (age > self.config['source_timeout_sec'] or
                        age < -self.config['future_tolerance_sec']):
                    raise ValueError(f'{name}_expired')
                source_ages.append(max(0.0, age))
            transform = self.buffer.lookup_transform(
                self.config['global_frame_id'], self.config['base_frame_id'], rclpy.time.Time())
            tf_stamp = stamp_ns(transform.header.stamp)
            tf_age = (now_ns - tf_stamp) / 1e9
            if (tf_age > self.config['source_timeout_sec'] or
                    tf_age < -self.config['future_tolerance_sec']):
                raise ValueError('map_base_transform_expired')
            source_ages.append(max(0.0, tf_age))
            pose = transform.transform
            yaw = quaternion_yaw(pose.rotation)
            if not all(math.isfinite(value) for value in (pose.translation.x, pose.translation.y)):
                raise ValueError('nonfinite_global_pose')
            data.update(valid=True, x=pose.translation.x, y=pose.translation.y, yaw=yaw,
                        source_age_sec=max(source_ages),
                        source_stamp_sec=now_ns / 1e9 - max(source_ages),
                        pose_stamp_sec=stamp_ns(transform.header.stamp) / 1e9)
        except Exception as exc:
            data['error'] = str(exc)
        data['evidence_generation'] = self.generation
        self.publisher.publish(String(data=json.dumps(data, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationProvider()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

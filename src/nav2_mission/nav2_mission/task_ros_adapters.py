"""ROS boundary adapters; task coordination never initializes localization."""
import json
import math
import time
import uuid

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Bool, Empty, String
from unique_identifier_msgs.msg import UUID

from .task_adapters import ExecutionUpdate, LocalizationState, StopState


def finite_number(value):
    try:
        return (not isinstance(value, bool) and isinstance(value, (int, float)) and
                math.isfinite(value))
    except OverflowError:
        return False


def decode_json(text):
    def invalid(value):
        raise ValueError(f'nonfinite JSON constant: {value}')
    data = json.loads(text, parse_constant=invalid)
    if not isinstance(data, dict):
        raise ValueError('expected JSON object')
    return data


class RosLocalization:
    def __init__(self, node, topic, clock=time.monotonic):
        self.node = node
        self.clock = clock
        self.state = LocalizationState(False, '', -math.inf, math.inf,
                                       'no localization state')
        self.subscription = node.create_subscription(String, topic, self.receive, 10)

    def receive(self, message):
        now = self.clock()
        try:
            data = decode_json(message.data)
            if type(data['valid']) is not bool or not isinstance(data['frame_id'], str):
                raise ValueError('invalid localization validity/frame')
            schema = data.get('schema_version', 0)
            if schema not in (0, 1):
                raise ValueError('unsupported localization schema')
            if data['valid'] and not all(finite_number(data[k]) for k in ('x', 'y', 'yaw')):
                raise ValueError('invalid localization pose')
            if schema == 1:
                age = data.get('source_age_sec')
                if data['valid'] and (not finite_number(age) or age < 0):
                    raise ValueError('invalid original measurement age')
                source_age = age if finite_number(age) else math.inf
                for key in ('map_id', 'localization_profile'):
                    if not isinstance(data.get(key), str) or not data[key]:
                        raise ValueError('missing localization context')
                if type(data.get('evidence_generation')) is not int:
                    raise ValueError('invalid localization evidence generation')
            else:
                if not finite_number(data.get('source_stamp_sec')):
                    raise ValueError('invalid localization source timestamp')
                source_age = (self.node.get_clock().now().nanoseconds / 1e9 -
                              data['source_stamp_sec'])
            self.state = LocalizationState(
                data['valid'], data['frame_id'], now, source_age,
                str(data.get('error', '')), str(data.get('source', '')),
                data.get('x'), data.get('y'), data.get('yaw'), data.get('pose_stamp_sec'),
                str(data.get('map_id', '')), str(data.get('localization_profile', '')),
                data.get('evidence_generation', 0), schema)
        except (KeyError, ValueError, TypeError) as exc:
            self.state = LocalizationState(False, '', now, math.inf, str(exc))

    def snapshot(self, now):
        # Age source evidence as well as receive time, without refreshing its stamp.
        from dataclasses import replace
        return replace(self.state, source_age_sec=self.state.source_age_sec +
                       max(0, now - self.state.received_at))


class RosStop:
    def __init__(self, node, clock=time.monotonic):
        self.clock = clock
        self.state = StopState(False, False, True, -math.inf, None)
        self.publisher = node.create_publisher(Bool, '/task_control/stop', 10)
        self.heartbeat = node.create_publisher(Empty, '/task_control/heartbeat', 10)
        self.subscription = node.create_subscription(
            String, '/task_control/state', self.receive, 10)
        self.request_stop('')

    def receive(self, message):
        now = self.clock()
        try:
            data = decode_json(message.data)
            for key in ('ready', 'stopped', 'stop_requested', 'manual_override', 'emergency_stop'):
                if type(data[key]) is not bool:
                    raise ValueError(f'{key} must be boolean')
            age = data['feedback_age_sec']
            if age is not None and (not finite_number(age) or age < 0):
                raise ValueError('feedback age invalid')
            self.state = StopState(data['ready'], data['stopped'], data['stop_requested'], now,
                                   age, data['manual_override'], data['emergency_stop'],
                                   str(data.get('fault') or ''))
        except (ValueError, TypeError, KeyError) as exc:
            self.state = StopState(False, False, True, now, None, fault=str(exc))

    def snapshot(self, now):
        from dataclasses import replace
        age = self.state.feedback_age_sec
        return replace(self.state, feedback_age_sec=None if age is None else
                       age + max(0, now - self.state.received_at))

    def authorize(self, task_id):
        self.publisher.publish(Bool(data=False))

    def request_stop(self, task_id):
        self.publisher.publish(Bool(data=True))

    def beat(self):
        self.heartbeat.publish(Empty())


class RosNavigation:
    def __init__(self, node, action_name, ownership=None):
        self.ownership = ownership
        self.action_name = action_name
        self.node = node
        self.client = ActionClient(node, NavigateToPose, action_name)

    def start(self, task_id, point, frame_id):
        if not self.client.server_is_ready():
            raise RuntimeError('navigation action unavailable')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(point['x'])
        goal.pose.pose.position.y = float(point['y'])
        goal.pose.pose.orientation.z = math.sin(point['yaw'] / 2)
        goal.pose.pose.orientation.w = math.cos(point['yaw'] / 2)
        # Empty BT selects target vehicle's configured Ackermann tree, no TB3 recovery.
        goal_id = uuid.uuid4().hex
        if self.ownership is not None:
            self.ownership.register(self.action_name, goal_id, pending_navigation=True)
        return RosNavigationOperation(self.client.send_goal_async(
            goal, goal_uuid=UUID(uuid=list(bytes.fromhex(goal_id)))),
            goal_id=goal_id, ownership=self.ownership)


class RosNavigationOperation:
    """Keep ownership across unresolved accepts; cancel acceptance is not terminal."""

    def __init__(self, response, goal_id=None, ownership=None):
        self.goal_id, self.ownership = goal_id, ownership
        self.response = response
        self.handle = None
        self.result = None
        self.cancel_requested = False
        self.cancel_sent = False
        self.terminal = None

    def cancel(self):
        self.cancel_requested = True

    def poll(self, now):
        if self.terminal:
            return self.terminal
        try:
            if self.handle is None and self.response.done():
                self.handle = self.response.result()
                if not self.handle.accepted:
                    self.terminal = ExecutionUpdate(
                        'navigating', True, False, 'navigation rejected')
                    self._terminal_owned()
                    return self.terminal
                self.result = self.handle.get_result_async()
            if self.handle is not None and self.cancel_requested and not self.cancel_sent:
                self.handle.cancel_goal_async()
                self.cancel_sent = True
            if self.result is not None and self.result.done():
                result = self.result.result()
                success = result.status == GoalStatus.STATUS_SUCCEEDED
                self.terminal = ExecutionUpdate('navigating', True, success,
                                                '' if success else f'Nav2 status {result.status}')
                self._terminal_owned()
                return self.terminal
        except Exception as exc:
            # Transport uncertainty cannot prove the owned motion has ended.
            return ExecutionUpdate('navigating', error=f'navigation transport error: {exc}')
        return ExecutionUpdate('navigating')

    def _terminal_owned(self):
        if self.ownership is not None:
            self.ownership.navigation_terminal(self.goal_id)

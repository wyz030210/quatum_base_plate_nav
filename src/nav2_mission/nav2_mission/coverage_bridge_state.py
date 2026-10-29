"""Ownership and correlated telemetry for one existing coverage bridge execution."""
import copy
import json
import math
import threading
import time
import uuid

import rclpy
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.parameter import Parameter, parameter_value_to_python
from std_msgs.msg import String
from unique_identifier_msgs.msg import UUID

from .coverage_task import PLANNER_DEFAULTS


def child_ids(outer_id):
    return {stage: uuid.uuid5(uuid.UUID(hex=outer_id), stage).hex
            for stage in ('compute', 'approach', 'follow')}


def path_json(path):
    return {'frame_id': path.header.frame_id, 'poses': [
        {'x': p.pose.position.x, 'y': p.pose.position.y, 'z': p.pose.position.z,
         'orientation': [p.pose.orientation.x, p.pose.orientation.y,
                         p.pose.orientation.z, p.pose.orientation.w]}
        for p in path.poses]}


class DeferredTerminal:
    """Only publish outer terminal after all owned responses/results are drained."""
    def __init__(self, handle):
        self.handle = handle
        self.terminal = 'abort'

    def __getattr__(self, name):
        return getattr(self.handle, name)

    def succeed(self):
        self.terminal = 'succeed'

    def canceled(self):
        self.terminal = 'canceled'

    def abort(self):
        self.terminal = 'abort'


class BridgeRun:
    def __init__(self, node, goal, config, task_id):
        self.node, self.goal, self.config = node, goal, config
        self.id = bytes(goal.goal_id.uuid).hex()
        self.records = []
        self.state = {'goal_id': self.id, 'task_id': task_id, 'phase': 'dependency_check',
                      'owned_goal_ids': child_ids(self.id), 'children': {},
                      'phases': ['dependency_check'], 'planned_path': None,
                      'feedback': {}, 'error': '',
                      'config': copy.deepcopy(config)}
        self.lease_maybe_owned = False
        self.acquire_future = None
        self.state_lock = threading.Lock()
        self.emit()

    def emit(self, phase=None, **changes):
        # Feedback callbacks can run alongside execution/cleanup. Serialize the
        # complete snapshot so an older feedback copy cannot erase a failure.
        with self.state_lock:
            state = copy.deepcopy(self.state)
            if phase and phase != state['phase']:
                state['phase'] = phase
                state['phases'].append(phase)
                if phase in ('approaching', 'following'):
                    state['feedback'] = {}
            state.update(changes)
            payload = json.dumps(state, allow_nan=False)
            self.state = state
            self.node._state_pub.publish(String(data=payload))

    def feedback(self, stage, message):
        fields = ('distance_to_goal', 'speed') if stage == 'follow' else (
            'distance_remaining', 'number_of_recoveries')
        data = {}
        for name in fields:
            value = getattr(message.feedback, name, None)
            if isinstance(value, (int, float)) and math.isfinite(value):
                data['distance_remaining' if name == 'distance_to_goal' else name] = value
        if data:
            self.emit(feedback=dict(stage=stage, **data))

    def send(self, client, goal, stage):
        goal_id = self.state['owned_goal_ids'][stage]
        self.state['children'][stage] = {'goal_id': goal_id, 'state': 'response_pending'}
        self.emit()
        record = {'stage': stage, 'response': None, 'handle': None,
                  'result': None, 'cancel_sent': False}
        self.records.append(record)
        response = client.send_goal_async(
            goal, feedback_callback=lambda msg: self.feedback(stage, msg),
            goal_uuid=UUID(uuid=list(bytes.fromhex(goal_id))))
        record['response'] = response
        return response

    def resolve(self, record):
        if record['response'] is None:
            raise RuntimeError('child submission response unknown')
        if record['handle'] is None and record['response'].done():
            handle = record['response'].result()
            if handle is None:
                raise RuntimeError('no child goal response')
            record['handle'] = handle
            if handle.accepted:
                record['result'] = handle.get_result_async()
                self.state['children'][record['stage']]['state'] = 'accepted'
            else:
                self.state['children'][record['stage']]['state'] = 'rejected'
            self.emit()
        return record['handle']

    def result_for(self, handle):
        for record in self.records:
            if record['response'].done() and record['response'].result() is handle:
                self.resolve(record)
                return record['result']
        raise RuntimeError('child goal not owned')

    def drain(self):
        self.emit('stopping')
        # Cancellation acknowledgement does not establish termination. Keep this
        # outer goal and the parameter lease until every possibly accepted child
        # has a terminal result. Unknown ownership stays visible and blocked.
        while rclpy.ok():
            pending = False
            for record in self.records:
                try:
                    handle = self.resolve(record)
                    if handle is None:
                        pending = True
                    elif handle.accepted:
                        future = record['result']
                        if future.done():
                            wrapped = future.result()
                            if wrapped is None or wrapped.status not in (4, 5, 6):
                                raise RuntimeError('child result is not terminal')
                            self.state['children'][record['stage']]['state'] = 'terminal'
                            self.state['children'][record['stage']]['status'] = int(wrapped.status)
                        else:
                            pending = True
                            if not record['cancel_sent']:
                                handle.cancel_goal_async()
                                record['cancel_sent'] = True
                except Exception as exc:
                    pending = True
                    self.state['error'] = f'owned child uncertain: {exc}'
            self.emit(cleanup_pending=('waiting owned child response or terminal'
                                       if pending else ''))
            if not pending:
                return True
            time.sleep(0.05)
        return False

    def _await(self, future):
        deadline = time.monotonic() + self.node._dependency_timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                raise RuntimeError('planner parameter transaction response pending')
            time.sleep(0.02)
        if not future.done():
            raise RuntimeError('planner parameter transport shutdown')
        return future.result()

    def acquire(self):
        if self.config is None:
            return
        self.emit('preparing')
        clients = (self.node._planner_set, self.node._planner_get)
        if not all(client.wait_for_service(timeout_sec=self.node._dependency_timeout_sec)
                   for client in clients):
            raise RuntimeError('planner parameter services unavailable')
        values = dict(self.config['planner'], task_parameter_owner=self.id)
        messages = [Parameter(name, Parameter.Type.INTEGER_ARRAY, value).to_parameter_msg()
                    if isinstance(value, list) else Parameter(name, value=value).to_parameter_msg()
                    for name, value in values.items()]
        # A lost response may still have acquired ownership; cleanup must read it
        # back and release only our own lease before returning an outer result.
        self.lease_maybe_owned = True
        self.acquire_future = self.node._planner_set.call_async(
            SetParametersAtomically.Request(parameters=messages))
        response = self._await(self.acquire_future)
        if not response.result.successful:
            self.lease_maybe_owned = False
            raise RuntimeError(f'planner lease rejected: {response.result.reason}')
        names = list(PLANNER_DEFAULTS) + ['task_parameter_owner']
        readback = self._await(self.node._planner_get.call_async(
            GetParameters.Request(names=names)))
        observed = {name: parameter_value_to_python(value)
                    for name, value in zip(names, readback.values)}
        if any(observed.get(name) != value for name, value in values.items()):
            raise RuntimeError('planner snapshot readback mismatch')
        effective = {name: observed[name] for name in PLANNER_DEFAULTS}
        self.emit('preparing', planner_confirmed=True, effective_planner=effective)
        points = self.config['polygons'][0]
        minimum = min(math.hypot(points[i+1][0]-points[i][0],
                                 points[i+1][1]-points[i][1]) for i in range(4))
        if minimum <= 2 * effective['default_headland_width'] + effective['operation_width']:
            raise ValueError('rectangle too narrow for effective headland and operation width')
        self.emit('planning')

    def release(self):
        if not self.lease_maybe_owned:
            return True
        while rclpy.ok():
            try:
                if self.acquire_future is not None:
                    if not self.acquire_future.done():
                        self.emit('stopping', error='waiting pending planner lease response')
                        time.sleep(0.05)
                        continue
                    self.acquire_future.result()
                owner = self._await(self.node._planner_get.call_async(
                    GetParameters.Request(names=['task_parameter_owner']))).values[0].string_value
                if not owner:
                    self.lease_maybe_owned = False
                    return True
                if owner != self.id:
                    raise RuntimeError('planner ownership changed unexpectedly')
                response = self._await(self.node._planner_set.call_async(
                    SetParametersAtomically.Request(parameters=[
                        Parameter('task_parameter_owner', value='').to_parameter_msg(),
                        Parameter('task_parameter_release', value=self.id).to_parameter_msg()])))
                if not response.result.successful:
                    raise RuntimeError(response.result.reason)
                self.lease_maybe_owned = False
                return True
            except Exception as exc:
                self.emit('stopping', error=f'planner ownership uncertain: {exc}')
                time.sleep(0.1)
        return False

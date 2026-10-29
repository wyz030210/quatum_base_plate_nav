"""Correlate immutable preparation, the existing outer action, and bridge telemetry."""
import copy
import json
import uuid

from nav2_mission_interfaces.srv import PrepareCoverage
from opennav_coverage_msgs.action import NavigateCompleteCoverage
from rclpy.action import ActionClient
from std_msgs.msg import String
from unique_identifier_msgs.msg import UUID

from .task_adapters import ExecutionUpdate
from .task_ros_adapters import RosNavigationOperation, decode_json


class RosCoverage:
    def __init__(self, node, action, preparation, state_topic):
        self.client = ActionClient(node, NavigateCompleteCoverage, action)
        self.preparation = node.create_client(PrepareCoverage, preparation)
        self.operations = {}
        self.subscription = node.create_subscription(String, state_topic, self.receive, 10)

    def receive(self, message):
        try:
            state = decode_json(message.data)
            operation = self.operations.get(state['goal_id'])
            if operation is not None and state.get('task_id') == operation.task_id:
                operation.state = state
        except (ValueError, KeyError, TypeError):
            pass

    def start(self, task_id, config):
        if not self.client.server_is_ready() or not self.preparation.service_is_ready():
            raise RuntimeError('coverage bridge unavailable')
        operation = CoverageOperation(self, task_id, config)
        self.operations[operation.goal_id] = operation
        return operation


class CoverageOperation:
    def __init__(self, adapter, task_id, config):
        self.adapter, self.task_id = adapter, task_id
        self.goal_id = uuid.uuid4().hex
        self.config = copy.deepcopy(config)
        self.state = {'phase': 'preparing', 'goal_id': self.goal_id, 'task_id': task_id}
        self.prepared = adapter.preparation.call_async(PrepareCoverage.Request(
            goal_id=self.goal_id, task_id=task_id, config_json=json.dumps(self.config)))
        self.motion = None
        self.discard = None
        self.canceled = False
        self.terminal = None
        self.dispatch_attempted = False
        self.dispatch_error = ''

    def cancel(self):
        self.canceled = True
        if self.motion:
            self.motion.cancel()

    def poll(self, now):
        if self.terminal:
            return self.terminal
        details = copy.deepcopy(self.state)
        if self.dispatch_error:
            return ExecutionUpdate('finalizing', error=self.dispatch_error, details=details)
        try:
            if not self.prepared.done():
                return ExecutionUpdate('preparing', details=details)
            response = self.prepared.result()
            if not response.accepted:
                self.terminal = ExecutionUpdate('preparing', True, False,
                                                response.message, details)
            elif self.motion is None:
                if self.canceled:
                    if self.discard is None:
                        self.discard = self.adapter.preparation.call_async(
                            PrepareCoverage.Request(goal_id=self.goal_id, discard=True))
                    if self.discard.done() and self.discard.result().accepted:
                        self.terminal = ExecutionUpdate('stopping', True, False,
                                                       'coverage canceled before dispatch',
                                                       details)
                else:
                    self.state = decode_json(response.state_json)
                    self.dispatch_attempted = True
                    try:
                        self.motion = RosNavigationOperation(self.adapter.client.send_goal_async(
                            NavigateCompleteCoverage.Goal(frame_id=self.config['frame_id']),
                            goal_uuid=UUID(uuid=list(bytes.fromhex(self.goal_id)))))
                    except Exception as exc:
                        self.dispatch_error = f'outer coverage submission uncertain: {exc}'
                        raise
            if self.motion is not None:
                update = self.motion.poll(now)
                if update.done:
                    # Action result is only delivered after bridge-owned children
                    # terminate and its planner lease is released.
                    wrapped = self.motion.result.result() if self.motion.result else None
                    code = int(wrapped.result.error_code) if wrapped is not None else None
                    succeeded = update.succeeded and code == 0
                    self.terminal = ExecutionUpdate(
                        self.state.get('phase', 'stopping'), True, succeeded,
                        '' if succeeded else f'coverage error {code}: {update.error}',
                        copy.deepcopy(self.state))
                elif update.error:
                    return ExecutionUpdate('stopping', error=update.error, details=details)
            if self.terminal:
                self.adapter.operations.pop(self.goal_id, None)
                return self.terminal
            bridge_error = self.state.get('error', '')
            if self.state.get('execution_failed'):
                bridge_error = (f"coverage bridge error {self.state.get('error_code')}: "
                                + bridge_error)
            if bridge_error:
                return ExecutionUpdate('finalizing', error=bridge_error, details=details)
            phase = self.state.get('phase', 'preparing')
            return ExecutionUpdate('finalizing' if phase == 'stopping' else phase,
                                   details=details)
        except Exception as exc:
            return ExecutionUpdate('stopping', error=f'coverage ownership uncertain: {exc}',
                                   details=details)

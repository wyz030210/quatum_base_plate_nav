"""Sequential, cancelable inspection through navigation and arrival boundaries."""
import copy
from datetime import datetime, timezone

from .inspection_config import prepare_inspection
from .task_adapters import ExecutionUpdate


_MESSAGES = {
    'security': '模拟安防巡检正常',
    'facility': '模拟拍照及设备状态检查正常',
    'environment': '模拟环境卫生检查正常',
    'dock': '到达回充点，任务收尾',
}


class SimulatedActions:
    """Replaceable arrival-action boundary; only simulation is provided initially."""

    def start(self, task_id, point, now):
        return SimulatedAction(point, now)


class SimulatedAction:
    def __init__(self, point, now):
        self.point = point
        self.deadline = now + point['stay_time']
        self.canceled = False

    def cancel(self):
        self.canceled = True

    def poll(self, now):
        if self.canceled:
            return ExecutionUpdate('inspecting', True, False, 'canceled')
        if now < self.deadline:
            return ExecutionUpdate('inspecting')
        return ExecutionUpdate('inspecting', True, True,
                               details={'message': _MESSAGES[self.point['type']]})


class Inspection:
    def __init__(self, navigation, actions=None):
        self.navigation = navigation
        self.actions = actions if actions is not None else SimulatedActions()

    def prepare(self, request):
        return prepare_inspection(request)

    def initial_details(self, config):
        return dict(points=[], succeeded=0, failed=0, skipped=0, canceled=0,
                    total_points=len(config['points']), unvisited=len(config['points']))

    def begin(self, task_id, config, now):
        return InspectionOperation(self.navigation, self.actions, task_id, config)


class InspectionOperation:
    def __init__(self, navigation, actions, task_id, config):
        self.navigation = navigation
        self.actions = actions
        self.task_id = task_id
        self.config = config
        self.index = 0
        self.motion = None
        self.action = None
        self.records = []
        self.started_at = None
        self.started_utc = None
        self.canceled = False
        self.finished = None

    def cancel(self):
        self.canceled = True
        if self.action is not None:
            self.action.cancel()
        elif self.motion is not None:
            self.motion.cancel()

    def _details(self):
        return dict(points=copy.deepcopy(self.records),
                    succeeded=sum(p['status'] == 'succeeded' for p in self.records),
                    failed=sum(p['status'] == 'failed' for p in self.records),
                    skipped=sum(p['status'] == 'skipped' for p in self.records),
                    canceled=sum(p['status'] == 'canceled' for p in self.records),
                    unvisited=len(self.config['points']) - len(self.records),
                    total_points=len(self.config['points']))

    def _record(self, now, navigation_result, inspection_result, status, message, error=''):
        point = self.config['points'][self.index]
        self.records.append(dict(
            task_id=self.task_id, mission_id=self.config['mission_id'],
            mission_name=self.config['mission_name'], id=point['id'], point_id=point['id'],
            point_name=point['name'], point_type=point['type'], allow_skip=point['allow_skip'],
            start_time=self.started_utc, end_time=datetime.now(timezone.utc).isoformat(),
            duration_sec=max(0, now - self.started_at), navigation_result=navigation_result,
            inspection_result=inspection_result, status=status, message=message, error=error,
            simulated=True, charging_verified=False))

    def _point_finished(self, now, navigation_result, inspection_result, succeeded, message,
                        error=''):
        point = self.config['points'][self.index]
        status = 'succeeded' if succeeded else 'skipped' if point['allow_skip'] else 'failed'
        self._record(now, navigation_result, inspection_result, status, message, error)
        phase = 'navigating' if inspection_result == 'not_run' else 'inspecting'
        if status == 'failed' or self.index == len(self.config['points']) - 1:
            self.finished = ExecutionUpdate(phase, True, status != 'failed',
                                            (error or message) if status == 'failed' else '',
                                            self._details())
            return self.finished
        self.index += 1
        self.motion = self.action = None
        return ExecutionUpdate('navigating', details=self._details())

    def _canceled(self, now, navigation_result, inspection_result, message='canceled'):
        if self.motion is not None:
            self._record(now, navigation_result, inspection_result, 'canceled', message, message)
        self.finished = ExecutionUpdate(
            'inspecting' if self.action is not None else 'navigating',
            True, False, message, self._details())
        return self.finished

    def poll(self, now):
        if self.finished is not None:
            return self.finished
        point = self.config['points'][self.index]
        if self.motion is None:
            if self.canceled:
                return self._canceled(now, 'not_run', 'not_run')
            self.started_at = now
            self.started_utc = datetime.now(timezone.utc).isoformat()
            self.motion = self.navigation.start(self.task_id, point, self.config['frame_id'])
        update = self.motion.poll(now)
        if not update.done:
            return ExecutionUpdate('navigating', error=update.error, details=self._details())
        if self.canceled and self.action is None:
            return self._canceled(now, 'success' if update.succeeded else 'canceled',
                                  'not_run', update.error or 'canceled')
        if not update.succeeded:
            return self._point_finished(now, 'failed', 'not_run', False,
                                        update.error or 'navigation failed', update.error)
        if self.action is None:
            self.action = self.actions.start(self.task_id, point, now)
        update = self.action.poll(now)
        if not update.done:
            return ExecutionUpdate('inspecting', error=update.error, details=self._details())
        if self.canceled:
            return self._canceled(now, 'success', 'canceled', update.error or 'canceled')
        return self._point_finished(now, 'success', 'success' if update.succeeded else 'failed',
                                    update.succeeded, update.details.get('message', update.error),
                                    update.error)

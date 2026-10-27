"""Cancelable simulated single-point inspection using the target Nav2 boundary."""
import copy
import math

from .task_adapters import ExecutionUpdate


class SinglePointInspection:
    def __init__(self, navigation):
        self.navigation = navigation

    def prepare(self, request):
        config = copy.deepcopy(request)
        if not isinstance(config, dict) or set(config) - {'frame_id', 'point'}:
            raise ValueError('unsupported inspection options')
        if not isinstance(config.get('frame_id'), str) or not config['frame_id']:
            raise ValueError('frame_id is required')
        point = config.get('point')
        if not isinstance(point, dict):
            raise ValueError('point is required')
        if set(point) - {'id', 'type', 'x', 'y', 'yaw', 'stay_time', 'allow_skip'}:
            raise ValueError('unsupported point options')
        if not isinstance(point.get('id'), str) or not point['id']:
            raise ValueError('point id is required')
        if point.get('type') not in {'security', 'facility', 'environment', 'dock'}:
            raise ValueError('unsupported point type')
        for key in ('x', 'y', 'yaw', 'stay_time'):
            value = point.get(key)
            if (isinstance(value, bool) or not isinstance(value, (float, int)) or
                    not math.isfinite(value)):
                raise ValueError(f'{key} must be a finite number')
        if point['stay_time'] < 0:
            raise ValueError('stay_time must be nonnegative')
        if type(point.get('allow_skip', False)) is not bool:
            raise ValueError('allow_skip must be boolean')
        if point.get('allow_skip', False):
            raise ValueError('allow_skip=true requires the multi-point inspection handler')
        return config

    def begin(self, task_id, config, now):
        return InspectionOperation(self.navigation.start(
            task_id, config['point'], config['frame_id']), config['point'])


class InspectionOperation:
    def __init__(self, motion, point):
        self.motion = motion
        self.point = point
        self.dwell_until = None
        self.canceled = False
        self.finished = None

    def cancel(self):
        self.canceled = True
        if self.dwell_until is None:
            self.motion.cancel()

    def poll(self, now):
        if self.finished:
            return self.finished
        if self.dwell_until is None:
            update = self.motion.poll(now)
            if not update.done:
                return update
            if self.canceled or not update.succeeded:
                self.finished = ExecutionUpdate('navigating', True, False,
                                                update.error or 'canceled')
                return self.finished
            self.dwell_until = now + self.point['stay_time']
        if self.canceled:
            self.finished = ExecutionUpdate('inspecting', True, False, 'canceled')
        elif now >= self.dwell_until:
            self.finished = ExecutionUpdate('inspecting', True, True, details={
                'points': [{'id': self.point['id'], 'status': 'succeeded',
                            'simulated': True,
                            'message': 'simulated inspection; docking/charging not verified'}],
                'succeeded': 1, 'failed': 0, 'skipped': 0,
            })
        return self.finished or ExecutionUpdate('inspecting')

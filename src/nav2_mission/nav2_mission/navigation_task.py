"""Ordinary pose navigation with the manager's authorization and stop lifecycle."""
from .coverage_task import finite


class NavigationTask:
    def __init__(self, navigation):
        self.navigation = navigation

    def prepare(self, request):
        if not isinstance(request, dict) or set(request) - {'frame_id', 'point'}:
            raise ValueError('navigation accepts frame_id and point only')
        frame = request.get('frame_id', 'map')
        point = request.get('point')
        if not isinstance(frame, str) or not frame.strip():
            raise ValueError('navigation frame_id must be nonempty')
        if not isinstance(point, dict) or set(point) != {'x', 'y', 'yaw'}:
            raise ValueError('navigation point requires x, y and yaw')
        if any(not finite(value) for value in point.values()):
            raise ValueError('navigation coordinates must be finite numbers')
        return {'frame_id': frame, 'point': dict(point)}

    def begin(self, task_id, config, now):
        return self.navigation.start(task_id, config['point'], config['frame_id'])

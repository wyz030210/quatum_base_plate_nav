"""Session editor with atomic admission through the existing task manager."""
import copy
import math

from .coverage.models import Point2D
from .coverage.region_editor import RegionEditor, valid_coordinate


class CoverageSession:
    """Call only from the manager's serial callback group."""

    def __init__(self, manager):
        self.manager = manager
        self.editor = RegionEditor()
        self.frame_id = 'map'
        self.planner = {'operation_width': 0.60, 'default_swath_angle': 1.57}

    def request(self):
        snapshot = self.editor.snapshot(self.planner['operation_width'])
        return {'frame_id': self.frame_id, 'polygons': [
            [[p.x, p.y] for p in ring] for ring in (snapshot.outer, *snapshot.holes)],
            'planner': copy.deepcopy(self.planner)}

    def status(self):
        try:
            self.request()
            valid, error = True, ''
        except ValueError as exc:
            valid, error = False, str(exc)
        return {'frame_id': self.frame_id, 'mode': self.editor.mode.value,
                'polygons': [[[p.x, p.y] for p in ring] for ring in (
                    self.editor.outer, *self.editor.holes)],
                'draft_hole': [[p.x, p.y] for p in self.editor.draft_hole],
                'planner': copy.deepcopy(self.planner), 'valid': valid, 'error': error,
                'active': self.manager.active}

    def command(self, operation, payload):
        try:
            if not isinstance(payload, dict):
                raise ValueError('expected JSON object')
            if operation == 'status':
                return self.reply(True, 'session state')
            if self.manager.active:
                raise ValueError('task active or stop unconfirmed; editing/start refused')
            if operation == 'start':
                if payload:
                    raise ValueError('start uses confirmed session configuration; configure first')
                reply = self.manager.start('coverage', self.request())
                return dict(reply, state=self.status())
            if operation == 'configure':
                self.configure(payload)
            elif operation == 'point':
                if set(payload) - {'x', 'y', 'frame_id'}:
                    raise ValueError('unknown point field')
                if payload.get('frame_id', self.frame_id) != self.frame_id:
                    raise ValueError('point frame does not match task frame')
                self.editor.add_point(Point2D(payload['x'], payload['y']))
            else:
                if payload:
                    raise ValueError('operation does not accept parameters')
                simple = {'clear': self.editor.clear, 'undo': self.editor.undo,
                          'begin_hole': self.editor.begin_hole}
                close = {'close_outer': self.editor.close_outer,
                         'close_hole': self.editor.close_hole}
                if operation in simple:
                    simple[operation]()
                elif operation in close:
                    close[operation](self.planner['operation_width'])
                else:
                    raise ValueError('unknown region operation')
            return self.reply(True, operation + ' accepted; inspect state before start')
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            return self.reply(False, str(exc))

    def configure(self, payload):
        if not payload or set(payload) - {'frame_id', 'operation_width', 'default_swath_angle'}:
            raise ValueError('configure accepts frame_id, operation_width, default_swath_angle')
        frame = payload.get('frame_id', self.frame_id)
        if not isinstance(frame, str) or not frame.strip():
            raise ValueError('frame_id must be nonempty')
        if frame != self.frame_id and (self.editor.outer or self.editor.draft_hole):
            raise ValueError('clear region before changing frame')
        planner = dict(self.planner)
        for name in planner:
            if name in payload:
                value = payload[name]
                if not valid_coordinate(value):
                    raise ValueError(name + ' must be a bounded finite number')
                if name == 'operation_width' and value <= 0:
                    raise ValueError('operation_width must be positive')
                if name == 'default_swath_angle' and not -math.pi <= value <= math.pi:
                    raise ValueError('default_swath_angle must be radians within [-pi, pi]')
                planner[name] = float(value)
        # Reject infeasible new width before changing the confirmed configuration.
        if self.editor.mode.value == 'OUTER_CLOSED':
            self.editor.snapshot(planner['operation_width'])
        self.frame_id, self.planner = frame, planner

    def reply(self, accepted, message):
        return {'accepted': accepted, 'task_id': '', 'message': message, 'state': self.status()}

"""Informational source geometry metrics on the common localization provider."""
import copy
import math
import time
from dataclasses import replace

from .coverage.metrics import (DistanceAccumulator, covered_segments_area_ratio,
                               path_completion, polyline_length)
from .coverage.models import Point2D, RegionSnapshot
from .coverage.region_editor import valid_coordinate


class ObservedCoverage:
    """Decorate public handler/operation interfaces, without taking motion ownership."""

    def __init__(self, handler, *, clock=time.monotonic, max_age=1.0):
        self.handler, self.clock, self.max_age = handler, clock, max_age
        self.operation = None

    def prepare(self, request):
        return self.handler.prepare(request)

    def begin(self, task_id, config, now):
        self.operation = ReferenceOperation(self.handler.begin(task_id, config, now),
                                            config, self.max_age, now)
        return self.operation

    def observe(self, data, ros_now):
        if self.operation is not None and not self.operation.done:
            self.operation.observe(data, ros_now, self.clock())


class ReferenceOperation:
    def __init__(self, owned, config, max_age, now):
        self.owned, self.config, self.max_age = owned, config, max_age
        self.region = RegionSnapshot(
            tuple(Point2D(*p) for p in config['polygons'][0]),
            tuple(tuple(Point2D(*p) for p in ring) for ring in config['polygons'][1:]))
        self.width = config['planner'].get('operation_width', 0.60)
        self.distance = DistanceAccumulator()
        self.trace, self.coverage_segments = [], []
        self.current_segment = None
        self.phase = 'preparing'
        self.last_received = now
        self.last_stamp = None
        self.last_identity = None
        self.gaps = 0
        self.in_gap = True
        self.done = False
        self.completion = None
        self.area = 0.0
        self.dirty = False

    def cancel(self):
        self.owned.cancel()

    def gap(self):
        if not self.in_gap:
            self.gaps += 1
        self.in_gap = True
        self.current_segment = None
        self.distance.reset_segment()

    def observe(self, data, ros_now, now):
        if self.phase not in ('approaching', 'following'):
            return
        try:
            source_stamp = data['source_stamp_sec']
            stamp = data.get('pose_stamp_sec', source_stamp)
            if (data['valid'] is not True or data['frame_id'] != self.config['frame_id']
                    or not all(valid_coordinate(data[k]) for k in ('x', 'y', 'yaw'))
                    or any(isinstance(value, bool) or not isinstance(value, (float, int))
                           or not math.isfinite(value) or not 0 <= ros_now - value <= self.max_age
                           for value in (source_stamp, stamp))):
                raise ValueError('invalid pose evidence')
            if self.last_stamp is not None and stamp <= self.last_stamp:
                if stamp < self.last_stamp:
                    self.gap()
                return
            identity = tuple(data.get(k, '') for k in (
                'source', 'localization_profile', 'map_id', 'evidence_generation'))
            if self.last_identity is not None and identity != self.last_identity:
                self.gap()
            self.last_identity, self.last_stamp, self.last_received = identity, stamp, now
            point = Point2D(float(data['x']), float(data['y']))
            if self.in_gap:
                self.trace.append([])
                self.in_gap = False
            self.trace[-1].append(point)
            self.distance.add(point.x, point.y)
            if self.phase == 'following':
                if self.current_segment is None:
                    self.current_segment = []
                    self.coverage_segments.append(self.current_segment)
                self.current_segment.append(point)
                self.dirty = True
        except (TypeError, ValueError, KeyError, OverflowError):
            self.gap()

    def poll(self, now):
        update = self.owned.poll(now)
        if update.phase != self.phase:
            # Approach and coverage are separate evidence segments.
            self.current_segment = None
            self.distance.reset_segment()
        if self.phase in ('approaching', 'following') and now - self.last_received > self.max_age:
            self.gap()
        self.phase = update.phase
        self.done = update.done
        details = copy.deepcopy(update.details)
        planned = details.get('planned_path') or {}
        length = polyline_length((p['x'], p['y']) for p in planned.get('poses', []))
        remaining = details.get('feedback', {}).get('distance_remaining')
        if self.phase == 'following' and isinstance(remaining, (int, float)):
            self.completion = path_completion(length, remaining) if length else None
        if self.dirty:
            self.area = covered_segments_area_ratio(
                self.coverage_segments, self.region, self.width)
            self.dirty = False
        details['reference_statistics'] = {
            'planned_path_length_m': length, 'path_completion_ratio': self.completion,
            'trace_estimated_area_ratio': self.area,
            'trace_distance_m': self.distance.distance,
            'trace_gap_count': self.gaps, 'trace_sample_count': sum(map(len, self.trace)),
            'trace_complete': self.gaps == 0 and bool(self.coverage_segments),
            'collision_count': None,
            'basis': ('geometric estimate from task-frame pose evidence; '
                      'not measured cleaning quality'),
        }
        details['executed_trace'] = {
            'frame_id': self.config['frame_id'],
            'segments': [[[p.x, p.y] for p in segment] for segment in self.trace]}
        return replace(update, details=details)

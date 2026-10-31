"""Geometry and odometry metrics for coverage reports."""

import math
from typing import Iterable, Sequence, Tuple

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

from .models import Point2D, RegionSnapshot
from .region_editor import validate_region


def polyline_length(points: Iterable[Tuple[float, float]]) -> float:
    sequence = list(points)
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(sequence, sequence[1:])
    )


class DistanceAccumulator:
    def __init__(self, jitter_threshold: float = 0.01) -> None:
        self.jitter_threshold = jitter_threshold
        self.distance = 0.0
        self._last = None

    def reset_segment(self) -> None:
        self._last = None

    def add(self, x: float, y: float) -> float:
        current = (float(x), float(y))
        if self._last is None:
            self._last = current
            return self.distance
        increment = math.hypot(
            current[0] - self._last[0], current[1] - self._last[1]
        )
        # Keep the last accepted sample.  Updating it for every sub-threshold
        # sample would erase genuine low-speed motion sampled at high rate.
        if increment >= self.jitter_threshold:
            self.distance += increment
            self._last = current
        return self.distance


def covered_area_ratio(
    trace: Sequence[Point2D],
    region: RegionSnapshot,
    operation_width: float,
) -> float:
    target: Polygon = validate_region(region, operation_width)
    if not trace:
        return 0.0
    coordinates = [(point.x, point.y) for point in trace]
    swept = (
        Point(coordinates[0]).buffer(operation_width / 2.0)
        if len(coordinates) == 1
        else LineString(coordinates).buffer(
            operation_width / 2.0, cap_style=1, join_style=1
        )
    )
    return max(0.0, min(1.0, swept.intersection(target).area / target.area))


def path_completion(total_length: float, distance_to_goal: float) -> float:
    if total_length <= 0.0:
        return 0.0
    return max(0.0, min(1.0, (total_length - distance_to_goal) / total_length))


def covered_segments_area_ratio(segments, region, operation_width):
    """Same source sweep estimate, without drawing lines across missing evidence."""
    target = validate_region(region, operation_width)
    sweeps = []
    for segment in segments:
        coordinates = [(point.x, point.y) for point in segment]
        if coordinates:
            geometry = Point(coordinates[0]) if len(coordinates) == 1 else LineString(coordinates)
            sweeps.append(geometry.buffer(operation_width / 2.0, cap_style=1, join_style=1))
    if not sweeps:
        return 0.0
    return max(0.0, min(1.0, unary_union(sweeps).intersection(target).area / target.area))

"""Session-only polygon editor and geometry validation."""

import math
from typing import List, Sequence, Tuple

from shapely.geometry import LinearRing, Polygon

from .models import EditMode, Point2D, RegionSnapshot


_EPSILON = 1.0e-6
MAX_COORDINATE = 1.0e7  # Metres in a local task frame; keep GEOS arithmetic bounded.


def valid_coordinate(value):
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and abs(value) <= MAX_COORDINATE and math.isfinite(value))


def _different_points(points: Sequence[Point2D]) -> List[Point2D]:
    cleaned: List[Point2D] = []
    for point in points:
        if not (valid_coordinate(point.x) and valid_coordinate(point.y)):
            raise ValueError("point coordinates must be finite numbers within +/-10000000 m")
        if not cleaned or math.hypot(
            point.x - cleaned[-1].x, point.y - cleaned[-1].y
        ) > _EPSILON:
            cleaned.append(point)
    if len(cleaned) > 1 and math.hypot(
        cleaned[0].x - cleaned[-1].x,
        cleaned[0].y - cleaned[-1].y,
    ) <= _EPSILON:
        cleaned.pop()
    return cleaned


def _normalize_ring(
    points: Sequence[Point2D], *, counterclockwise: bool
) -> Tuple[Point2D, ...]:
    cleaned = _different_points(points)
    if len(cleaned) < 3:
        raise ValueError("a ring needs at least three distinct points")
    ring = LinearRing([(point.x, point.y) for point in cleaned])
    if not ring.is_simple:
        raise ValueError("ring self-intersects")
    if ring.is_ccw != counterclockwise:
        cleaned.reverse()
    return tuple(cleaned)


def validate_region(
    snapshot: RegionSnapshot, operation_width: float = 0.5
) -> Polygon:
    if operation_width is not None and (
        not valid_coordinate(operation_width) or operation_width <= 0.0
    ):
        raise ValueError("operation_width must be positive")
    outer = _normalize_ring(snapshot.outer, counterclockwise=True)
    holes = tuple(
        _normalize_ring(hole, counterclockwise=False)
        for hole in snapshot.holes
    )
    shell = Polygon([(point.x, point.y) for point in outer])
    exclusions = [Polygon([(point.x, point.y) for point in hole]) for hole in holes]
    for index, hole in enumerate(exclusions):
        if (not shell.contains(hole) or shell.boundary.intersects(hole)
                or any(hole.intersects(other) for other in exclusions[:index])):
            raise ValueError("outer boundary or holes form an invalid polygon: "
                             "holes must be strictly inside and disjoint without contact")
    polygon = Polygon(
        [(point.x, point.y) for point in outer],
        [[(point.x, point.y) for point in hole] for hole in holes],
    )
    if not polygon.is_valid:
        raise ValueError("outer boundary or holes form an invalid polygon")
    if polygon.area <= _EPSILON:
        raise ValueError("region has no usable area")
    # An erosion is only a feasibility warning/check; the original polygon is
    # never altered or used as a replacement for the operator's boundary.
    if operation_width is not None and polygon.buffer(-operation_width / 2.0).is_empty:
        raise ValueError("region is too narrow for the operation width")
    return polygon


class RegionEditor:
    """Pure state machine behind the RViz clicked-point editor."""

    def __init__(self) -> None:
        self.outer: List[Point2D] = []
        self.holes: List[Tuple[Point2D, ...]] = []
        self.draft_hole: List[Point2D] = []
        self.mode = EditMode.OUTER_DRAFT

    def add_point(self, point: Point2D) -> None:
        if not (valid_coordinate(point.x) and valid_coordinate(point.y)):
            raise ValueError("point coordinates must be finite numbers within +/-10000000 m")
        target = self.draft_hole if self.mode == EditMode.HOLE_DRAFT else self.outer
        if self.mode == EditMode.OUTER_CLOSED:
            raise ValueError("begin a hole before adding more points")
        if target and math.hypot(point.x - target[-1].x, point.y - target[-1].y) <= _EPSILON:
            return
        target.append(point)

    def undo(self) -> None:
        if self.mode == EditMode.OUTER_CLOSED:
            raise ValueError("there is no draft point to undo; begin a hole or clear")
        target = self.draft_hole if self.mode == EditMode.HOLE_DRAFT else self.outer
        if not target:
            raise ValueError("there is no draft point to undo")
        target.pop()

    def clear(self) -> None:
        self.outer.clear()
        self.holes.clear()
        self.draft_hole.clear()
        self.mode = EditMode.OUTER_DRAFT

    def close_outer(self, operation_width: float = 0.5) -> None:
        if self.mode != EditMode.OUTER_DRAFT:
            raise ValueError("outer boundary is already closed")
        normalized = _normalize_ring(self.outer, counterclockwise=True)
        validate_region(RegionSnapshot(normalized, tuple()), operation_width)
        self.outer = list(normalized)
        self.mode = EditMode.OUTER_CLOSED

    def begin_hole(self) -> None:
        if self.mode != EditMode.OUTER_CLOSED:
            raise ValueError("close the outer boundary before starting a hole")
        self.draft_hole.clear()
        self.mode = EditMode.HOLE_DRAFT

    def close_hole(self, operation_width: float = 0.5) -> None:
        if self.mode != EditMode.HOLE_DRAFT:
            raise ValueError("no hole is being edited")
        hole = _normalize_ring(self.draft_hole, counterclockwise=False)
        candidate = RegionSnapshot(tuple(self.outer), tuple(self.holes) + (hole,))
        validate_region(candidate, operation_width)
        self.holes.append(hole)
        self.draft_hole.clear()
        self.mode = EditMode.OUTER_CLOSED

    def snapshot(self, operation_width: float = 0.5) -> RegionSnapshot:
        if self.mode != EditMode.OUTER_CLOSED:
            raise ValueError("finish the current boundary before starting")
        snapshot = RegionSnapshot(tuple(self.outer), tuple(self.holes))
        validate_region(snapshot, operation_width)
        return snapshot

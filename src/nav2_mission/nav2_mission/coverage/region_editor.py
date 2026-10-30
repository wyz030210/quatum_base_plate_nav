"""Session-only polygon editor and geometry validation."""

import math
from typing import List, Sequence, Tuple

from shapely.geometry import LineString, LinearRing, Point, Polygon
from shapely.ops import nearest_points

from .models import EditMode, Point2D, RegionSnapshot


_EPSILON = 1.0e-6
MAX_GEOMETRY_SAMPLES = 4096
MAX_HOLES = 64
MAX_GEOMETRY_WORK = 1000000
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


def _has_narrow_sampled_passage(polygon: Polygon, width: float) -> bool:
    """Probe inward normals at a finite scale; exempt an adjacent convex corner."""
    # Removing redundant collinear vertices preserves the exact polygon while
    # ensuring a straight side split into segments still has one corner.
    polygon = polygon.simplify(0, preserve_topology=True)
    rings = [list(ring.coords) for ring in (polygon.exterior, *polygon.interiors)]
    boundary = polygon.boundary
    shell = Polygon(polygon.exterior)
    convex_shell = shell.equals(shell.convex_hull)
    spacing = width / 8.0
    if spacing <= _EPSILON:
        raise ValueError("region too complex for operation width validation")
    sample_count = sum(
        max(1, math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / spacing))
        for ring in rings for a, b in zip(ring, ring[1:]))
    segment_count = sum(len(ring) - 1 for ring in rings)
    if (sample_count > MAX_GEOMETRY_SAMPLES or
            sample_count * segment_count > MAX_GEOMETRY_WORK):
        raise ValueError("region too complex for operation width validation")
    if convex_shell and not polygon.interiors:
        return False  # Every hit would be an exempt convex-shell hit.
    tolerance = max(_EPSILON, width * 1e-7)
    for ring_index, ring in enumerate(rings):
        count = len(ring) - 1
        for index in range(count):
            a, b = ring[index], ring[index + 1]
            dx, dy = b[0] - a[0], b[1] - a[1]
            length = math.hypot(dx, dy)
            if length <= tolerance:
                continue
            ux, uy = dx / length, dy / length
            nx, ny = -uy, ux
            samples = max(1, math.ceil(length / spacing))
            for sample in range(samples):
                t = length * (sample + 0.5) / samples
                x, y = a[0] + ux * t, a[1] + uy * t
                ray = LineString([(x + nx * tolerance, y + ny * tolerance),
                                  (x + nx * (width - tolerance),
                                   y + ny * (width - tolerance))])
                hit = ray.intersection(boundary)
                if hit.is_empty:
                    continue
                end = nearest_points(Point(x, y), hit)[1]
                # A convex shell can taper to an ordinary sharp or rounded
                # corner. The erosion check still rejects an entirely narrow
                # convex region, and hits on holes are still checked below.
                if (ring_index == 0 and convex_shell and
                        polygon.exterior.distance(end) <= tolerance):
                    continue
                # Adjacent edges meeting at a convex vertex form an ordinary
                # corner, even when the local gap near its point is tiny.
                adjacent_corner = False
                adjacent = ((index, (index - 1) % count),
                            (index + 1, (index + 1) % count))
                for vertex_index, other_index in adjacent:
                    before = ring[(vertex_index - 1) % count]
                    vertex = ring[vertex_index % count]
                    after = ring[(vertex_index + 1) % count]
                    cross = ((vertex[0] - before[0]) * (after[1] - vertex[1]) -
                             (vertex[1] - before[1]) * (after[0] - vertex[0]))
                    if cross > 0 and LineString([
                            ring[other_index], ring[other_index + 1]]).distance(end) <= tolerance:
                        adjacent_corner = True
                        break
                if not adjacent_corner:
                    return True
    return False


def validate_region(
    snapshot: RegionSnapshot, operation_width: float = 0.5
) -> Polygon:
    if operation_width is not None and (
        not valid_coordinate(operation_width) or operation_width <= 0.0
    ):
        raise ValueError("operation_width must be positive")
    if (len(snapshot.holes) > MAX_HOLES or
            sum(map(len, (snapshot.outer, *snapshot.holes))) > MAX_GEOMETRY_SAMPLES):
        raise ValueError("region too complex for operation width validation")
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
    if operation_width is not None and (
        polygon.buffer(-operation_width / 2.0).is_empty
        or _has_narrow_sampled_passage(polygon, operation_width)
    ):
        raise ValueError("region is too narrow for the operation width")
    return polygon


class RegionEditor:
    """Pure state machine behind the RViz clicked-point editor."""

    def __init__(self) -> None:
        self.outer: List[Point2D] = []
        self.holes: List[Tuple[Point2D, ...]] = []
        self.draft_hole: List[Point2D] = []
        self.mode = EditMode.OUTER_DRAFT
        self._outer_input: List[Point2D] = []
        self._hole_inputs: List[List[Point2D]] = []

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
            if self.holes:
                self.holes.pop()
                self.draft_hole = self._hole_inputs.pop()
                self.mode = EditMode.HOLE_DRAFT
            else:
                self.outer = self._outer_input.copy()
                self.mode = EditMode.OUTER_DRAFT
            return
        target = self.draft_hole if self.mode == EditMode.HOLE_DRAFT else self.outer
        if not target:
            if self.mode == EditMode.HOLE_DRAFT:
                self.mode = EditMode.OUTER_CLOSED
                return
            raise ValueError("there is no draft point to undo")
        target.pop()

    def clear(self) -> None:
        self.outer.clear()
        self.holes.clear()
        self.draft_hole.clear()
        self._outer_input.clear()
        self._hole_inputs.clear()
        self.mode = EditMode.OUTER_DRAFT

    def close_outer(self, operation_width: float = 0.5) -> None:
        if self.mode != EditMode.OUTER_DRAFT:
            raise ValueError("outer boundary is already closed")
        normalized = _normalize_ring(self.outer, counterclockwise=True)
        validate_region(RegionSnapshot(normalized, tuple()), operation_width)
        self._outer_input = self.outer.copy()
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
        self._hole_inputs.append(self.draft_hole.copy())
        self.holes.append(hole)
        self.draft_hole.clear()
        self.mode = EditMode.OUTER_CLOSED

    def snapshot(self, operation_width: float = 0.5) -> RegionSnapshot:
        if self.mode != EditMode.OUTER_CLOSED:
            raise ValueError("finish the current boundary before starting")
        snapshot = RegionSnapshot(tuple(self.outer), tuple(self.holes))
        validate_region(snapshot, operation_width)
        return snapshot

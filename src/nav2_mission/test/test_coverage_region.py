import pytest

from nav2_mission.coverage.models import EditMode, Point2D, RegionSnapshot
from nav2_mission.coverage.region_editor import RegionEditor, validate_region


def _square(x0=0.0, y0=0.0, size=4.0):
    return (
        Point2D(x0, y0),
        Point2D(x0 + size, y0),
        Point2D(x0 + size, y0 + size),
        Point2D(x0, y0 + size),
    )


def test_editor_closes_outer_and_hole_with_normalized_orientation():
    editor = RegionEditor()
    for point in _square():
        editor.add_point(point)
    editor.close_outer(0.5)
    assert editor.mode == EditMode.OUTER_CLOSED
    editor.begin_hole()
    for point in _square(1.0, 1.0, 1.0):
        editor.add_point(point)
    editor.close_hole(0.5)
    snapshot = editor.snapshot(0.5)
    assert len(snapshot.holes) == 1
    assert validate_region(snapshot, 0.5).area == pytest.approx(15.0)


def test_self_intersection_is_rejected():
    bow_tie = RegionSnapshot(
        (
            Point2D(0.0, 0.0),
            Point2D(2.0, 2.0),
            Point2D(0.0, 2.0),
            Point2D(2.0, 0.0),
        ),
        tuple(),
    )
    with pytest.raises(ValueError, match="self-intersects"):
        validate_region(bow_tie)


def test_hole_outside_outer_is_rejected():
    region = RegionSnapshot(_square(), (_square(3.5, 3.5, 1.0),))
    with pytest.raises(ValueError, match="invalid polygon"):
        validate_region(region)


def test_clear_and_undo_only_change_draft():
    editor = RegionEditor()
    editor.add_point(Point2D(0.0, 0.0))
    editor.add_point(Point2D(1.0, 0.0))
    editor.undo()
    assert len(editor.outer) == 1
    editor.clear()
    assert editor.mode == EditMode.OUTER_DRAFT
    assert not editor.outer


def test_region_too_narrow_for_operation_width_is_rejected():
    region = RegionSnapshot(
        (
            Point2D(0.0, 0.0),
            Point2D(2.0, 0.0),
            Point2D(2.0, 0.2),
            Point2D(0.0, 0.2),
        ),
        tuple(),
    )
    with pytest.raises(ValueError, match="too narrow"):
        validate_region(region, 0.5)


@pytest.mark.parametrize('holes', [
    ((_square(0.0, 1.0, 1.0)),),
    ((Point2D(0, 2), Point2D(1, 1), Point2D(1, 3)),),
    (_square(1, 1, 1), _square(2, 2, 1)),
    (_square(1, 1, 1), _square(1.5, 1.5, 1)),
])
def test_hole_boundary_contact_and_hole_intersections_are_rejected(holes):
    with pytest.raises(ValueError, match='invalid polygon'):
        validate_region(RegionSnapshot(_square(), holes))


@pytest.mark.parametrize('coordinate', [float('nan'), float('inf'), 1e200, True])
def test_illegal_vertices_are_rejected_before_geometry_operations(coordinate):
    with pytest.raises(ValueError, match='coordinates'):
        validate_region(RegionSnapshot((Point2D(coordinate, 0), *_square()[1:]), ()))


def test_undo_reopens_closed_rings_and_preserves_other_boundaries():
    editor = RegionEditor()
    for point in _square():
        editor.add_point(point)
    editor.close_outer()
    editor.begin_hole()
    for point in _square(1, 1, 1):
        editor.add_point(point)
    editor.close_hole()
    editor.undo()
    assert editor.mode == EditMode.HOLE_DRAFT
    assert editor.draft_hole == list(_square(1, 1, 1))
    assert editor.outer == list(_square())
    editor.undo()
    assert editor.draft_hole == list(_square(1, 1, 1))[:-1]
    editor.add_point(_square(1, 1, 1)[-1])
    editor.close_hole()
    editor.undo()
    editor.undo()
    editor.clear()
    for point in _square():
        editor.add_point(point)
    editor.close_outer()
    editor.undo()
    assert editor.mode == EditMode.OUTER_DRAFT
    assert editor.outer == list(_square())
    editor.undo()
    assert editor.outer == list(_square())[:-1]


def test_long_narrow_protrusion_is_rejected():
    coordinates = [(0, 0), (4, 0), (4, 1.9), (6, 1.9),
                   (6, 2.1), (4, 2.1), (4, 4), (0, 4)]
    with pytest.raises(ValueError, match='too narrow'):
        validate_region(RegionSnapshot(tuple(Point2D(*p) for p in coordinates), ()), 0.6)


def test_ordinary_convex_corners_bends_and_circular_holes_remain_valid():
    import math
    # The pointed end is an ordinary convex corner, not a long straight corridor.
    triangle = tuple(Point2D(*point) for point in [(0, 0), (4, 0), (2, 4)])
    assert validate_region(RegionSnapshot(triangle, ()), 0.6).area == 8
    # A wide L bend with a re-entrant corner remains usable.
    bend = tuple(Point2D(*point) for point in [
        (0, 0), (4, 0), (4, 2), (2, 2), (2, 4), (0, 4)])
    assert validate_region(RegionSnapshot(bend, ()), 0.6).area == 12
    tapered = tuple(Point2D(*point) for point in [
        (0, 0), (4, 0), (4, 1.85), (6, 1.9),
        (6, 2.1), (4, 2.15), (4, 4), (0, 4)])
    with pytest.raises(ValueError, match="too narrow"):
        validate_region(RegionSnapshot(tapered, ()), 0.6)
    circle = tuple(Point2D(4 + 3 * math.cos(i * math.pi / 16),
                           4 + 3 * math.sin(i * math.pi / 16)) for i in range(32))
    hole = tuple(Point2D(4 + math.cos(i * math.pi / 16),
                         4 + math.sin(i * math.pi / 16)) for i in range(32))
    assert validate_region(RegionSnapshot(circle, (hole,)), 0.6).is_valid


def test_width_boundary_and_long_narrow_hole_corridor():
    def rectangle(height):
        return RegionSnapshot(tuple(Point2D(*p) for p in [
            (0, 0), (4, 0), (4, height), (0, height)]), ())
    assert validate_region(rectangle(0.61), 0.6).is_valid
    with pytest.raises(ValueError, match='too narrow'):
        validate_region(RegionSnapshot(_square(), (_square(0.2, 1.0, 2.0),)), 0.6)


def test_reopening_last_hole_preserves_previous_holes():
    editor = RegionEditor()
    for point in _square(size=8):
        editor.add_point(point)
    editor.close_outer()
    for hole in (_square(1, 1, 1), _square(5, 5, 1)):
        editor.begin_hole()
        for point in hole:
            editor.add_point(point)
        editor.close_hole()
    editor.undo()
    assert len(editor.holes) == 1
    assert set(editor.holes[0]) == set(_square(1, 1, 1))
    assert editor.draft_hole == list(_square(5, 5, 1))


def test_empty_hole_draft_can_be_abandoned_without_clearing_previous_holes():
    editor = RegionEditor()
    for point in _square(size=8):
        editor.add_point(point)
    editor.close_outer()
    for hole in (_square(1, 1, 1), _square(5, 5, 1)):
        editor.begin_hole()
        for point in hole:
            editor.add_point(point)
        editor.close_hole()
    editor.undo()
    for _ in range(4):
        editor.undo()
    editor.undo()
    assert editor.mode == EditMode.OUTER_CLOSED
    assert len(editor.snapshot().holes) == 1


@pytest.mark.parametrize('coordinates', [
    # A short stub, shorter than the 0.6 m operation width.
    [(0, 0), (4, 0), (4, 1.9), (4.2, 1.9), (4.2, 2.1), (4, 2.1), (4, 4), (0, 4)],
    # A bent 0.2 m branch, including nonparallel corner segments.
    [(0, 0), (4, 0), (4, 1.8), (5, 1.8), (5.2, 2), (5.2, 3),
     (5, 3), (5, 2.2), (4.8, 2), (4, 2), (4, 4), (0, 4)],
])
def test_short_and_bent_narrow_branches_are_rejected(coordinates):
    region = RegionSnapshot(tuple(Point2D(*p) for p in coordinates), ())
    with pytest.raises(ValueError, match='too narrow'):
        validate_region(region, 0.6)


def test_curved_narrow_branch_and_hole_gap_are_rejected():
    import math
    from shapely.geometry import LineString, Point, Polygon

    room = Polygon([(0, 0), (4, 0), (4, 4), (0, 4)])
    arc = LineString([(3.8 + 1.8 * math.cos(i * math.pi / 32),
                       2 + 1.8 * math.sin(i * math.pi / 32))
                      for i in range(-16, 1)])
    curved = room.union(arc.buffer(0.1))
    hole = Point(3.7, 2).buffer(1, resolution=16)
    gap = Polygon([(0, 0), (5, 0), (5, 4), (0, 4)], [hole.exterior.coords])
    for polygon in (curved, gap):
        outer = tuple(Point2D(x, y) for x, y in list(polygon.exterior.coords)[:-1])
        holes = tuple(tuple(Point2D(x, y) for x, y in list(ring.coords)[:-1])
                      for ring in polygon.interiors)
        with pytest.raises(ValueError, match='too narrow'):
            validate_region(RegionSnapshot(outer, holes), 0.6)


def test_split_collinear_sides_do_not_make_convex_corner_a_passage():
    triangle = [(0, 0), (3, 0), (6, 0), (3.15, 1.5), (0.3, 3), (0.15, 1.5)]
    assert validate_region(RegionSnapshot(tuple(Point2D(*p) for p in triangle), ()), 0.6).is_valid


def test_excessive_finite_scale_is_rejected_explicitly():
    with pytest.raises(ValueError, match='too complex'):
        validate_region(RegionSnapshot(_square(), ()), 1e-12)


def test_polygonal_rounded_convex_corner_is_not_a_narrow_passage():
    from shapely.geometry import Polygon

    triangle = Polygon([(0, 0), (5, 0), (1, 3)]).buffer(0.05)
    outer = tuple(Point2D(x, y) for x, y in list(triangle.exterior.coords)[:-1])
    assert validate_region(RegionSnapshot(outer, ()), 0.6).is_valid


@pytest.mark.parametrize('gap,accepted', [(0.59, False), (0.6, True), (0.61, True)])
def test_sampled_passage_width_boundary(gap, accepted):
    coords = [(0, 0), (4, 0), (4, 2 - gap / 2), (6, 2 - gap / 2),
              (6, 2 + gap / 2), (4, 2 + gap / 2), (4, 4), (0, 4)]
    region = RegionSnapshot(tuple(Point2D(*p) for p in coords), ())
    if accepted:
        assert validate_region(region, 0.6).is_valid
    else:
        with pytest.raises(ValueError, match='too narrow'):
            validate_region(region, 0.6)


def test_excess_holes_and_sampling_work_are_rejected_before_probing():
    import math
    with pytest.raises(ValueError, match='too complex'):
        validate_region(RegionSnapshot(_square(), (_square(1, 1, 0.01),) * 65), 0.6)
    outer = tuple(Point2D(20 * math.cos(i * 2 * math.pi / 1200),
                          20 * math.sin(i * 2 * math.pi / 1200)) for i in range(1200))
    with pytest.raises(ValueError, match='too complex'):
        validate_region(RegionSnapshot(outer, (_square(-1, -1, 1),)), 0.6)


@pytest.mark.parametrize('gap,accepted', [(0.5999, False), (0.6001, True)])
def test_width_threshold_after_rotation_and_large_translation(gap, accepted):
    import math

    angle = 0.437
    coordinates = [(0, 0), (4, 0), (4, 2 - gap / 2), (6, 2 - gap / 2),
                   (6, 2 + gap / 2), (4, 2 + gap / 2), (4, 4), (0, 4)]
    points = tuple(Point2D(1e6 + x * math.cos(angle) - y * math.sin(angle),
                           -1e6 + x * math.sin(angle) + y * math.cos(angle))
                   for x, y in coordinates)
    region = RegionSnapshot(points, ())
    if accepted:
        assert validate_region(region, 0.6).is_valid
    else:
        with pytest.raises(ValueError, match='too narrow'):
            validate_region(region, 0.6)


@pytest.mark.parametrize('angle', [0, 0.437])
def test_convex_flat_ended_taper_with_long_narrow_end_is_rejected(angle):
    import math

    # Splitting a side also exercises removal of redundant collinear vertices.
    coordinates = [(0, 0), (15, 0), (30, 0), (30, 0.2), (0, 4)]
    region = RegionSnapshot(tuple(Point2D(1000 + x * math.cos(angle) - y * math.sin(angle),
                                          -1000 + x * math.sin(angle) + y * math.cos(angle))
                                  for x, y in coordinates), ())
    with pytest.raises(ValueError, match='too narrow'):
        validate_region(region, 0.6)


@pytest.mark.parametrize('angle', [0, 31])
def test_rounded_convex_corner_of_nonconvex_region_is_accepted(angle):
    from shapely.affinity import rotate
    from shapely.geometry import Polygon

    polygon = rotate(Polygon([(0, 0), (8, 0), (1, 4), (1, 3), (0, 3)]).buffer(0.05),
                     angle)
    outer = tuple(Point2D(x, y) for x, y in list(polygon.exterior.coords)[:-1])
    assert validate_region(RegionSnapshot(outer, ()), 0.6).is_valid


@pytest.mark.parametrize('gap,accepted', [(0.59, False), (0.6, True), (0.61, True)])
def test_gap_between_holes_respects_operation_width(gap, accepted):
    def rectangle(x0, x1):
        return tuple(Point2D(*p) for p in [(x0, 2), (x1, 2), (x1, 8), (x0, 8)])

    region = RegionSnapshot(_square(size=10), (rectangle(1, 4), rectangle(4 + gap, 8)))
    if accepted:
        assert validate_region(region, 0.6).is_valid
    else:
        with pytest.raises(ValueError, match='too narrow'):
            validate_region(region, 0.6)


@pytest.mark.parametrize("radius", [0.001, 0.01, 0.05, 0.1])
def test_narrow_branch_with_rounded_blunt_end_is_rejected(radius):
    from shapely.geometry import Polygon

    polygon = Polygon([(0, 0), (4, 0), (4, 1.9), (5, 1.9), (5, 2.1),
                       (4, 2.1), (4, 4), (0, 4)]).buffer(radius)
    outer = tuple(Point2D(x, y) for x, y in list(polygon.exterior.coords)[:-1])
    with pytest.raises(ValueError, match='too narrow'):
        validate_region(RegionSnapshot(outer, ()), 0.6)


@pytest.mark.parametrize("resolution", [4, 16, 32])
def test_narrow_branch_with_semicircular_end_is_rejected(resolution):
    from shapely.geometry import LineString, Polygon

    room = Polygon([(0, 0), (4, 0), (4, 4), (0, 4)])
    polygon = room.union(LineString([(3.9, 2), (5, 2)]).buffer(0.1, resolution=resolution))
    outer = tuple(Point2D(x, y) for x, y in list(polygon.exterior.coords)[:-1])
    with pytest.raises(ValueError, match='too narrow'):
        validate_region(RegionSnapshot(outer, ()), 0.6)


@pytest.mark.parametrize('radius', [0, 0.01, 0.05])
def test_ordinary_acute_corner_with_or_without_rounding_is_accepted(radius):
    from shapely.geometry import Polygon

    polygon = Polygon([(0, 0), (8, 0), (0, 4)])
    if radius:
        polygon = polygon.buffer(radius)
    outer = tuple(Point2D(x, y) for x, y in list(polygon.exterior.coords)[:-1])
    assert validate_region(RegionSnapshot(outer, ()), 0.6).is_valid

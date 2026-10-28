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


def test_undo_closed_boundary_does_not_corrupt_valid_region():
    editor = RegionEditor()
    for point in _square():
        editor.add_point(point)
    editor.close_outer()
    with pytest.raises(ValueError, match='draft'):
        editor.undo()
    assert validate_region(editor.snapshot()).area == 16.0

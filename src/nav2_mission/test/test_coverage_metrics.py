import pytest

from nav2_mission.coverage.metrics import (
    DistanceAccumulator,
    covered_area_ratio,
    path_completion,
    polyline_length,
)
from nav2_mission.coverage.models import Point2D, RegionSnapshot


def test_polyline_and_odom_distance_ignore_small_jitter():
    assert polyline_length([(0.0, 0.0), (3.0, 4.0), (6.0, 4.0)]) == 8.0
    accumulator = DistanceAccumulator(jitter_threshold=0.1)
    accumulator.add(0.0, 0.0)
    accumulator.add(0.01, 0.0)
    accumulator.add(1.0, 0.0)
    assert accumulator.distance == pytest.approx(1.0)


def test_distance_accumulator_preserves_low_speed_motion():
    accumulator = DistanceAccumulator(jitter_threshold=0.01)
    for sample in range(21):
        accumulator.add(sample * 0.002, 0.0)
    # Motion is accumulated in threshold-sized chunks; at most one final
    # sub-threshold remainder is intentionally omitted as jitter.
    assert 0.03 <= accumulator.distance <= 0.04


def test_covered_area_is_clipped_and_overlap_is_not_double_counted():
    region = RegionSnapshot(
        (
            Point2D(0.0, 0.0),
            Point2D(4.0, 0.0),
            Point2D(4.0, 4.0),
            Point2D(0.0, 4.0),
        ),
        tuple(),
    )
    trace = [Point2D(0.0, 2.0), Point2D(4.0, 2.0), Point2D(0.0, 2.0)]
    ratio = covered_area_ratio(trace, region, 1.0)
    assert ratio == pytest.approx(0.25, rel=0.05)


def test_hole_is_removed_from_target_and_swept_area():
    region = RegionSnapshot(
        (
            Point2D(0.0, 0.0),
            Point2D(4.0, 0.0),
            Point2D(4.0, 4.0),
            Point2D(0.0, 4.0),
        ),
        ((Point2D(1.0, 1.0), Point2D(1.0, 3.0), Point2D(3.0, 3.0), Point2D(3.0, 1.0)),),
    )
    ratio = covered_area_ratio(
        [Point2D(0.0, 2.0), Point2D(4.0, 2.0)], region, 1.0
    )
    assert 0.15 < ratio < 0.18


def test_path_completion_is_clamped():
    assert path_completion(10.0, 7.5) == 0.25
    assert path_completion(10.0, -1.0) == 1.0
    assert path_completion(10.0, 12.0) == 0.0
    assert path_completion(0.0, 0.0) == 0.0


def test_trace_gaps_are_not_swept_as_unobserved_connecting_lines():
    from nav2_mission.coverage.metrics import covered_segments_area_ratio
    region = RegionSnapshot((Point2D(0, 0), Point2D(4, 0),
                             Point2D(4, 4), Point2D(0, 4)), ())
    segments = [[Point2D(0, 1), Point2D(4, 1)], [Point2D(0, 3), Point2D(4, 3)]]
    assert covered_segments_area_ratio(segments, region, 0.5) == pytest.approx(0.25)

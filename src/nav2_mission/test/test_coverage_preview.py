"""Coverage preview goals must stay on the compute-only action interface."""

from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from opennav_coverage_msgs.action import ComputeCoveragePath, NavigateCompleteCoverage
from rclpy.parameter import Parameter

from nav2_mission.coverage_region_selector_node import CoverageRegionSelector


@pytest.fixture
def selector():
    rclpy.init()
    node = CoverageRegionSelector(parameter_overrides=[
        Parameter("plan_only", value=True),
        Parameter("auto_plan_on_close", value=True),
    ])
    real_client = node._execute_client
    node._execute_client = Mock()
    node._execute_client.wait_for_server.return_value = True
    node._execute_client.send_goal_async.return_value = Future()
    real_publisher = node._preview_path_pub
    node._preview_path_pub = Mock()
    yield node
    node._execute_client = real_client
    node._preview_path_pub = real_publisher
    node.destroy_node()
    rclpy.shutdown()


def click(node, x, y):
    point = PointStamped()
    point.header.frame_id = "map"
    point.point.x = float(x)
    point.point.y = float(y)
    node._on_clicked_point(point)


def test_closing_region_sends_only_a_compute_goal(selector):
    for x, y in [(0, 0), (8, 0), (8, 8), (0, 8), (0.1, 0.1)]:
        click(selector, x, y)
    goal = selector._execute_client.send_goal_async.call_args.args[0]
    assert isinstance(goal, ComputeCoveragePath.Goal)
    assert selector._execute_action_name == "/compute_coverage_path"
    assert goal.frame_id == "map"
    assert goal.generate_headland and goal.generate_route and goal.generate_path
    assert [(p.axis1, p.axis2) for p in goal.polygons[0].coordinates] == [
        (0, 0), (8, 0), (8, 8), (0, 8), (0, 0),
    ]
    assert selector._goal_active
    assert not any(t == "/cmd_vel" for t, _ in selector.get_topic_names_and_types())


def test_preview_result_is_published_and_new_polygon_clears_it(selector):
    path = Path()
    path.header.frame_id = "map"
    path.poses.append(PoseStamped())
    future = Future()
    future.set_result(SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(error_code=0, nav_path=path),
    ))
    selector._on_goal_result(future)
    assert selector._preview_path_pub.publish.call_args.args[0] is path
    selector._points = [(0, 0), (8, 0), (8, 8)]
    selector._closed = True
    click(selector, 10, 10)
    assert selector._preview_path_pub.publish.call_args.args[0].poses == []


def test_failed_planning_never_publishes_a_failed_path(selector):
    future = Future()
    future.set_result(SimpleNamespace(
        status=GoalStatus.STATUS_ABORTED,
        result=SimpleNamespace(error_code=803, nav_path=Path()),
    ))
    selector._on_goal_result(future)
    selector._preview_path_pub.publish.assert_not_called()
    assert not selector._goal_active


def test_non_finite_clicked_point_is_ignored(selector):
    click(selector, float("nan"), 0)
    assert selector._points == []


def test_default_mode_retains_execution_client():
    rclpy.init()
    node = CoverageRegionSelector()
    try:
        assert not node._plan_only
        assert node._preview_path_pub is None
        assert node._execute_action_name == "/execute_coverage_navpath"
        assert node._execute_client._action_type is NavigateCompleteCoverage
    finally:
        node.destroy_node()
        rclpy.shutdown()

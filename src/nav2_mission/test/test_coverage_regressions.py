"""Coverage regressions with real ROS messages, without driving hardware."""

import math
import threading
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from nav2_mission.coverage_navpath_bridge_node import CoverageNavPathBridge


def bridge():
    node = CoverageNavPathBridge.__new__(CoverageNavPathBridge)
    node.get_logger = Mock(return_value=Mock())
    node._navigate_to_start = True
    node._default_frame_id = "map"
    node._start_position_tolerance = 0.5
    node._start_yaw_tolerance = 0.5
    node._server_wait_timeout_sec = 1.0
    node._navigate_to_start_timeout_sec = 1.0
    node._navigate_to_pose_action_name = "/navigate_to_pose"
    node._robot_pose_in = Mock(return_value=(0.0, 0.0, 0.0))
    node._wait_for_action_server = Mock(return_value=True)
    node._wait_future = Mock(return_value=node.WAIT_DONE)
    node.get_clock = Mock(return_value=NS(now=lambda: NS(to_msg=lambda: PoseStamped().header.stamp)))
    handle = NS(accepted=True, get_result_async=lambda: NS(result=lambda: NS(status=4)))
    node._navigate_to_pose_client = NS(send_goal_async=Mock(return_value=NS(result=lambda: handle)))
    return node


def test_approach_fills_pose_header_without_mutating_path():
    node = bridge()
    path = Path()
    path.header.frame_id = "map"
    pose = PoseStamped()
    pose.pose.position.x = 5.0
    pose.pose.orientation.w = 1.0
    path.poses = [pose]
    assert node._approach_start(path, Mock(), NS(error_code=0))
    goal = node._navigate_to_pose_client.send_goal_async.call_args.args[0]
    assert goal.pose.header.frame_id == "map"
    assert path.poses[0].header.frame_id == ""


def test_missing_tf_does_not_dispatch_follow_path():
    node = bridge()
    node._robot_pose_in.return_value = None
    path = Path()
    path.header.frame_id = "map"
    path.poses = [PoseStamped()]
    result, goal = NS(error_code=0), Mock()
    assert not node._approach_start(path, goal, result)
    assert result.error_code == node.ERR_APPROACH_FAILED
    goal.abort.assert_called_once()


def test_revisited_start_does_not_drop_unvisited_coverage():
    node = bridge()
    node._base_frame_id = "base_link"
    node._tf_buffer = NS(lookup_transform=lambda *args: NS(transform=NS(translation=NS(x=0.29, y=0.0))))
    path = Path()
    path.header.frame_id = "map"
    for x, y in [(0, 0), (2, 0), (2, 2), (0.3, 0)]:
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        path.poses.append(pose)
    assert len(node._trim_path_for_follow(path).poses) == 4


@pytest.mark.parametrize("points", [
    [(0, 0), (2, 2), (0, 2), (2, 0)],
    [(0, 0), (1, 0), (2, 0)],
    [(0, 0), (1, 0), (math.nan, 2)],
    [(0, 0), (1, 0), (math.inf, 2)],
    [(0, 0), (1, 1), (0, 0)],
])
def test_invalid_region_rejected(points):
    with pytest.raises(ValueError):
        bridge()._normalize_polygon(points, "fixture")


def test_valid_polygon_normalized():
    points = [(0, 0), (0, 0), (2, 0), (2, 2), (0, 2)]
    assert bridge()._normalize_polygon(points, "fixture") == [
        (0, 0), (2, 0), (2, 2), (0, 2), (0, 0)]


@pytest.mark.parametrize("valid", [True, False])
def test_costmap_gate(valid):
    node = bridge()
    node.get_parameter = Mock(return_value=NS(value=True))
    node._dependency_timeout_sec = 1.0
    response = NS(is_valid=valid, invalid_pose_indices=[] if valid else [2])
    node._path_valid_client = NS(service_is_ready=lambda: True,
                                call_async=lambda request: NS(result=lambda: response))
    result, goal = NS(error_code=0), Mock()
    assert node._validate_nav_path(Path(), goal, result) == valid
    if not valid:
        assert result.error_code == node.ERR_UNSAFE_NAV_PATH
        goal.abort.assert_called_once()


def test_safety_abort_precedes_completed_future(monkeypatch):
    node = bridge()
    node._safety_abort_lock = threading.Lock()
    node._safety_abort = True
    monkeypatch.setattr("rclpy.ok", lambda: True)
    cancel = Mock()
    assert CoverageNavPathBridge._wait_future(
        node, NS(done=lambda: True), 1.0, cancel_fn=cancel) == node.WAIT_ABORTED
    cancel.assert_called_once()

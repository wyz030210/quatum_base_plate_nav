"""Public session editor services and RViz input on the real manager node."""
import json
import time

import pytest

from nav2_mission_interfaces.srv import RegionCommand, StartTask
from geometry_msgs.msg import PointStamped

import test_task_control_ros as task_test

ros_system = task_test.ros_system


def edit(call, operation, **payload):
    return call(RegionCommand, '/task_control/region', RegionCommand.Request(
        operation=operation, request_json=json.dumps(payload)))


def test_rviz_draft_close_hole_undo_and_atomic_active_edit_refusal(ros_system):
    call, goals, commands, boundary = ros_system
    assert edit(call, 'configure', frame_id='map', operation_width=0.5,
                default_swath_angle=0.2).accepted
    for x, y in [(0, 0), (4, 0), (4, 4), (0, 4)]:
        assert edit(call, 'point', x=x, y=y, frame_id='map').accepted
    assert edit(call, 'close_outer').accepted
    assert edit(call, 'begin_hole').accepted
    for x, y in [(1, 1), (2, 1), (2, 2), (1, 2), (1, 3)]:
        assert edit(call, 'point', x=x, y=y).accepted
    assert edit(call, 'undo').accepted
    assert edit(call, 'close_hole').accepted
    before = json.loads(edit(call, 'status').state_json)
    assert before['valid'] and len(before['polygons']) == 2
    boundary['execute_time'] = 2.0
    start = call(StartTask, '/task_control/start', StartTask.Request(
        task_type='inspection', request_json=json.dumps({'frame_id': 'map', 'point': {
            'id': 'busy', 'type': 'security', 'x': 0.0, 'y': 0.0, 'yaw': 0.0,
            'stay_time': 0.0}})))
    assert start.accepted
    for op, payload in [('clear', {}), ('undo', {}), ('begin_hole', {}),
                        ('point', {'x': 2, 'y': 3}),
                        ('configure', {'operation_width': 1.0}), ('start', {})]:
        reply = edit(call, op, **payload)
        assert not reply.accepted and 'active' in reply.message
    after = json.loads(edit(call, 'status').state_json)
    assert after['polygons'] == before['polygons']
    assert after['planner'] == before['planner']


def test_rviz_points_and_cli_operate_on_same_session(ros_system):
    import subprocess
    import sys
    from rclpy.node import Node
    call, _, _, _ = ros_system
    publisher_node = Node('rviz_point_tool_test')
    publisher = publisher_node.create_publisher(PointStamped, '/clicked_point', 10)
    try:
        deadline = time.monotonic() + 3
        while not publisher.get_subscription_count() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert publisher.get_subscription_count()
        for x, y in [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]:
            point = PointStamped()
            point.header.frame_id = 'map'
            point.point.x, point.point.y = x, y
            publisher.publish(point)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            state = json.loads(edit(call, 'status').state_json)
            if len(state['polygons'][0]) == 4:
                break
            time.sleep(0.02)
        assert len(state['polygons'][0]) == 4
        result = subprocess.run(
            [sys.executable, '-m', 'nav2_mission.task_cli', 'region', 'close_outer'],
            text=True, capture_output=True, timeout=10)
        assert result.returncode == 0, result.stderr
        assert json.loads(result.stdout)['state']['valid']
        result = subprocess.run(
            [sys.executable, '-m', 'nav2_mission.task_cli', 'region', 'undo'],
            text=True, capture_output=True, timeout=10)
        assert result.returncode == 0, result.stderr
        reopened = json.loads(result.stdout)['state']
        assert reopened['mode'] == 'OUTER_DRAFT' and len(reopened['polygons'][0]) == 4
        assert edit(call, 'close_outer').accepted
        assert edit(call, 'begin_hole').accepted
        for x, y in [(1, 1), (2, 1), (2, 2), (1, 2)]:
            assert edit(call, 'point', x=x, y=y).accepted
        assert edit(call, 'close_hole').accepted
        result = subprocess.run(
            [sys.executable, '-m', 'nav2_mission.task_cli', 'region', 'undo'],
            text=True, capture_output=True, timeout=10)
        assert result.returncode == 0, result.stderr
        reopened = json.loads(result.stdout)['state']
        assert reopened['mode'] == 'HOLE_DRAFT' and len(reopened['draft_hole']) == 4
        assert len(reopened['polygons']) == 1 and len(reopened['polygons'][0]) == 4
        assert edit(call, 'clear').accepted
        assert not json.loads(edit(call, 'status').state_json)['valid']
    finally:
        publisher_node.destroy_node()


@pytest.mark.parametrize('coordinates', [
    [(0, 0), (4, 0), (4, 1.9), (6, 1.9), (6, 2.1), (4, 2.1), (4, 4), (0, 4)],
    [(0, 0), (4, 0), (4, 1.85), (6, 1.9), (6, 2.1), (4, 2.15), (4, 4), (0, 4)],
    [(0, 0), (4, 0), (4, 1.9), (4.2, 1.9), (4.2, 2.1), (4, 2.1), (4, 4), (0, 4)],
    [(0, 0), (4, 0), (4, 1.8), (5, 1.8), (5.2, 2), (5.2, 3),
     (5, 3), (5, 2.2), (4.8, 2), (4, 2), (4, 4), (0, 4)],
])
def test_editor_rejects_narrow_protrusions(ros_system, coordinates):
    call, _, _, _ = ros_system
    for x, y in coordinates:
        assert edit(call, 'point', x=x, y=y).accepted
    result = edit(call, 'close_outer')
    assert not result.accepted and 'too narrow' in result.message
    assert not json.loads(edit(call, 'status').state_json)['valid']


def test_rejecting_larger_width_preserves_confirmed_editor_state(ros_system):
    call, _, _, _ = ros_system
    coordinates = [(0, 0), (4, 0), (4, 1.65), (5, 1.65),
                   (5, 2.35), (4, 2.35), (4, 4), (0, 4)]
    for x, y in coordinates:
        assert edit(call, 'point', x=x, y=y).accepted
    assert edit(call, 'close_outer').accepted
    before = json.loads(edit(call, 'status').state_json)
    assert before['valid']
    reply = edit(call, 'configure', operation_width=1.0)
    assert not reply.accepted and 'too narrow' in reply.message
    after = json.loads(edit(call, 'status').state_json)
    assert after['valid'] and after['planner'] == before['planner']
    assert after['polygons'] == before['polygons']
    assert not edit(call, 'configure', operation_width=1e-12).accepted
    after = json.loads(edit(call, 'status').state_json)
    assert after['planner'] == before['planner'] and after['valid']


def test_cli_can_abandon_reopened_empty_hole_without_losing_previous_one(ros_system):
    import subprocess
    import sys

    call, _, _, _ = ros_system
    for x, y in [(0, 0), (8, 0), (8, 8), (0, 8)]:
        assert edit(call, 'point', x=x, y=y).accepted
    assert edit(call, 'close_outer').accepted
    for x0, y0 in [(1, 1), (5, 5)]:
        assert edit(call, 'begin_hole').accepted
        for x, y in [(x0, y0), (x0 + 1, y0), (x0 + 1, y0 + 1), (x0, y0 + 1)]:
            assert edit(call, 'point', x=x, y=y).accepted
        assert edit(call, 'close_hole').accepted
    assert edit(call, 'undo').accepted  # reopen the second hole
    for _ in range(4):
        assert edit(call, 'undo').accepted
    result = subprocess.run(
        [sys.executable, '-m', 'nav2_mission.task_cli', 'region', 'undo'],
        text=True, capture_output=True, timeout=10)
    assert result.returncode == 0, result.stderr
    state = json.loads(result.stdout)['state']
    assert state['valid'] and state['mode'] == 'OUTER_CLOSED'
    assert len(state['polygons']) == 2 and not state['draft_hole']
    assert edit(call, 'begin_hole').accepted


def test_editor_accepts_a_circular_hole_and_ordinary_sharp_corners(ros_system):
    import math

    call, _, _, _ = ros_system
    for x, y in [(0, 0), (8, 0), (8, 8), (0, 8)]:
        assert edit(call, 'point', x=x, y=y).accepted
    assert edit(call, 'close_outer').accepted
    assert edit(call, 'begin_hole').accepted
    for i in range(24):
        angle = i * 2 * math.pi / 24
        assert edit(call, 'point', x=4 + math.cos(angle), y=4 + math.sin(angle)).accepted
    assert edit(call, 'close_hole').accepted
    state = json.loads(edit(call, 'status').state_json)
    assert state['valid'] and len(state['polygons']) == 2


def test_editor_rejects_curved_narrow_branch(ros_system):
    import math
    from shapely.geometry import LineString, Polygon

    call, _, _, _ = ros_system
    room = Polygon([(0, 0), (4, 0), (4, 4), (0, 4)])
    arc = LineString([(3.8 + 1.8 * math.cos(i * math.pi / 32),
                       2 + 1.8 * math.sin(i * math.pi / 32))
                      for i in range(-16, 1)])
    for x, y in list(room.union(arc.buffer(0.1)).exterior.coords)[:-1]:
        assert edit(call, 'point', x=x, y=y).accepted
    reply = edit(call, 'close_outer')
    assert not reply.accepted and 'too narrow' in reply.message

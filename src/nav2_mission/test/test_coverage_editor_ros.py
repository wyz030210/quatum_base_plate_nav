"""Public session editor services and RViz input on the real manager node."""
import json
import time

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
        assert edit(call, 'clear').accepted
        assert not json.loads(edit(call, 'status').state_json)['valid']
    finally:
        publisher_node.destroy_node()

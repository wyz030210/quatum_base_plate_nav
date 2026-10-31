"""Installed control client at real ROS editor/action/service boundaries."""
import json
import threading
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_mission_interfaces.srv import CancelTask, RegionCommand
from nav_msgs.msg import OccupancyGrid, Path
from opennav_coverage_msgs.action import ComputeCoveragePath
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from nav2_mission.coverage_control_gui import CoverageControls, map_center_summary
from nav2_mission.coverage_editor_node import CoverageEditorNode


def until(predicate, timeout=8):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.02)
    assert predicate(), 'ROS condition did not complete before timeout'


@pytest.fixture
def controls():
    rclpy.init()
    editor = CoverageEditorNode()
    gui = CoverageControls(parameter_overrides=[Parameter('enable_execution', value=True)])
    preview = Node('coverage_server', namespace='coverage_preview')
    vehicle = Node('coverage_server')
    boundary = {'goals': [], 'hold': False, 'release': threading.Event(),
                'error': 0, 'cancels': 0, 'paths': []}
    for name, value in {'task_parameter_owner': '', 'task_parameter_release': '',
                        'operation_width': 0.6, 'default_swath_angle': 1.57,
                        'swath_spacing': 1.5, 'default_headland_width': 1.5,
                        'min_turning_radius': 0.95}.items():
        preview.declare_parameter(name, value)
    vehicle.declare_parameter('operation_width', 9.0)

    def compute(handle):
        boundary['goals'].append(handle.request)
        if boundary['hold']:
            boundary['release'].wait(timeout=6)
        result = ComputeCoveragePath.Result(error_code=boundary['error'])
        result.nav_path.header.frame_id = 'map'
        result.nav_path.poses = [PoseStamped() for _ in range(3)]
        for i, pose in enumerate(result.nav_path.poses):
            pose.pose.position.x = float(i)
            pose.pose.orientation.w = 1.0
        if handle.is_cancel_requested:
            handle.canceled()
        elif result.error_code:
            handle.abort()
        else:
            handle.succeed()
        return result

    action = ActionServer(preview, ComputeCoveragePath, 'compute_coverage_path',
                          execute_callback=compute, callback_group=ReentrantCallbackGroup(),
                          cancel_callback=lambda _: CancelResponse.ACCEPT)

    def cancel(request, response):
        boundary['cancels'] += 1
        response.accepted = True
        return response
    vehicle.create_service(CancelTask, '/task_control/cancel', cancel)
    vehicle.create_subscription(Path, '/coverage_ui/preview_path',
                                lambda msg: boundary['paths'].append(msg), 10)
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (editor, gui, preview, vehicle):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    until(lambda: gui.region.service_is_ready() and gui.cancel_task.service_is_ready())
    until(lambda: gui.compute.server_is_ready() and gui.set_params.service_is_ready())

    def operation(name, **payload):
        gui.submit(name, **payload)
        until(lambda: gui._worker is None or not gui._worker.is_alive())

    def region_call(operation, **payload):
        request = RegionCommand.Request(operation=operation, request_json=json.dumps(payload))
        return gui.call(gui.region, request)

    try:
        yield gui, preview, vehicle, boundary, operation, region_call
    finally:
        boundary['release'].set()
        gui.close()
        assert gui._worker is None or not gui._worker.is_alive()
        assert executor.shutdown(timeout_sec=4)
        thread.join(timeout=4)
        assert not thread.is_alive()
        action.destroy()
        for node in (gui, preview, vehicle, editor):
            node.destroy_node()
        rclpy.shutdown()


def rectangle(operation):
    for x, y in [(0., 0.), (8., 0.), (8., 8.), (0., 8.)]:
        operation('point', x=x, y=y)
    operation('close_outer')


def notices(gui):
    result = []
    while not gui.events.empty():
        kind, value = gui.events.get_nowait()
        if kind == 'notice':
            result.append(value)
    return result


def test_startup_is_idle_and_preview_uses_only_dedicated_planner(controls):
    gui, preview, vehicle, boundary, operation, region_call = controls
    assert not boundary['goals'] and not boundary['cancels']
    rectangle(operation)
    operation('configure', operation_width=0.7, default_swath_angle=0.4)
    operation('preview')
    until(lambda: any(path.poses for path in boundary['paths']))
    goal = boundary['goals'][0]
    assert goal.generate_headland and goal.generate_route and goal.generate_path
    assert len(goal.polygons[0].coordinates) == 5
    assert vehicle.get_parameter('operation_width').value == 9.0
    assert preview.get_parameter('operation_width').value == 0.7
    assert preview.get_parameter('task_parameter_owner').value == ''
    assert any('规划成功' in message for message in notices(gui))
    assert not region_call('start').accepted  # The standalone editor cannot move anything.


def test_changed_region_discards_pending_result_and_cancel_bypasses_preview(controls):
    gui, preview, _, boundary, operation, region_call = controls
    rectangle(operation)
    boundary['hold'] = True
    gui.submit('preview')
    until(lambda: bool(boundary['goals']))
    gui.submit('cancel')
    until(lambda: boundary['cancels'] == 1)
    assert region_call('clear').accepted
    boundary['release'].set()
    until(lambda: not gui._worker.is_alive())
    assert not any(path.poses for path in boundary['paths'])
    assert preview.get_parameter('task_parameter_owner').value == ''
    assert any('旧预览已丢弃' in message for message in notices(gui))


def test_failed_planning_never_publishes_a_path_and_releases_parameters(controls):
    gui, preview, _, boundary, operation, _ = controls
    rectangle(operation)
    boundary['error'] = 803
    operation('preview')
    assert not any(path.poses for path in boundary['paths'])
    assert any('803' in message for message in notices(gui))
    assert preview.get_parameter('task_parameter_owner').value == ''


def test_closing_window_cancels_preview_without_issuing_task_cancel(controls):
    gui, preview, _, boundary, operation, _ = controls
    rectangle(operation)
    boundary['hold'] = True
    gui.submit('preview')
    until(lambda: bool(boundary['goals']))
    closer = threading.Thread(target=gui.close)
    closer.start()
    boundary['release'].set()
    closer.join(timeout=8)
    assert not closer.is_alive()
    assert not boundary['cancels']
    assert preview.get_parameter('task_parameter_owner').value == ''


def test_map_diagnostic_does_not_claim_footprint_clearance():
    occupancy = OccupancyGrid()
    occupancy.header.frame_id = 'map'
    occupancy.info.width = 3
    occupancy.info.height = 1
    occupancy.info.resolution = 1.0
    occupancy.info.origin.orientation.w = 1.0
    occupancy.data = [0, 100, -1]
    path = Path()
    path.header.frame_id = 'map'
    for x in (0.1, 1.1, 2.1, 3.1):
        pose = PoseStamped()
        pose.pose.position.x = x
        path.poses.append(pose)
    result = map_center_summary(path, occupancy)
    assert '占据 1' in result and '未知 1' in result and '地图外 1' in result
    path.poses = path.poses[:1]
    assert '未检查车体轮廓' in map_center_summary(path, occupancy)

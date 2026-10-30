"""RViz/session/CLI through the real manager and bridge; fake downstream only."""
import json
import math
import subprocess
import sys
import threading
import uuid

import pytest
from nav2_mission_interfaces.srv import PrepareCoverage, StartTask
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter

from test_coverage_editor_ros import edit
import test_coverage_task_ros as coverage_test

coverage_system = coverage_test.coverage_system
wait_until = coverage_test.wait_until


@pytest.fixture
def delay_failure_telemetry(monkeypatch):
    from nav2_mission.coverage_task_ros import RosCoverage

    original = RosCoverage.receive
    release = threading.Event()
    delayed = []

    def receive(adapter, message):
        if json.loads(message.data).get('error'):
            def deliver():
                if release.wait(3):
                    original(adapter, message)
            thread = threading.Thread(target=deliver)
            delayed.append(thread)
            thread.start()
        else:
            original(adapter, message)

    monkeypatch.setattr(RosCoverage, 'receive', receive)
    yield release
    release.set()
    for thread in delayed:
        thread.join(timeout=3)


def draw(call, operation, points):
    for x, y in points:
        assert edit(call, 'point', x=x, y=y).accepted
    reply = edit(call, operation)
    assert reply.accepted, reply.message


def test_edited_hole_executes_one_frozen_complete_path_and_reference_report(coverage_system):
    _, status, call, boundary = coverage_system
    boundary['hold'] = 'follow'
    assert edit(call, 'configure', operation_width=0.7, default_swath_angle=0.4).accepted
    draw(call, 'close_outer', [(0, 0), (12, 0), (12, 12), (0, 12)])
    assert edit(call, 'begin_hole').accepted
    draw(call, 'close_hole', [(8, 8), (9, 8), (9, 9), (8, 9)])
    confirmed = json.loads(edit(call, 'status').state_json)
    result = subprocess.run([sys.executable, '-m', 'nav2_mission.task_cli', 'region', 'start'],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
    reply = json.loads(result.stdout)
    assert reply['accepted']
    running = wait_until(lambda: value if (value := status())['phase'] == 'following' else None)
    assert running['config']['planner'] == confirmed['planner']
    assert len(running['config']['polygons']) == 2
    assert not edit(call, 'configure', operation_width=1.2).accepted
    assert not edit(call, 'clear').accepted
    compute = [req for kind, req in boundary['goals'] if kind == 'compute']
    assert len(compute) == 1 and len(compute[0].polygons) == 2
    hole = {(p.axis1, p.axis2) for p in compute[0].polygons[1].coordinates}
    assert hole == {(8., 8.), (9., 8.), (9., 9.), (8., 9.)}
    follow = next(req for kind, req in boundary['goals'] if kind == 'follow')
    assert follow.path == boundary['path']
    wait_until(lambda: status()['details'].get('reference_statistics', {}).get(
        'trace_sample_count'))
    wait_until(lambda: boundary['visuals'].get('planned_path') and
               [p.pose for p in boundary['visuals']['planned_path'].poses] ==
               [p.pose for p in boundary['path'].poses])
    wait_until(lambda: boundary['visuals'].get('executed_trace') and
               len(boundary['visuals']['executed_trace'].markers) > 1)
    boundary['hold'] = ''
    ended = wait_until(lambda: value if not (value := status())['active'] else None)
    assert ended['result'] == 'succeeded' and ended['cleanup_confirmed'], ended
    with open(ended['report_path']) as stream:
        report = json.load(stream)
    assert report['task_id'] == reply['task_id']
    assert report['config'] == running['config']
    statistics = report['details']['reference_statistics']
    assert 0 < statistics['path_completion_ratio'] < 1
    assert 0 < statistics['trace_estimated_area_ratio'] < 0.01
    assert statistics['collision_count'] is None
    assert 'not measured cleaning quality' in statistics['basis']


def test_reference_area_uses_authoritative_width_when_request_omits_it(coverage_system):
    _, status, call, boundary = coverage_system
    boundary.update(hold='follow', pose_evidence={'x': 1.0, 'y': 1.0})
    changed = call(SetParametersAtomically, '/coverage_server/set_parameters_atomically',
                   SetParametersAtomically.Request(parameters=[
                       Parameter('operation_width', value=1.0).to_parameter_msg()]))
    assert changed.result.successful
    started = call(StartTask, '/task_control/start', StartTask.Request(
        task_type='coverage', request_json=json.dumps({
            'frame_id': 'map', 'polygons': [[[0, 0], [8, 0], [8, 8], [0, 8]]]})))
    assert started.accepted
    running = wait_until(lambda: value if (value := status())['phase'] == 'following' and
                         value['details']['reference_statistics']['trace_estimated_area_ratio']
                         else None)
    statistics = running['details']['reference_statistics']
    # A stationary sample sweeps a radius-0.5m disc inside a 64m2 square.
    assert statistics['trace_estimated_area_ratio'] == pytest.approx(math.pi / 256, rel=0.01)
    assert statistics['operation_width_m'] == 1.0
    boundary['hold'] = ''
    wait_until(lambda: not status()['active'])


def test_stale_pose_gap_is_visible_and_never_swept_as_a_connecting_line(coverage_system):
    start, status, _, boundary = coverage_system
    boundary.update(hold='follow', pose_evidence={'x': 1.0, 'y': 1.0})
    assert start().accepted

    def observed(x, y):
        value = status()
        return any([x, y] in segment for segment in
                   value.get('details', {}).get('executed_trace', {}).get('segments', []))

    wait_until(lambda: status()['phase'] == 'following' and
               status()['details']['reference_statistics']['trace_estimated_area_ratio'] > 0)
    boundary['pose_evidence'].update(x=2.0)
    wait_until(lambda: observed(2, 1))
    boundary['pose_evidence'].update(pose_stamp_sec=0.0)
    wait_until(lambda: status()['details']['reference_statistics']['trace_gap_count'] == 1)
    boundary['pose_evidence'].update(x=1.0, y=7.0)
    boundary['pose_evidence'].pop('pose_stamp_sec')
    wait_until(lambda: observed(1, 7))
    boundary['pose_evidence'].update(x=2.0)
    wait_until(lambda: observed(2, 7))
    boundary['hold'] = ''
    ended = wait_until(lambda: value if not (value := status())['active'] else None)
    assert ended['result'] == 'succeeded', ended
    with open(ended['report_path']) as stream:
        statistics = json.load(stream)['details']['reference_statistics']
    assert statistics['trace_gap_count'] == 1 and not statistics['trace_complete']
    assert statistics['trace_distance_m'] == pytest.approx(2.0)
    # Two length-1m swept capsules, separated by an unobserved 6m jump.
    assert statistics['trace_estimated_area_ratio'] == pytest.approx(
        2 * (0.7 + math.pi * 0.35 ** 2) / 64, rel=0.01)


def test_late_first_valid_pose_does_not_hide_missing_initial_trace(coverage_system):
    start, status, _, boundary = coverage_system
    boundary.update(hold='follow', pose_evidence={'pose_stamp_sec': 0.0})
    assert start().accepted
    wait_until(lambda: status()['phase'] == 'following')
    statistics = status()['details']['reference_statistics']
    assert statistics['trace_gap_count'] == 1 and not statistics['trace_complete']
    assert statistics['trace_sample_count'] == 0
    boundary['pose_evidence'].pop('pose_stamp_sec')
    wait_until(lambda: status()['details']['reference_statistics']['trace_sample_count'] > 0)
    boundary['hold'] = ''
    ended = wait_until(lambda: value if not (value := status())['active'] else None)
    statistics = ended['details']['reference_statistics']
    assert ended['result'] == 'succeeded'
    assert statistics['trace_gap_count'] == 1 and not statistics['trace_complete']


@pytest.mark.parametrize('polygons,planner', [
    ([[[0, 0], [4, 4], [0, 4], [4, 0]]], {}),
    ([[[0, 0], [8, 0], [8, 8], [0, 8]], [[7, 7], [9, 7], [9, 9], [7, 9]]], {}),
    ([[[0, 0], [8, 0], [8, 8], [0, 8]], [[0, 2], [1, 1], [1, 3]]], {}),
    ([[[0, 0], [8, 0], [8, 8], [0, 8]], [[1, 1], [2, 1], [2, 2], [1, 2]],
      [[2, 2], [3, 2], [3, 3], [2, 3]]], {}),
    ([[[False, 0], [8, 0], [8, 8], [0, 8]]], {}),
    ([[[0, 0], [1e200, 0], [8, 8], [0, 8]]], {}),
    ([[[0, 0], [8, 0], [8, 0.2], [0, 0.2]]], {'operation_width': 0.7}),
    ([[[0, 0], [8, 0], [8, 8], [0, 8]]], {'default_swath_angle': 3.5}),
])
def test_direct_start_and_bridge_preparation_share_editor_geometry_rejection(
        coverage_system, polygons, planner):
    _, status, call, boundary = coverage_system
    request = json.dumps({'frame_id': 'map', 'polygons': polygons, 'planner': planner})
    direct = call(StartTask, '/task_control/start', StartTask.Request(
        task_type='coverage', request_json=request))
    assert not direct.accepted and direct.message
    prepared = call(PrepareCoverage, '/coverage/prepare', PrepareCoverage.Request(
        goal_id=uuid.uuid4().hex, task_id='invalid-region', config_json=request))
    assert not prepared.accepted and prepared.message
    assert not status()['active'] and not boundary['requests']


def test_authoritative_headland_rejects_region_before_planning(coverage_system):
    _, status, call, boundary = coverage_system
    started = call(StartTask, '/task_control/start', StartTask.Request(
        task_type='coverage', request_json=json.dumps({
            'frame_id': 'map', 'polygons': [[[0, 0], [8, 0], [8, 0.8], [0, 0.8]]],
            'planner': {'operation_width': 0.7}})))
    assert started.accepted  # Region fits the brush, but not the leased headland.
    ended = wait_until(lambda: value if not (value := status())['active'] else None)
    assert ended['result'] == 'failed' and 'headland' in ended['error'], ended
    assert not boundary['requests'] and ended['cleanup_confirmed']


def test_failure_detail_arriving_after_action_result_is_reported(
        delay_failure_telemetry, coverage_system):
    _, status, call, boundary = coverage_system
    started = call(StartTask, '/task_control/start', StartTask.Request(
        task_type='coverage', request_json=json.dumps({
            'frame_id': 'map', 'polygons': [[[0, 0], [8, 0], [8, 0.8], [0, 0.8]]],
            'planner': {'operation_width': 0.7}})))
    assert started.accepted
    stopping = wait_until(lambda: value if (value := status())['phase'] == 'stopping'
                          else None)
    assert stopping['active'] and boundary['stop_state']['stop_requested']
    delay_failure_telemetry.set()
    ended = wait_until(lambda: value if not (value := status())['active'] else None)
    assert ended['result'] == 'failed' and 'headland' in ended['error'], ended
    assert ended['cleanup_confirmed'] and not boundary['requests']


def test_failure_without_telemetry_finishes_with_generic_error(
        delay_failure_telemetry, coverage_system):
    _, status, call, boundary = coverage_system
    started = call(StartTask, '/task_control/start', StartTask.Request(
        task_type='coverage', request_json=json.dumps({
            'frame_id': 'map', 'polygons': [[[0, 0], [8, 0], [8, 0.8], [0, 0.8]]],
            'planner': {'operation_width': 0.7}})))
    assert started.accepted
    ended = wait_until(lambda: value if not (value := status())['active'] else None)
    assert ended['result'] == 'failed' and 'coverage error 905' in ended['error'], ended
    assert ended['cleanup_confirmed'] and boundary['stop_state']['stop_requested']
    assert not delay_failure_telemetry.is_set()


@pytest.mark.parametrize('coverage_system', [{'generate_headland': False}], indirect=True)
def test_disabled_headland_does_not_apply_an_unrequested_margin(coverage_system):
    _, status, call, boundary = coverage_system
    for pose, (x, y) in zip(boundary['path'].poses, ((1., .4), (2., .4), (3., .4), (4., .4))):
        pose.pose.position.x, pose.pose.position.y = x, y
    started = call(StartTask, '/task_control/start', StartTask.Request(
        task_type='coverage', request_json=json.dumps({
            'frame_id': 'map', 'polygons': [[[0, 0], [8, 0], [8, 0.8], [0, 0.8]]],
            'planner': {'operation_width': 0.7}})))
    assert started.accepted
    ended = wait_until(lambda: value if not (value := status())['active'] else None)
    assert ended['result'] == 'succeeded' and ended['cleanup_confirmed'], ended
    computed = next(req for kind, req in boundary['goals'] if kind == 'compute')
    assert not computed.generate_headland

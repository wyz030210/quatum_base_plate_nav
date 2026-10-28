"""Site inspection through generated ROS services and the actual Nav2 client."""
import json
from pathlib import Path
import time

import pytest
import yaml
from nav2_mission_interfaces.srv import CancelTask, GetTaskStatus, StartTask

from test_inspection_tasks import site_request
import test_task_control_ros as ros_boundaries

ros_system = ros_boundaries.ros_system


def status(call):
    return json.loads(call(GetTaskStatus, '/task_control/status',
                           GetTaskStatus.Request()).status_json)


def wait_status(call, predicate):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        observed = status(call)
        if predicate(observed):
            return observed
        time.sleep(0.02)
    pytest.fail(f'task status did not converge: {observed}')


def start(call, request):
    req = StartTask.Request(task_type='inspection', request_json=json.dumps(request))
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        reply = call(StartTask, '/task_control/start', req)
        if reply.accepted:
            return reply
        time.sleep(0.02)
    pytest.fail(f'start rejected: {reply.message}')


def test_ros_site_template_runs_all_four_arrival_actions_with_vehicle_default_tree(
        ros_system, tmp_path):
    call, goals, commands, boundary = ros_system
    request = site_request(tmp_path, kinds=('security', 'facility', 'environment', 'dock'))
    first = start(call, request)
    ended = wait_status(call, lambda s: not s['active'])
    assert ended['result'] == 'succeeded' and ended['task_id'] == first.task_id
    assert [goal.pose.pose.position.x for goal in goals] == [31.25, 32.25, 33.25, 34.25]
    assert all(goal.behavior_tree == '' and goal.pose.header.frame_id == 'map' for goal in goals)
    report = json.loads(Path(ended['report_path']).read_text())
    assert report['details']['succeeded'] == 4 and report['details']['failed'] == 0
    assert report['details']['points'][3]['charging_verified'] is False
    assert commands[-1] is True and report['cleanup_confirmed']


@pytest.mark.parametrize('optional', [True, False])
def test_ros_navigation_failure_applies_template_skip_rule_without_retry(
        ros_system, tmp_path, optional):
    call, goals, commands, boundary = ros_system
    boundary['outcomes'] = [False, True]
    request = site_request(tmp_path, optional=(0,) if optional else ())
    start(call, request)
    ended = wait_status(call, lambda s: not s['active'])
    assert ended['result'] == ('succeeded' if optional else 'failed')
    assert len(goals) == (2 if optional else 1)
    record = ended['details']['points'][0]
    assert record['status'] == ('skipped' if optional else 'failed')
    assert record['inspection_result'] == 'not_run' and record['error'] == 'Nav2 status 6'


@pytest.mark.parametrize('phase', ['pending_accept', 'inspecting'])
def test_ros_optional_template_cancel_stays_responsive_and_never_visits_next(
        ros_system, tmp_path, phase):
    call, goals, commands, boundary = ros_system
    request = site_request(tmp_path, optional=(0, 1))
    if phase == 'pending_accept':
        boundary.update(accept_delay=0.4, execute_time=2.0)
    else:
        path = Path(request['points_file'])
        points = yaml.safe_load(path.read_text())
        points['points'][0]['stay_time'] = 60.0
        path.write_text(yaml.safe_dump(points))
    first = start(call, request)
    if phase == 'pending_accept':
        deadline = time.monotonic() + 3
        while not boundary['requests'] and time.monotonic() < deadline:
            time.sleep(0.01)
        assert boundary['requests']
    else:
        wait_status(call, lambda s: s['phase'] == 'inspecting')
    before = time.monotonic()
    cancel = call(CancelTask, '/task_control/cancel', CancelTask.Request(task_id=first.task_id))
    assert cancel.accepted and time.monotonic() - before < 1.0
    ended = wait_status(call, lambda s: not s['active'])
    assert ended['result'] == 'canceled' and ended['cleanup_confirmed']
    assert len(goals) == 1 and ended['details']['canceled'] == 1
    assert ended['details']['skipped'] == 0 and ended['details']['unvisited'] == 1
    if phase == 'pending_accept':
        assert len(boundary['canceled']) == 1
    report = json.loads(Path(ended['report_path']).read_text())
    assert report['task_id'] == first.task_id and report['details'] == ended['details']


def test_ros_rejects_unsupported_source_tree_before_accepting_task(ros_system, tmp_path):
    call, goals, commands, boundary = ros_system
    request = site_request(tmp_path)
    path = Path(request['missions_file'])
    missions = yaml.safe_load(path.read_text())
    missions['missions'][0]['nav_bt'] = 'inspection_nav_to_pose.xml'
    path.write_text(yaml.safe_dump(missions))
    reply = call(StartTask, '/task_control/start', StartTask.Request(
        task_type='inspection', request_json=json.dumps(request)))
    assert not reply.accepted and 'nav_bt' in reply.message
    assert goals == [] and status(call)['phase'] == 'idle'

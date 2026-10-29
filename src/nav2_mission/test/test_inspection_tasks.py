"""Template inspection behavior at unified start/cancel/status boundaries."""
import json
from pathlib import Path
from dataclasses import replace

import yaml
import pytest

from nav2_mission.task_adapters import ExecutionUpdate
from test_task_control import system, tick


def site_request(tmp_path, *, kinds=('security', 'dock'), optional=()):
    points = [dict(id=f'site_{index}', name=f'现场点 {index}', type=kind,
                   pose=dict(x=31.25 + index, y=-17.5, yaw=0.75),
                   stay_time=0.0, allow_skip=index in optional)
              for index, kind in enumerate(kinds)]
    missions = [dict(id='site_patrol', name='现场巡检',
                     points=[point['id'] for point in points], nav_bt='',
                     stop_on_required_point_failure=True)]
    points_file = tmp_path / 'site_points.yaml'
    missions_file = tmp_path / 'site_missions.yaml'
    points_file.write_text(yaml.safe_dump(dict(points=points)), encoding='utf-8')
    missions_file.write_text(yaml.safe_dump(dict(missions=missions)), encoding='utf-8')
    return dict(frame_id='map', points_file=str(points_file),
                missions_file=str(missions_file), mission_id='site_patrol')


def finish_motion(manager, clock, nav, index, *, succeeded=True, error=''):
    nav.motions[index].update = ExecutionUpdate('navigating', True, succeeded, error)
    tick(manager, clock)


def test_site_template_runs_in_order_and_keeps_source_fields_and_point_summary(tmp_path):
    manager, clock, nav, stop = system(tmp_path / 'reports')
    request = site_request(tmp_path, kinds=('security', 'facility', 'environment', 'dock'))
    reply = manager.start('inspection', request)
    assert reply['accepted'], reply
    # Files are frozen during request acceptance, not reread between points.
    Path(request['points_file']).write_text('points: []')
    for index in range(4):
        tick(manager, clock)
        assert [goal[1]['id'] for goal in nav.goals] == [f'site_{i}' for i in range(index + 1)]
        assert nav.goals[index][1]['x'] == 31.25 + index
        assert nav.goals[index][1]['yaw'] == 0.75
        finish_motion(manager, clock, nav, index)
    assert manager.status()['phase'] == 'stopping'
    stop.confirm()
    manager.tick()
    status = manager.status()
    report = json.loads(Path(status['report_path']).read_text())
    assert report['task_id'] == reply['task_id'] and report['result'] == 'succeeded'
    points = report['details']['points']
    assert [point['point_type'] for point in points] == [
        'security', 'facility', 'environment', 'dock']
    assert all(point['simulated'] and point['task_id'] == reply['task_id'] for point in points)
    assert points[0]['point_name'] == '现场点 0'
    assert [point['message'] for point in points] == [
        '模拟安防巡检正常', '模拟拍照及设备状态检查正常',
        '模拟环境卫生检查正常', '到达回充点，任务收尾']
    assert points[3]['charging_verified'] is False
    assert report['details']['succeeded'] == 4
    assert report['details']['failed'] == report['details']['skipped'] == 0


@pytest.mark.parametrize('optional', [True, False])
def test_navigation_failure_skips_optional_point_or_aborts_required_point(tmp_path, optional):
    manager, clock, nav, stop = system(tmp_path / 'reports')
    request = site_request(tmp_path, optional=(0,) if optional else ())
    assert manager.start('inspection', request)['accepted']
    tick(manager, clock)
    finish_motion(manager, clock, nav, 0, succeeded=False, error='path blocked')
    if optional:
        tick(manager, clock)
        assert [goal[1]['id'] for goal in nav.goals] == ['site_0', 'site_1']
        finish_motion(manager, clock, nav, 1)
    else:
        assert len(nav.goals) == 1
    stop.confirm()
    manager.tick()
    status = manager.status()
    assert status['result'] == ('succeeded' if optional else 'failed')
    report = json.loads(Path(status['report_path']).read_text())
    assert report['details']['points'][0]['status'] == ('skipped' if optional else 'failed')
    assert report['details']['points'][0]['inspection_result'] == 'not_run'
    assert report['details']['points'][0]['message'] == 'path blocked'
    assert report['details']['succeeded'] == (1 if optional else 0)
    assert report['details']['skipped'] == (1 if optional else 0)
    assert report['details']['failed'] == (0 if optional else 1)


@pytest.mark.parametrize('phase', ['navigating', 'inspecting', 'between_points'])
def test_cancel_ends_entire_optional_mission_and_reports_without_dispatching_next(tmp_path, phase):
    manager, clock, nav, stop = system(tmp_path / 'reports')
    request = site_request(tmp_path, optional=(0, 1))
    if phase == 'inspecting':
        points = yaml.safe_load(Path(request['points_file']).read_text())
        points['points'][0]['stay_time'] = 60.0
        Path(request['points_file']).write_text(yaml.safe_dump(points))
    task_id = manager.start('inspection', request)['task_id']
    tick(manager, clock)
    if phase != 'navigating':
        finish_motion(manager, clock, nav, 0)
    if phase == 'inspecting':
        assert manager.status()['phase'] == 'inspecting'
    assert manager.cancel(task_id)['accepted']
    if phase == 'navigating':
        stop.confirm()
        manager.tick()
        assert manager.status()['active']
        nav.motions[0].update = ExecutionUpdate('navigating', True, False, 'Nav2 canceled')
    stop.confirm()
    manager.tick()
    status = manager.status()
    assert status['result'] == 'canceled' and status['cleanup_confirmed'], status
    report = json.loads(Path(status['report_path']).read_text())
    assert len(nav.goals) == 1
    assert report['details']['skipped'] == report['details']['failed'] == 0
    if phase == 'between_points':
        assert report['details']['succeeded'] == 1
    else:
        assert report['details']['canceled'] == 1
        assert report['details']['points'][0]['status'] == 'canceled'
        assert report['details']['points'][0]['inspection_result'] == (
            'canceled' if phase == 'inspecting' else 'not_run')


@pytest.mark.parametrize('reason', ['manual_takeover', 'localization_lost'])
def test_health_loss_overrides_optional_failure_and_never_resumes(tmp_path, reason):
    from nav2_mission.task_adapters import LocalizationState
    manager, clock, nav, stop = system(tmp_path / 'reports')
    request = site_request(tmp_path, optional=(0, 1))
    manager.start('inspection', request)
    tick(manager, clock)
    if reason == 'manual_takeover':
        stop.state = replace(stop.state, manual_override=True, ready=False)
    else:
        manager.localization.snapshot = lambda now: LocalizationState(
            False, 'map', now, 0.0, 'source lost')
    # Health changes and a point failure arrive in the same public control cycle.
    nav.motions[0].update = ExecutionUpdate('navigating', True, False, 'path blocked')
    tick(manager, clock)
    stop.confirm()
    manager.tick()
    status = manager.status()
    assert status['result'] == 'failed' and not status['active']
    assert len(nav.goals) == 1
    assert status['details']['skipped'] == 0
    assert status['details']['points'][0]['status'] == 'canceled'
    # No automatic continuation after the external provider recovers.
    clock.now += 0.1
    manager.tick()
    assert len(nav.goals) == 1


@pytest.mark.parametrize('optional', [True, False])
def test_arrival_action_failure_is_skipped_or_aborts_and_preserves_raw_error(tmp_path, optional):
    from nav2_mission.inspection import Inspection
    from nav2_mission.task_control import TaskManager
    from test_task_control import Clock, Localization, Motion, Navigation, Stop

    class ArrivalDevice:
        def __init__(self):
            self.visited = []

        def start(self, task_id, point, now):
            self.visited.append(point['id'])
            operation = Motion()
            operation.update = ExecutionUpdate('inspecting', True, False, 'device error 42',
                                               {'message': 'simulated device unavailable'})
            return operation

    clock = Clock()
    nav = Navigation()
    stop = Stop(clock)
    device = ArrivalDevice()
    manager = TaskManager(nav, Localization(), stop, tmp_path / 'reports', clock=clock,
                          handlers={'inspection': Inspection(nav, actions=device)})
    request = site_request(tmp_path, optional=(0,) if optional else ())
    manager.start('inspection', request)
    tick(manager, clock)
    assert device.visited == []
    finish_motion(manager, clock, nav, 0)
    assert device.visited == ['site_0']
    if optional:
        tick(manager, clock)
        finish_motion(manager, clock, nav, 1, succeeded=False, error='dock blocked')
    stop.confirm()
    manager.tick()
    status = manager.status()
    assert status['result'] == 'failed'
    first = status['details']['points'][0]
    assert first['status'] == ('skipped' if optional else 'failed')
    assert first['inspection_result'] == 'failed' and first['error'] == 'device error 42'
    assert first['message'] == 'simulated device unavailable'
    if not optional:
        assert status['error'] == 'device error 42' and len(nav.goals) == 1


@pytest.mark.parametrize(('field', 'value', 'error'), [
    ('id', '../escape', 'point id'),
    ('id', 'a' * 65, 'point id'),
    ('name', None, 'point name'),
    ('type', 'unknown', 'point type'),
    ('stay_time', -0.1, 'stay_time'),
    ('stay_time', True, 'finite number'),
    ('stay_time', 10 ** 400, 'finite number'),
    ('allow_skip', 'true', 'allow_skip'),
    ('pose', {'x': float('nan'), 'y': 0.0, 'yaw': 0.0}, 'finite number'),
    ('pose', {'x': float('inf'), 'y': 0.0, 'yaw': 0.0}, 'finite number'),
    ('pose', {'x': -float('inf'), 'y': 0.0, 'yaw': 0.0}, 'finite number'),
    ('pose', {'x': True, 'y': 0.0, 'yaw': 0.0}, 'finite number'),
    ('pose', {'x': 0.0, 'y': 0.0}, 'yaw'),
    ('reserved', True, 'unsupported'),
])
def test_source_point_validation_rejects_before_navigation(tmp_path, field, value, error):
    manager, clock, nav, stop = system(tmp_path / 'reports')
    request = site_request(tmp_path)
    path = Path(request['points_file'])
    data = yaml.safe_load(path.read_text())
    data['points'][0][field] = value
    path.write_text(yaml.safe_dump(data))
    reply = manager.start('inspection', request)
    assert not reply['accepted'] and error in reply['message'], reply
    assert manager.status()['phase'] == 'idle' and nav.goals == []


@pytest.mark.parametrize(('field', 'value', 'error'), [
    ('id', '../escape', 'mission id'),
    ('name', '', 'mission name'),
    ('points', [], 'non-empty'),
    ('points', ['missing'], 'unknown point'),
    ('points', ['site_0', 'site_0'], 'duplicate point'),
    ('nav_bt', 'inspection_nav_to_pose.xml', 'nav_bt'),
    ('stop_on_required_point_failure', False, 'stop_on_required_point_failure'),
    ('stop_on_required_point_failure', 1, 'stop_on_required_point_failure'),
    ('reserved', True, 'unsupported'),
])
def test_source_template_validation_rejects_before_navigation(tmp_path, field, value, error):
    manager, clock, nav, stop = system(tmp_path / 'reports')
    request = site_request(tmp_path)
    path = Path(request['missions_file'])
    data = yaml.safe_load(path.read_text())
    data['missions'][0][field] = value
    if field == 'id':
        request['mission_id'] = value
    path.write_text(yaml.safe_dump(data))
    reply = manager.start('inspection', request)
    assert not reply['accepted'] and error in reply['message'], reply
    assert nav.goals == []


@pytest.mark.parametrize('missing', ['name', 'allow_skip'])
def test_source_site_point_requires_named_action_and_skip_rule(tmp_path, missing):
    manager, clock, nav, stop = system(tmp_path / 'reports')
    request = site_request(tmp_path)
    path = Path(request['points_file'])
    data = yaml.safe_load(path.read_text())
    del data['points'][0][missing]
    path.write_text(yaml.safe_dump(data))
    assert not manager.start('inspection', request)['accepted']
    assert nav.goals == []


def test_source_text_is_trimmed_and_yaml_is_loaded_safely(tmp_path):
    manager, clock, nav, stop = system(tmp_path / 'reports')
    request = site_request(tmp_path)
    path = Path(request['points_file'])
    data = yaml.safe_load(path.read_text())
    data['points'][0]['id'] = ' site_0 '
    data['points'][0]['name'] = '  现场门岗  '
    path.write_text(yaml.safe_dump(data))
    assert manager.start('inspection', request)['accepted']
    tick(manager, clock)
    assert nav.goals[0][1]['id'] == 'site_0' and nav.goals[0][1]['name'] == '现场门岗'
    manager.cancel()
    nav.motions[0].update = ExecutionUpdate('navigating', True, False, 'canceled')
    stop.confirm()
    manager.tick()
    path.write_text("!!python/object/apply:builtins.eval ['40 + 2']\n")
    reply = manager.start('inspection', request)
    assert not reply['accepted'] and 'configuration' in reply['message']


def test_cancel_before_navigation_keeps_empty_point_summary_for_the_accepted_template(tmp_path):
    manager, clock, nav, stop = system(tmp_path / 'reports')
    request = site_request(tmp_path)
    task_id = manager.start('inspection', request)['task_id']
    manager.cancel(task_id)
    stop.confirm()
    manager.tick()
    status = manager.status()
    assert status['result'] == 'canceled' and nav.goals == []
    report = json.loads(Path(status['report_path']).read_text())
    assert report['details'] == {'points': [], 'succeeded': 0, 'failed': 0, 'skipped': 0,
                                 'canceled': 0, 'total_points': 2, 'unvisited': 2}


@pytest.mark.parametrize('key', ['points', 'missions'])
@pytest.mark.parametrize('problem', ['duplicate', 'empty', 'missing_file'])
def test_source_documents_reject_duplicates_empty_or_missing_files(tmp_path, key, problem):
    manager, clock, nav, stop = system(tmp_path / 'reports')
    request = site_request(tmp_path)
    path = Path(request[key + '_file'])
    data = yaml.safe_load(path.read_text())
    if problem == 'missing_file':
        path.unlink()
    else:
        data[key] = [data[key][0], data[key][0]] if problem == 'duplicate' else []
        path.write_text(yaml.safe_dump(data))
    reply = manager.start('inspection', request)
    assert not reply['accepted'] and reply['message']
    assert nav.goals == [] and stop.state.stop_requested


def test_source_unknown_selected_mission_is_rejected(tmp_path):
    manager, clock, nav, stop = system(tmp_path / 'reports')
    request = site_request(tmp_path)
    request['mission_id'] = 'missing'
    reply = manager.start('inspection', request)
    assert not reply['accepted'] and 'unknown mission' in reply['message']
    assert nav.goals == []


def test_template_report_write_failure_keeps_queryable_point_outcomes(tmp_path):
    occupied = tmp_path / 'occupied'
    occupied.write_text('not a directory')
    manager, clock, nav, stop = system(occupied / 'reports')
    request = site_request(tmp_path)
    assert manager.start('inspection', request)['accepted']
    for index in range(2):
        tick(manager, clock)
        finish_motion(manager, clock, nav, index)
    stop.confirm()
    manager.tick()
    status = manager.status()
    assert status['result'] == 'failed' and status['execution_result'] == 'succeeded'
    assert status['report_error'] and status['report_path'] == ''
    assert status['details']['succeeded'] == 2 and status['cleanup_confirmed']


def test_shipped_site_template_requires_surveyed_coordinates_before_use(tmp_path):
    manager, clock, nav, stop = system(tmp_path / 'reports')
    configs = Path(__file__).parents[1] / 'config'
    request = dict(frame_id='map', mission_id='site_patrol',
                   points_file=str(configs / 'inspection_site_points.template.yaml'),
                   missions_file=str(configs / 'inspection_site_missions.template.yaml'))
    reply = manager.start('inspection', request)
    assert not reply['accepted'] and 'finite number' in reply['message']
    assert nav.goals == []

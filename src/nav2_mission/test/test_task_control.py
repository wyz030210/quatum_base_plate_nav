"""Observable task lifecycle at the approved start/cancel/status seam."""
import json
from pathlib import Path
from dataclasses import replace

from nav2_mission.task_control import TaskManager
from nav2_mission.task_adapters import (
    ExecutionUpdate, LocalizationState, StopState,
)


class Clock:
    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now


class Localization:
    def snapshot(self, now):
        return LocalizationState(True, 'map', now, 0.0, source='fake_amcl')


class Stop:
    def __init__(self, clock):
        self.clock = clock
        self.state = StopState(True, True, True, clock(), 0.0)

    def snapshot(self, now):
        return self.state

    def authorize(self, task_id):
        self.state = StopState(True, False, False, self.clock(), 0.0)

    def request_stop(self, task_id):
        pass

    def confirm(self):
        self.clock.now += 0.01
        self.state = StopState(True, True, True, self.clock(), 0.0)


class Motion:
    def __init__(self):
        self.update = ExecutionUpdate('navigating')
        self.cancel_requested = False

    def poll(self, now):
        return self.update

    def cancel(self):
        self.cancel_requested = True


class Navigation:
    def __init__(self):
        self.goals = []
        self.motions = []

    def start(self, task_id, point, frame_id):
        self.goals.append((task_id, point, frame_id))
        op = Motion()
        self.motions.append(op)
        return op


REQUEST = {'frame_id': 'map', 'point': {
    'id': 'gate', 'type': 'security', 'x': 1.0, 'y': 2.0, 'yaw': 0.2,
    'stay_time': 2.0, 'allow_skip': False,
}}


def system(tmp_path):
    clock = Clock()
    navigation = Navigation()
    stop = Stop(clock)
    manager = TaskManager(navigation, Localization(), stop, tmp_path, clock=clock)
    return manager, clock, navigation, stop


def tick(manager, clock):
    clock.now += 0.1
    manager.tick()


def test_single_point_inspection_waits_for_navigation_dwell_and_fresh_stop(tmp_path):
    manager, clock, nav, stop = system(tmp_path)
    reply = manager.start('inspection', REQUEST)
    assert reply['accepted'] and reply['task_id']
    tick(manager, clock)
    assert manager.status()['phase'] == 'navigating'
    assert nav.goals[0][1]['id'] == 'gate'
    nav.motions[0].update = ExecutionUpdate('navigating', done=True, succeeded=True)
    tick(manager, clock)
    assert manager.status()['phase'] == 'inspecting'
    clock.now += 2.1
    stop.state = replace(stop.state, received_at=clock())
    manager.tick()
    assert manager.status()['phase'] == 'stopping'
    assert manager.status()['result'] is None
    stop.confirm()
    manager.tick()
    status = manager.status()
    assert status['result'] == 'succeeded'
    report = json.loads(Path(status['report_path']).read_text())
    assert report['task_id'] == reply['task_id']
    assert report['simulated'] is True
    assert report['phase'] == 'finished'
    from datetime import datetime
    assert datetime.fromisoformat(report['started_utc']).utcoffset().total_seconds() == 0
    assert datetime.fromisoformat(report['finished_utc']).utcoffset().total_seconds() == 0
    assert report['elapsed_sec'] >= 2.0
    assert report['details']['points'][0]['simulated'] is True


def test_start_rejects_when_vehicle_has_not_confirmed_initial_stop(tmp_path):
    manager, clock, nav, stop = system(tmp_path)
    stop.state = replace(stop.state, stopped=False)
    assert manager.start('inspection', REQUEST)['accepted'] is False
    assert nav.goals == []


def test_cancel_waits_for_owned_pending_motion_and_new_stop_not_cancel_ack(tmp_path):
    manager, clock, nav, stop = system(tmp_path)
    task_id = manager.start('inspection', REQUEST)['task_id']
    tick(manager, clock)
    assert manager.start('inspection', REQUEST)['accepted'] is False
    assert manager.cancel(task_id)['accepted']
    assert manager.cancel(task_id)['accepted']
    assert nav.motions[0].cancel_requested
    stop.confirm()
    manager.tick()
    assert manager.status()['active']
    assert not manager.start('inspection', REQUEST)['accepted']
    nav.motions[0].update = ExecutionUpdate('navigating', done=True, error='canceled')
    manager.tick()
    assert manager.status()['result'] == 'canceled'
    assert manager.status()['error'] == 'user canceled'
    assert manager.cancel(task_id)['accepted']
    next_task = manager.start('inspection', REQUEST)
    assert next_task['accepted'] and next_task['task_id'] != task_id
    # A terminal response belonging to the old operation cannot change the next run.
    nav.motions[0].update = ExecutionUpdate('navigating', done=True, succeeded=True)
    tick(manager, clock)
    assert manager.status()['task_id'] == next_task['task_id']
    assert manager.status()['phase'] == 'navigating'
    assert len(nav.goals) == 2


def test_cancel_during_simulated_dwell_ends_task_and_does_not_resume(tmp_path):
    manager, clock, nav, stop = system(tmp_path)
    task_id = manager.start('inspection', REQUEST)['task_id']
    tick(manager, clock)
    nav.motions[0].update = ExecutionUpdate('navigating', True, True)
    tick(manager, clock)
    assert manager.status()['phase'] == 'inspecting'
    manager.cancel(task_id)
    stop.confirm()
    manager.tick()
    assert manager.status()['result'] == 'canceled'
    clock.now += 100
    manager.tick()
    assert len(nav.goals) == 1 and not manager.status()['active']


def test_old_zero_stop_sample_and_cancel_timeout_never_release_task(tmp_path):
    manager, clock, nav, stop = system(tmp_path)
    manager.start('inspection', REQUEST)
    tick(manager, clock)
    manager.cancel()
    nav.motions[0].update = ExecutionUpdate('navigating', True, False, 'canceled')
    stop.state = StopState(True, True, True, clock() - 0.01, 0.0)
    manager.tick()
    assert manager.status()['active']
    clock.now += 6
    manager.tick()
    assert manager.status()['active']
    assert 'cancel_timeout' in manager.status()['cleanup_error']
    assert not manager.start('inspection', REQUEST)['accepted']
    stop.confirm()
    manager.tick()
    assert manager.status()['result'] == 'canceled'


def test_unwritable_report_is_queryable_failure_after_safe_cleanup(tmp_path):
    manager, clock, nav, stop = system(tmp_path / 'not-a-directory' / 'reports')
    (tmp_path / 'not-a-directory').write_text('occupied')
    manager.start('inspection', REQUEST)
    manager.cancel()
    stop.confirm()
    manager.tick()
    status = manager.status()
    assert status['result'] == 'failed' and status['report_error']
    assert status['cleanup_confirmed'] and not status['active']


def test_navigation_failure_never_runs_simulated_arrival_action(tmp_path):
    manager, clock, nav, stop = system(tmp_path)
    manager.start('inspection', REQUEST)
    tick(manager, clock)
    nav.motions[0].update = ExecutionUpdate('navigating', True, False, 'path blocked')
    tick(manager, clock)
    stop.confirm()
    manager.tick()
    report = json.loads(Path(manager.status()['report_path']).read_text())
    assert report['result'] == 'failed' and report['error'] == 'path blocked'
    assert 'inspecting' not in report['phases']


def test_frozen_request_and_bad_options_are_observable_through_status(tmp_path):
    import copy
    manager, clock, nav, stop = system(tmp_path)
    bad = dict(REQUEST, reserved_option=True)
    assert not manager.start('inspection', bad)['accepted']
    request = copy.deepcopy(REQUEST)
    manager.start('inspection', request)
    request['point']['x'] = 999.0
    tick(manager, clock)
    assert manager.status()['config']['point']['x'] == 1.0
    assert nav.goals[0][1]['x'] == 1.0


def test_navigation_transport_exception_requests_stop_and_remains_blocked(tmp_path):
    manager, clock, nav, stop = system(tmp_path)
    manager.start('inspection', REQUEST)
    tick(manager, clock)

    def transport_failure(now):
        raise RuntimeError('action transport lost')

    nav.motions[0].poll = transport_failure
    tick(manager, clock)
    status = manager.status()
    assert status['phase'] == 'stopping' and 'transport lost' in status['error']
    stop.confirm()
    manager.tick()
    assert manager.status()['active']


def test_emergency_stop_records_outcome_after_parking_and_keeps_new_starts_blocked(tmp_path):
    manager, clock, nav, stop = system(tmp_path)
    manager.start('inspection', REQUEST)
    tick(manager, clock)
    stop.state = replace(stop.state, emergency_stop=True, ready=False, received_at=clock())
    manager.tick()
    assert manager.status()['phase'] == 'stopping'
    nav.motions[0].update = ExecutionUpdate('navigating', True, False, 'canceled')
    stop.confirm()
    stop.state = replace(stop.state, emergency_stop=True, ready=False)
    manager.tick()
    assert manager.status()['result'] == 'failed'
    assert manager.status()['error'] == 'safety invalid or manual/emergency stop'
    assert manager.status()['cleanup_confirmed'] and not manager.status()['active']
    assert not manager.start('inspection', REQUEST)['accepted']


def test_localization_adapter_exception_is_queryable_and_enters_safe_cleanup(tmp_path):
    manager, clock, nav, stop = system(tmp_path)
    manager.start('inspection', REQUEST)
    tick(manager, clock)

    def broken_localization(now):
        raise RuntimeError('localization connection lost')

    manager.localization.snapshot = broken_localization
    tick(manager, clock)
    assert manager.status()['phase'] == 'stopping'
    assert 'localization connection lost' in manager.status()['error']


def test_external_stop_during_navigation_ends_the_automatic_task(tmp_path):
    manager, clock, nav, stop = system(tmp_path)
    manager.start('inspection', REQUEST)
    tick(manager, clock)
    stop.state = replace(stop.state, stop_requested=True, received_at=clock())
    manager.tick()
    assert manager.status()['phase'] == 'stopping'
    assert manager.status()['error'] == 'external stop requested'


def test_direct_point_skip_rule_is_supported_without_retrying_failed_navigation(tmp_path):
    import copy
    manager, clock, nav, stop = system(tmp_path)
    request = copy.deepcopy(REQUEST)
    request['point']['allow_skip'] = True
    assert manager.start('inspection', request)['accepted']
    tick(manager, clock)
    nav.motions[0].update = ExecutionUpdate('navigating', True, False, 'path blocked')
    tick(manager, clock)
    stop.confirm()
    manager.tick()
    status = manager.status()
    assert status['result'] == 'succeeded'
    assert status['details']['skipped'] == 1 and status['details']['succeeded'] == 0
    assert len(nav.goals) == 1


def test_task_freezes_localization_context_without_mutating_handler_config(tmp_path):
    manager, clock, nav, stop = system(tmp_path)
    state = LocalizationState(True, 'map', clock(), 0.0, source='amcl',
                              x=1.0, y=2.0, yaw=0.0, map_id='map-A',
                              localization_profile='indoor-A', evidence_generation=1,
                              schema_version=1)
    manager.localization.snapshot = lambda now: replace(state, received_at=now)
    request = dict(REQUEST, map_id='map-A', localization_profile='indoor-A')
    assert manager.start('inspection', request)['accepted']
    assert 'map_id' not in manager.status()['config']
    tick(manager, clock)
    state = replace(state, evidence_generation=2)
    tick(manager, clock)
    assert manager.status()['phase'] == 'stopping'
    assert 'context changed' in manager.status()['error']
    assert nav.motions[0].cancel_requested

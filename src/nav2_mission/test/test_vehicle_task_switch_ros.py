"""Switch through real manager, localization, bridge and common command gate."""
import json
import time

import pytest
from nav2_mission_interfaces.srv import CancelTask, StartTask
from test_localization_task_ros import wait_for

pytest_plugins = ['test_localization_task_ros']


@pytest.mark.parametrize('complete_chain', [('amcl', 'inspection'), ('rtk_ekf', 'inspection')],
                         indirect=True)
def test_inspection_coverage_and_ordinary_navigation_switch_only_after_stop(complete_chain):
    io = complete_chain
    boundary, flags = io['boundary'], io['flags']

    def submit(task, config):
        return io['call'](StartTask, '/task_control/start', StartTask.Request(
            task_type=task, request_json=json.dumps(config)))

    def stop_and_wait(task_id):
        flags['linear'] = 0.2
        wait_for(lambda: boundary['safety'] and not boundary['safety'][-1]['stopped'])
        result = io['call'](CancelTask, '/task_control/cancel',
                            CancelTask.Request(task_id=task_id))
        assert result.accepted
        wait_for(lambda: io['status']()['phase'] == 'stopping')
        assert io['status']()['active']
        boundary['release_terminal'].set()
        time.sleep(0.2)
        assert io['status']()['active']
        flags['linear'] = 0.0
        ended = wait_for(lambda: (s if not (s := io['status']())['active'] else None))
        assert ended['result'] == 'canceled' and ended['cleanup_confirmed'], ended
        return ended

    inspection = io['start']()
    assert inspection.accepted, inspection.message
    wait_for(lambda: any(stage == 'navigate' for stage, _ in boundary['goals']))
    coverage_config = {'frame_id': 'map', 'polygons': [[[0, 0], [8, 0], [8, 8], [0, 8]]]}
    assert not submit('coverage', coverage_config).accepted
    stop_and_wait(inspection.task_id)

    boundary['hold'] = 'follow'
    boundary['release_terminal'].clear()
    coverage = submit('coverage', coverage_config)
    assert coverage.accepted, coverage.message
    wait_for(lambda: any(stage == 'follow' for stage, _ in boundary['goals']))
    navigation_config = {'frame_id': 'map', 'point': {'x': 3.0, 'y': 2.0, 'yaw': 0.0}}
    assert not submit('navigation', navigation_config).accepted
    stop_and_wait(coverage.task_id)
    assert len(boundary['checks']) >= 2

    boundary['hold'] = ''
    ordinary = submit('navigation', navigation_config)
    assert ordinary.accepted, ordinary.message
    ended = wait_for(lambda: (s if not (s := io['status']())['active'] else None))
    assert ended['task_id'] == ordinary.task_id and ended['result'] == 'succeeded'
    assert ended['cleanup_confirmed'] and not ended['simulated']
    assert not boundary['initialposes']

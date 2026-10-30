"""Exercise the evidence command at its process and ROS boundaries."""
import json
import os
from pathlib import Path
import subprocess
import sys


SCRIPT = Path(__file__).parents[1] / 'nav2_mission' / 'field_preflight.py'


def run_preflight(path, *args):
    env = dict(os.environ, PATH=str(path), PYTHONDONTWRITEBYTECODE='1')
    return subprocess.run([sys.executable, str(SCRIPT), *args], env=env,
                          text=True, capture_output=True, check=True)


def test_missing_ros_is_explicit_and_field_cases_remain_unexecuted(tmp_path):
    record = json.loads(run_preflight(tmp_path).stdout)
    assert record['ros_graph']['status'] == 'unavailable'
    assert record['localization']['observed'] is None
    assert record['safety']['observed'] is None
    assert all(case['status'] == 'not_run' for case in record['field_cases'])
    assert record['vehicle']['measured'] is None


def test_default_graph_inspection_uses_only_read_operations(tmp_path):
    log = tmp_path / 'calls'
    ros = tmp_path / 'ros2'
    ros.write_text("#!/bin/sh\n"
                   f"printf '%s\\n' \"$*\" >> '{log}'\n"
                   "case \"$*\" in\n"
                   "  'topic list -t') printf '"
                   "/task_control/localization [std_msgs/msg/String]\\n"
                   "/task_control/state [std_msgs/msg/String]\\n"
                   "/task_control/status_snapshot [std_msgs/msg/String]\\n' ;;\n"
                   "  'service list -t') printf '"
                   "/task_control/status [nav2_mission_interfaces/srv/GetTaskStatus]\\n' ;;\n"
                   "  'topic echo --once --field data /task_control/localization') "
                   "printf '{\"valid\":true,\"map_id\":\"expected\","
                   "\"observed_map_id\":\"actual\"}\\n' ;;\n"
                   "  'topic echo --once --field data /task_control/state') "
                   "printf '{\"ready\":false,\"stopped\":false}\\n' ;;\n"
                   "  'topic echo --once --field data /task_control/status_snapshot') "
                   "printf '{\"active\":false,\"phase\":\"idle\"}\\n' ;;\n"
                   "  *) exit 11 ;;\n"
                   "esac\n")
    ros.chmod(0o755)
    record = json.loads(run_preflight(tmp_path, '--source', 'amcl', '--profile', 'indoor').stdout)
    assert record['localization']['observed']['observed_map_id'] == 'actual'
    assert record['localization']['selected'] == {'source': 'amcl', 'profile': 'indoor'}
    assert record['safety']['observed']['ready'] is False
    assert record['task_status']['value']['phase'] == 'idle'
    calls = log.read_text().splitlines()
    assert calls == ['topic list -t', 'service list -t',
                     'topic echo --once --field data /task_control/localization',
                     'topic echo --once --field data /task_control/state',
                     'topic echo --once --field data /task_control/status_snapshot']
    assert all(case['status'] == 'not_run' for case in record['field_cases'])


def test_configuration_evidence_distinguishes_configured_and_measured(tmp_path):
    vehicle = tmp_path / 'vehicle.yaml'
    vehicle.write_text('wheelbase: 0.8\nmax_speed: 0.2\n')
    map_file = tmp_path / 'map.yaml'
    map_file.write_text('image: map.pgm\nresolution: 0.05\n')
    record = json.loads(run_preflight(tmp_path, '--vehicle-config', str(vehicle),
                                      '--map-config', str(map_file)).stdout)
    assert record['vehicle']['configured']['content']['wheelbase'] == 0.8
    assert record['vehicle']['measured'] is None
    assert record['map']['configured']['content']['resolution'] == 0.05
    assert record['map']['observed'] is None
    assert record['datum_and_frame_registration_measured'] is None


def test_unrepresentable_yaml_does_not_prevent_evidence_collection(tmp_path):
    config = tmp_path / 'config.yaml'
    config.write_text('calibrated: 2026-10-30\nlimit: .nan\n')
    record = json.loads(run_preflight(tmp_path, '--vehicle-config', str(config)).stdout)
    assert record['vehicle']['configured']['content'] is None
    assert record['vehicle']['configured']['sha256'] is not None
    assert record['vehicle']['configured']['error']
    assert all(case['status'] == 'not_run' for case in record['field_cases'])

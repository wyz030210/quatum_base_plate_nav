"""Collect read-only software and field-acceptance evidence; never command motion."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

import yaml


CASES = (
    'inspection_multipoint', 'inspection_cancel_navigation_and_dwell',
    'coverage_entire_path_and_stop', 'coverage_with_hole',
    'amcl_source_loss_and_recovery', 'rtk_source_loss_and_recovery',
    'cancel_each_coverage_stage', 'manual_takeover_no_resume',
    'emergency_stop_and_reset', 'command_and_feedback_loss',
    'firmware_watchdog_and_measured_stop', 'task_switch_and_ordinary_navigation',
    'reports_accessible',
)
TOPICS = ('/task_control/localization', '/task_control/state',
          '/task_control/status_snapshot', '/map', '/Odometry',
          '/odometry/filtered_odom', '/tf', '/tf_static')
SOURCE_TOPICS = {'amcl': ('/amcl_pose', '/scan'),
                 'rtk_ekf': ('/localization/rtk_source_state', '/odometry/filtered_map')}
SERVICES = ('/task_control/start', '/task_control/cancel', '/task_control/status',
            '/is_path_valid')


def command(argv, timeout):
    try:
        result = subprocess.run(argv, capture_output=True, text=True, timeout=timeout,
                                check=False, stdin=subprocess.DEVNULL)
        if result.returncode:
            return {'status': 'error', 'detail': result.stderr.strip() or
                    f'exit {result.returncode}', 'value': None}
        return {'status': 'observed', 'detail': None, 'value': result.stdout.strip()}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {'status': 'unavailable', 'detail': str(exc), 'value': None}


def evidence_file(path):
    result = {'path': str(Path(path).resolve()), 'sha256': None, 'content': None, 'error': None}
    try:
        content = Path(path).read_bytes()
        result['sha256'] = hashlib.sha256(content).hexdigest()
        value = yaml.safe_load(content)
        json.dumps(value, allow_nan=False)
        result['content'] = value
    except (OSError, yaml.YAMLError, UnicodeDecodeError, TypeError, ValueError) as exc:
        result['error'] = str(exc)
    return result


def snapshot(ros, topic, present, timeout):
    if not present:
        return {'status': 'missing', 'value': None, 'detail': 'topic not observed'}
    result = command([ros, 'topic', 'echo', '--once', '--field', 'data', topic], timeout)
    if result['status'] != 'observed':
        return result
    try:
        value = yaml.safe_load(result['value'])
        if isinstance(value, str):
            value = json.loads(value)
        if not isinstance(value, dict):
            raise ValueError('status is not a JSON object')
        # Non-finite values are not valid portable evidence.
        json.dumps(value, allow_nan=False)
        result['value'] = value
    except (ValueError, TypeError, yaml.YAMLError) as exc:
        result.update(status='error', value=None, detail=str(exc))
    return result


def present_names(result):
    if result['status'] != 'observed':
        return set()
    return {line.split(' [', 1)[0] for line in result['value'].splitlines() if line}


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', default='.', help='deployed source checkout for revision')
    parser.add_argument('--source', choices=('amcl', 'rtk_ekf'))
    parser.add_argument('--profile', help='selected localization profile')
    parser.add_argument('--vehicle-config', metavar='YAML')
    parser.add_argument('--map-config', metavar='YAML')
    parser.add_argument('--datum-config', metavar='YAML')
    parser.add_argument('--config', action='append', default=[], metavar='YAML',
                        help='configuration evidence file; repeat for map, vehicle, datum')
    parser.add_argument('--build-evidence', action='append', default=[], metavar='YAML',
                        help='software build/dependency evidence file')
    parser.add_argument('--timeout', type=float, default=3.0,
                        help='maximum seconds for each read-only ROS command')
    options = parser.parse_args(args)
    if not 0 < options.timeout <= 30:
        parser.error('--timeout must be in (0, 30]')
    git = shutil.which('git')
    unavailable_git = {'status': 'unavailable', 'detail': 'git not found', 'value': None}
    revision = (command([git, '-C', options.repo, 'rev-parse', 'HEAD'], options.timeout)
                if git else unavailable_git)
    dirty = (command([git, '-C', options.repo, 'status', '--porcelain'], options.timeout)
             if git else unavailable_git)
    ros = shutil.which('ros2')
    if ros:
        topics = command([ros, 'topic', 'list', '-t'], options.timeout)
        services = command([ros, 'service', 'list', '-t'], options.timeout)
    else:
        topics = services = {'status': 'unavailable', 'detail': 'ros2 not found', 'value': None}
    names = present_names(topics)
    service_names = present_names(services)
    required_topics = TOPICS + SOURCE_TOPICS.get(options.source, ())
    loc = snapshot(ros, '/task_control/localization',
                   '/task_control/localization' in names, options.timeout) if ros else {
                       'status': 'unavailable', 'value': None, 'detail': 'ros2 not found'}
    safety = snapshot(ros, '/task_control/state', '/task_control/state' in names,
                      options.timeout) if ros else {
                          'status': 'unavailable', 'value': None, 'detail': 'ros2 not found'}
    task = snapshot(ros, '/task_control/status_snapshot',
                    '/task_control/status_snapshot' in names, options.timeout) if ros else {
                        'status': 'unavailable', 'value': None, 'detail': 'ros2 not found'}
    record = {
        'schema_version': 1,
        'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
        'source_revision': revision, 'source_worktree_status': dirty,
        'site': None,
        'localization': {'selected': {'source': options.source, 'profile': options.profile},
                         'observed': loc['value'], 'observation': loc},
        'safety': {'observed': safety['value'], 'observation': safety},
        'task_status': task,
        'ros_graph': {'status': 'observed' if topics['status'] == services['status'] ==
                      'observed' else 'unavailable', 'topics': topics, 'services': services,
                      'required_topics': {name: (name in names if topics['status'] == 'observed'
                                                 else None) for name in required_topics},
                      'required_services': {
                          name: (name in service_names
                                 if services['status'] == 'observed' else None)
                          for name in SERVICES}},
        'configured_files': [evidence_file(path) for path in options.config],
        'map': {'configured': evidence_file(options.map_config) if options.map_config else None,
                'observed': loc['value'].get('observed_map_id') if loc['value'] else None},
        'datum_configured': evidence_file(options.datum_config) if options.datum_config else None,
        'datum_and_frame_registration_measured': None,
        'vehicle': {'configured': evidence_file(options.vehicle_config)
                    if options.vehicle_config else None, 'measured': None},
        'dependencies_and_build': {'ros_distro': os.environ.get('ROS_DISTRO'),
                                   'ament_prefix_path': os.environ.get('AMENT_PREFIX_PATH'),
                                   'evidence_files': [evidence_file(path) for path in
                                                      options.build_evidence]},
        'software_checks': [],
        'field_cases': [{'id': item, 'status': 'not_run', 'start_time': None,
                         'end_time': None, 'task_id': None, 'phase': None, 'result': None,
                         'cleanup': None, 'report_paths': [], 'measurements': None,
                         'observations': None, 'evidence': []} for item in CASES],
    }
    print(json.dumps(record, indent=2, ensure_ascii=False, allow_nan=False))


if __name__ == '__main__':
    main()

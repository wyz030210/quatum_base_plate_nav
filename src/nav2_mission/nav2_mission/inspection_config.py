"""Load and freeze source-format site points and templates for task requests."""
import copy
import math
import re
from pathlib import Path

import yaml


def _mapping(value, allowed, context):
    if not isinstance(value, dict):
        raise ValueError(f'{context} must be a mapping')
    if set(value) - set(allowed):
        raise ValueError(f'unsupported {context} options')
    return value


def _identifier(value, context):
    value = _text(value, context)
    if re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', value) is None:
        raise ValueError(f'{context} must use 1-64 ASCII letters, digits, _ or -')
    return value


def _text(value, context):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f'{context} must be a non-empty string')
    return value.strip()


def _point(raw, *, site=False):
    keys = {'id', 'name', 'type', 'stay_time', 'allow_skip'}
    point = copy.deepcopy(_mapping(raw, keys | ({'pose'} if site else {'x', 'y', 'yaw'}),
                                   'point'))
    point['id'] = _identifier(point.get('id'), 'point id')
    point['name'] = _text(point.get('name', None if site else point['id']), 'point name')
    point['type'] = _text(point.get('type'), 'point type')
    if point['type'] not in {'security', 'facility', 'environment', 'dock'}:
        raise ValueError('unsupported point type')
    if site:
        pose = _mapping(point.pop('pose', None), {'x', 'y', 'yaw'}, 'pose')
        point.update(pose)
    for key in ('x', 'y', 'yaw', 'stay_time'):
        value = point.get(key)
        try:
            valid = (not isinstance(value, bool) and isinstance(value, (int, float)) and
                     math.isfinite(value))
        except OverflowError:
            valid = False
        if not valid:
            raise ValueError(f'{key} must be a finite number')
    if point['stay_time'] < 0:
        raise ValueError('stay_time must be nonnegative')
    if not site:
        point.setdefault('allow_skip', False)
    if type(point.get('allow_skip')) is not bool:
        raise ValueError('allow_skip must be boolean')
    return point


def _read(path, key):
    path = _text(path, f'{key}_file')
    try:
        data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ValueError(f'cannot read {key} configuration: {exc}') from exc
    entries = _mapping(data, {key}, key).get(key)
    if not isinstance(entries, list) or not entries:
        raise ValueError(f'{key} must be a non-empty list')
    return entries


def prepare_inspection(request):
    """Keep direct requests and accept explicit site files without demo defaults."""
    config = copy.deepcopy(_mapping(request, {
        'frame_id', 'point', 'points_file', 'missions_file', 'mission_id'}, 'inspection'))
    config['frame_id'] = _text(config.get('frame_id'), 'frame_id')
    if 'point' in config:
        if set(config) != {'frame_id', 'point'}:
            raise ValueError('point and site template requests cannot be combined')
        point = _point(config['point'])
        config.update(point=point, points=[point], mission_id='direct',
                      mission_name='Direct inspection')
        return config
    points = {}
    for raw in _read(config.get('points_file'), 'points'):
        point = _point(raw, site=True)
        if point['id'] in points:
            raise ValueError(f'duplicate point id: {point["id"]}')
        points[point['id']] = point
    missions = {}
    for raw in _read(config.get('missions_file'), 'missions'):
        mission = _mapping(raw, {'id', 'name', 'points', 'nav_bt',
                                 'stop_on_required_point_failure'}, 'mission')
        mission_id = _identifier(mission.get('id'), 'mission id')
        if mission_id in missions:
            raise ValueError(f'duplicate mission id: {mission_id}')
        mission['name'] = _text(mission.get('name'), 'mission name')
        ids = mission.get('points')
        if not isinstance(ids, list) or not ids:
            raise ValueError('mission points must be a non-empty list')
        ids = [_identifier(point_id, 'mission point id') for point_id in ids]
        mission['points'] = ids
        for point_id in ids:
            if point_id not in points:
                raise ValueError(f'unknown point: {point_id}')
        if len(set(ids)) != len(ids):
            raise ValueError('mission contains duplicate point ids')
        if mission.get('nav_bt', '') != '':
            raise ValueError('nav_bt override unsupported; use the vehicle default tree')
        if mission.get('stop_on_required_point_failure', True) is not True:
            raise ValueError('stop_on_required_point_failure must be true')
        missions[mission_id] = mission
    selected = _identifier(config.get('mission_id'), 'mission_id')
    if selected not in missions:
        raise ValueError(f'unknown mission id: {selected}')
    mission = missions[selected]
    config.update(mission_id=selected, mission_name=mission['name'], nav_bt='',
                  stop_on_required_point_failure=True,
                  points=[points[point_id] for point_id in mission['points']])
    return config

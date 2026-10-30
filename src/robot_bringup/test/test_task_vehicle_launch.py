"""Inspect the task-enabled vehicle entry points without starting any hardware."""
import pytest
from launch.actions import OpaqueFunction

from test_outdoor_launch import (
    arguments, capture_nodes, context_for, descendants, include_named, load,
)


@pytest.mark.parametrize('filename,source,navigation', [
    ('indoor_amcl.launch.py', 'amcl', 'nav2_indoor.launch.py'),
    ('outdoor_ekf.launch.py', 'rtk_ekf', 'nav2.launch.py'),
])
def test_vehicle_entry_enables_one_shared_task_layer(filename, source, navigation):
    module = load('robot_bringup/launch/' + filename)
    nodes = capture_nodes(module)
    description = module.generate_launch_description()
    context = context_for(description, use_task_control='true', use_sim_time='true',
                          task_map_id='a' * 64, task_localization_profile='field')
    for item in descendants(description.entities):
        if isinstance(item, OpaqueFunction):
            for action in item.execute(context) or []:
                action.execute(context)
    enabled = [n for n in nodes if not n.get('condition') or n['condition'].evaluate(context)]
    manager = next(n for n in enabled if n['executable'] == 'task_control_node')
    provider = next(n for n in enabled if n['executable'] == 'localization_provider')
    assert sum(n['executable'] == 'task_control_node' for n in enabled) == 1
    assert sum(n['executable'] == 'localization_provider' for n in enabled) == 1
    assert provider['parameters'][0]['source'] == source
    for node in (manager, provider):
        params = node['parameters'][0]
        assert params['map_id'].evaluate(context) == 'a' * 64
        assert params['localization_profile'].evaluate(context) == 'field'
        assert params['use_sim_time'].evaluate(context)
    assert manager['parameters'][0]['require_localization_schema'] is True
    base = arguments(include_named(description, context, 'auto_control.launch.py'), context)
    assert base['task_control_required'] == 'true' and base['use_sim_time'] == 'true'
    nav = arguments(include_named(description, context, navigation), context)
    assert nav['use_coverage'].lower() == 'true' and nav['use_task_control'] == 'true'


def test_task_entry_requires_provisioned_map_and_profile():
    module = load('robot_bringup/launch/outdoor_ekf.launch.py')
    description = module.generate_launch_description()
    context = context_for(description, use_task_control='true')
    with pytest.raises(RuntimeError, match='task_map_id|task_localization_profile'):
        validator = next(item for item in descendants(description.entities)
                         if isinstance(item, OpaqueFunction))
        validator.execute(context)


def test_task_mode_disables_legacy_selector_and_uses_managed_rviz_goal():
    import yaml
    from pathlib import Path
    for mode, expected in [('false', True), ('true', False)]:
        module = load('wheeltec_nav2/launch/navigation_with_coverage_launch.py')
        description = module.generate_launch_description()
        context = context_for(description, use_coverage='true', use_task_control=mode,
                              use_sim_time='true', params_file='/tmp/nav.yaml',
                              coverage_params_file='/tmp/coverage.yaml')
        selector = include_named(description, context, 'coverage_region_selector.launch.py')
        bridge = include_named(description, context, 'coverage_navpath_bridge.launch.py')
        assert selector.condition.evaluate(context) is expected
        assert bridge.condition.evaluate(context)
        assert arguments(bridge, context)['use_sim_time'] == 'true'
    src = Path(__file__).resolve().parents[2]
    old = yaml.safe_load((src / 'wheeltec_nav2/rviz/map.rviz').read_text())
    managed = yaml.safe_load((src / 'wheeltec_nav2/rviz/map_task.rviz').read_text())

    def goal_topic(config):
        return next(tool['Topic']['Value'] for tool in config['Visualization Manager']['Tools']
                    if tool['Class'] == 'rviz_default_plugins/SetGoal')
    assert goal_topic(old) == '/goal_pose'
    assert goal_topic(managed) == '/task_control/goal_pose'


def test_common_command_converter_uses_selected_clock_and_required_gate():
    module = load('vehicle_can_bridge/launch/auto_control.launch.py')
    nodes = capture_nodes(module)
    description = module.generate_launch_description()
    context = context_for(description, use_sim_time='true', task_control_required='true')
    converter = next(node for node in nodes if node['executable'] == 'cmd_vel_to_can')
    params = converter['parameters'][-1]
    assert params['use_sim_time'].evaluate(context)
    assert params['task_control_required'].evaluate(context)

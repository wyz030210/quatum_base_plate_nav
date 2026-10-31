"""The control window follows coverage flags, without starting hardware in tests."""
import pytest

from test_outdoor_launch import arguments, capture_nodes, context_for, include_named, load


@pytest.mark.parametrize('coverage,gui,managed', [
    ('true', 'true', 'true'), ('true', 'true', 'false'),
    ('false', 'true', 'true'), ('true', 'false', 'false'),
])
def test_coverage_window_and_legacy_selector_are_exclusive(coverage, gui, managed):
    module = load('wheeltec_nav2/launch/navigation_with_coverage_launch.py')
    description = module.generate_launch_description()
    context = context_for(description, use_coverage=coverage, use_coverage_gui=gui,
                          use_task_control=managed, params_file='/tmp/nav.yaml',
                          coverage_params_file='/tmp/coverage.yaml')
    panel = include_named(description, context, 'coverage_control.launch.py')
    selector = include_named(description, context, 'coverage_region_selector.launch.py')
    assert panel.condition.evaluate(context) is (coverage == gui == 'true')
    assert selector.condition.evaluate(context) is (
        coverage == 'true' and gui == managed == 'false')
    args = arguments(panel, context)
    assert args['enable_execution'] == managed
    assert args['start_editor'] == str(managed == 'false')


def test_standalone_preview_uses_edit_only_service_and_has_no_motion_nodes():
    module = load('wheeltec_nav2/launch/coverage_preview.launch.py')
    nodes = capture_nodes(module)
    description = module.generate_launch_description()
    context = context_for(description)
    panel = include_named(description, context, 'coverage_control.launch.py')
    args = arguments(panel, context)
    assert args['enable_execution'] == 'false' and args['start_editor'] == 'true'
    assert args['use_gui'] == 'true'
    assert any(n['executable'] == 'map_server' for n in nodes)
    motion_nodes = {'coverage_region_selector_node', 'task_control_node',
                    'controller_server', 'cmd_vel_to_can'}
    assert not any(n['executable'] in motion_nodes for n in nodes)


def test_panel_planner_is_namespaced_and_execution_is_opt_in():
    module = load('nav2_mission/launch/coverage_control.launch.py')
    nodes = capture_nodes(module)
    description = module.generate_launch_description()
    context = context_for(description, coverage_params_file='/tmp/coverage.yaml')
    planner = next(n for n in nodes if n['executable'] == 'opennav_coverage')
    assert planner['namespace'] == 'coverage_preview'
    controls = next(n for n in nodes if n['executable'] == 'coverage_control_gui')
    assert not controls['parameters'][0]['enable_execution'].evaluate(context)
    assert not next(n for n in nodes if n['executable'] == 'coverage_editor_node')[
        'condition'].evaluate(context)


@pytest.mark.parametrize('entry,nav', [
    ('robot_bringup/launch/indoor_amcl.launch.py', 'nav2_indoor.launch.py'),
    ('robot_bringup/launch/outdoor_ekf.launch.py', 'nav2.launch.py'),
    ('wheeltec_nav2/launch/nav2_indoor.launch.py', 'bringup_launch.py'),
    ('wheeltec_nav2/launch/nav2.launch.py', 'bringup_launch.py'),
    ('wheeltec_nav2/launch/bringup_launch.py', 'navigation_with_coverage_launch.py'),
])
def test_gui_flag_reaches_navigation(entry, nav):
    module = load(entry)
    description = module.generate_launch_description()
    context = context_for(description, use_coverage_gui='false', map='/tmp/map.yaml',
                          minimum_turning_radius='0.95')
    values = arguments(include_named(description, context, nav), context)
    assert values['use_coverage_gui'] == 'false'

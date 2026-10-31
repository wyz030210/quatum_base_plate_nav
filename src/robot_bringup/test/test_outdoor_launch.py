"""Inspect launch objects and substitutions without starting ROS processes."""

import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, TimerAction,
)
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions


SRC = Path(__file__).resolve().parents[2]
LAUNCH = SRC / "robot_bringup/launch/outdoor_ekf.launch.py"
RTK_DEVICE = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
)


def load(relative):
    path = SRC / relative
    spec = importlib.util.spec_from_file_location(path.stem.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if hasattr(module, "PythonLaunchDescriptionSource"):
        factory = module.PythonLaunchDescriptionSource

        def source(path):
            result = factory(path)
            result.test_location = normalize_to_list_of_substitutions(path)
            return result

        module.PythonLaunchDescriptionSource = source
    if hasattr(module, "get_package_share_directory"):
        module.get_package_share_directory = lambda name: (
            str(SRC / name) if (SRC / name).is_dir()
            else get_package_share_directory(name)
        )
    return module


def context_for(description, **overrides):
    context = LaunchContext()
    context.launch_configurations.update({k: str(v) for k, v in overrides.items()})
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


def capture_nodes(module):
    captured = []
    for name in ("Node", "ComposableNode"):
        if not hasattr(module, name):
            continue
        factory = getattr(module, name)

        def wrapper(_factory=factory, **kwargs):
            result = _factory(**kwargs)
            captured.append({**kwargs, "_action": result})
            return result

        setattr(module, name, wrapper)
    return captured


def descendants(actions):
    for action in actions:
        yield action
        if isinstance(action, TimerAction):
            yield from descendants(action.actions)
        elif not isinstance(action, IncludeLaunchDescription):
            yield from descendants(action.get_sub_entities() or [])


def include_named(description, context, filename):
    return next(
        action for action in descendants(description.entities)
        if isinstance(action, IncludeLaunchDescription)
        and perform_substitutions(context, action.launch_description_source.test_location).endswith(filename)
    )


def arguments(action, context):
    return {name: perform_substitutions(context, normalize_to_list_of_substitutions(value))
            for name, value in action.launch_arguments}


def test_outdoor_entry_replaces_old_name():
    assert LAUNCH.is_file()
    assert not (LAUNCH.parent / "system.launch.py").exists()


def test_outdoor_starts_current_hardware_and_both_ekfs():
    module = load("robot_bringup/launch/outdoor_ekf.launch.py")
    nodes = capture_nodes(module)
    description = module.generate_launch_description()
    context = context_for(description)
    assert "turn_on_wheeltec_robot" not in LAUNCH.read_text()
    for filename in ("auto_control.launch.py", "msg_MID360_launch.py",
                     "mapping.launch.py", "location_launch.py", "nav2.launch.py"):
        include_named(description, context, filename)
    enabled = [n for n in nodes if not n.get("condition") or n["condition"].evaluate(context)]
    assert {n["name"] for n in enabled} == {"ekf_odom_node", "ekf_map_node"}
    assert all(n["parameters"][-1]["use_sim_time"].evaluate(context) is False
               for n in enabled)
    assert arguments(include_named(description, context, "mapping.launch.py"), context)["rviz"] == "false"


def test_map_and_vehicle_defaults_and_overrides_are_forwarded(tmp_path):
    module = load("robot_bringup/launch/outdoor_ekf.launch.py")
    description = module.generate_launch_description()
    default = context_for(description)
    assert default.launch_configurations["map"] == str(
        Path(get_package_share_directory("wheeltec_nav2")) / "map" / "outdoor_map.yaml"
    )
    assert default.launch_configurations["use_coverage"] == "false"
    defaults = {"max_forward_speed": "0.25", "max_reverse_speed": "0.10",
                "max_yaw_rate": "0.50", "max_linear_accel": "0.40",
                "max_yaw_accel": "0.50", "max_steering_angle_deg": "17.5",
                "wheel_base": "0.30", "wheel_radius": "0.1275"}
    for name, value in defaults.items():
        assert default.launch_configurations[name] == value
    map_path = tmp_path / "test.yaml"
    map_path.write_text("image: test.pgm\n")
    context = context_for(description, map=map_path, max_forward_speed="0.18",
                          max_linear_accel="0.30", max_steering_angle_deg="15.0")
    for action in module.validate_arguments(context) or []:
        action.execute(context)
    base = arguments(include_named(description, context, "auto_control.launch.py"), context)
    nav = arguments(include_named(description, context, "nav2.launch.py"), context)
    assert base["max_forward_speed"] == nav["max_forward_speed"] == "0.18"
    assert base["max_steering_angle_deg"] == "15.0"
    assert nav["max_linear_accel"] == "0.30"
    assert nav["map"] == str(map_path)
    assert float(nav["minimum_turning_radius"]) == pytest.approx(.30 / math.tan(math.radians(15)))


@pytest.mark.parametrize("entry,map_name,child", [
    ("indoor_amcl.launch.py", "indoor_map.yaml", "nav2_indoor.launch.py"),
    ("outdoor_ekf.launch.py", "outdoor_map.yaml", "nav2.launch.py"),
])
def test_package_map_defaults_allow_explicit_override(tmp_path, entry, map_name, child):
    module = load("robot_bringup/launch/" + entry)
    description = module.generate_launch_description()
    context = context_for(description)
    assert context.launch_configurations["map"] == str(
        Path(get_package_share_directory("wheeltec_nav2")) / "map" / map_name
    )
    custom_map = str(tmp_path / "custom.yaml")
    context = context_for(description, map=custom_map)
    nav = include_named(description, context, child)
    map_value = dict(nav.launch_arguments)["map"]
    assert perform_substitutions(
        context, normalize_to_list_of_substitutions(map_value)
    ) == custom_map


@pytest.mark.parametrize("overrides,match", [
    ({"map": "/does/not/exist.yaml"}, "map"),
    ({"use_coverage": "ture"}, "use_coverage"),
    ({"max_forward_speed": "nan"}, "max_forward_speed"),
    ({"max_linear_accel": "-1"}, "max_linear_accel"),
    ({"max_steering_angle_deg": "90"}, "max_steering_angle_deg"),
    ({"can_serial_port": "/tmp/same", "rtk_serial_port": "/tmp/same"}, "serial"),
])
def test_invalid_configuration_is_rejected_before_hardware(tmp_path, overrides, match):
    module = load("robot_bringup/launch/outdoor_ekf.launch.py")
    description = module.generate_launch_description()
    map_path = tmp_path / "map.yaml"
    map_path.write_text("image: map.pgm\n")
    values = {"map": str(map_path), **overrides}
    context = context_for(description, **values)
    with pytest.raises(RuntimeError, match=match):
        module.validate_arguments(context)
    validation = next(a for a in description.entities if isinstance(a, OpaqueFunction))
    assert description.entities.index(validation) < next(
        i for i, a in enumerate(description.entities)
        if not isinstance(a, (DeclareLaunchArgument, OpaqueFunction))
    )


@pytest.mark.parametrize("composition", ["True", "False"])
def test_outdoor_nav_disables_amcl_and_links_server_to_coverage(composition):
    module = load("wheeltec_nav2/launch/nav2.launch.py")
    description = module.generate_launch_description()
    context = context_for(description, use_composition=composition, use_coverage="false")
    args = arguments(include_named(description, context, "bringup_launch.py"), context)
    assert args["use_amcl"] == "false"
    assert args["use_coverage_server"] == args["use_coverage"] == "false"
    context.launch_configurations["use_coverage"] = "true"
    args = arguments(include_named(description, context, "bringup_launch.py"), context)
    assert args["use_coverage_server"] == args["use_coverage"] == "true"
    with open(args["params_file"]) as stream:
        params = yaml.safe_load(stream)
    controller = params["controller_server"]["ros__parameters"]
    assert controller["odom_topic"] == "/odometry/filtered_odom"
    assert controller["FollowPath"]["AckermannConstraints"]["min_turning_r"] == .95
    assert params["planner_server"]["ros__parameters"]["GridBased"]["minimum_turning_radius"] == .95


@pytest.mark.parametrize("coverage", ["true", "false"])
def test_coverage_node_and_lifecycle_switch_together_in_both_modes(coverage):
    module = load("wheeltec_nav2/launch/navigation_with_coverage_launch.py")
    nodes = capture_nodes(module)
    description = module.generate_launch_description()
    context = context_for(description, params_file="unused", coverage_params_file="unused",
                          use_coverage=coverage, use_coverage_server=coverage)
    expected = coverage == "true"
    servers = [n for n in nodes if n.get("name") == "coverage_server"]
    assert len(servers) == 2
    assert all(n["condition"].evaluate(context) is expected for n in servers)
    managers = [n for n in nodes if n.get("name") == "lifecycle_manager_navigation"]
    assert len(managers) == 2
    for manager in managers:
        params = {k: v for p in manager["parameters"] if isinstance(p, dict) for k, v in p.items()}
        assert ("coverage_server" in params["node_names"].evaluate(context)) is expected
    for filename in ("coverage_navpath_bridge.launch.py", "coverage_region_selector.launch.py"):
        assert include_named(description, context, filename).condition.evaluate(context) is expected


def test_indoor_keeps_legacy_amcl_and_server_defaults():
    module = load("wheeltec_nav2/launch/bringup_launch.py")
    description = module.generate_launch_description()
    context = context_for(description, map="unused")
    assert context.launch_configurations["use_amcl"] == "true"
    assert context.launch_configurations["use_coverage_server"] == "true"
    localization = include_named(description, context, "localization_launch.py")
    assert localization.condition.evaluate(context) is True
    context.launch_configurations["use_amcl"] = "false"
    assert localization.condition.evaluate(context) is False


def test_package_declares_current_dependencies():
    root = ET.parse(SRC / "robot_bringup/package.xml").getroot()
    deps = {element.text for element in root.findall("depend")}
    assert {"fast_lio", "vehicle_can_bridge", "location", "wheeltec_nav2"} <= deps
    assert "turn_on_wheeltec_robot" not in deps


@pytest.mark.parametrize("composition", ["true", "false"])
def test_map_only_mode_manages_one_map_and_no_amcl(composition):
    module = load("wheeltec_nav2/launch/bringup_launch.py")
    nodes = capture_nodes(module)
    factory = module.LoadComposableNodes

    def capture_loader(**kwargs):
        result = factory(**kwargs)
        result.test_components = kwargs["composable_node_descriptions"]
        return result

    module.LoadComposableNodes = capture_loader
    description = module.generate_launch_description()
    context = context_for(description, map="unused", use_amcl="false",
                          use_composition=composition)

    def active(actions):
        for action in actions:
            if action.condition and not action.condition.evaluate(context):
                continue
            yield action
            if isinstance(action, GroupAction):
                yield from active(action.get_sub_entities())

    actions = list(active(description.entities))
    assert include_named(description, context, "localization_launch.py") not in actions
    enabled = {id(a) for a in actions}
    for action in actions:
        enabled.update(id(c) for c in getattr(action, "test_components", []))
    maps = [n for n in nodes if n.get("name") == "map_server" and id(n["_action"]) in enabled]
    managers = [n for n in nodes if n.get("name") == "lifecycle_manager_localization"
                and id(n["_action"]) in enabled]
    assert len(maps) == len(managers) == 1
    assert managers[0]["parameters"][0]["node_names"] == ["map_server"]


@pytest.mark.parametrize("entry,argument", [
    ("robot_bringup/launch/outdoor_ekf.launch.py", "rtk_serial_port"),
    ("location/launch/location_launch.py", "serial_port"),
])
def test_confirmed_rtk_device_is_the_default(entry, argument):
    module = load(entry)
    nodes = capture_nodes(module)
    description = module.generate_launch_description()
    context = context_for(description)
    assert context.launch_configurations[argument] == RTK_DEVICE
    if argument == "rtk_serial_port":
        rtk = include_named(description, context, "location_launch.py")
        assert arguments(rtk, context)["serial_port"] == RTK_DEVICE
    else:
        assert nodes[0]["parameters"][0]["serial_port"].perform(context) == RTK_DEVICE


@pytest.mark.parametrize("rtk_port", [RTK_DEVICE, "/dev/custom_rtk"])
def test_outdoor_rtk_port_override_preserves_can_device(rtk_port):
    module = load("robot_bringup/launch/outdoor_ekf.launch.py")
    description = module.generate_launch_description()
    context = context_for(description, rtk_serial_port=rtk_port)
    rtk = include_named(description, context, "location_launch.py")
    base = include_named(description, context, "auto_control.launch.py")
    assert arguments(rtk, context)["serial_port"] == rtk_port
    assert arguments(base, context)["serial_port"] == (
        "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
    )


def test_same_serial_device_via_different_aliases_is_rejected(tmp_path):
    module = load("robot_bringup/launch/outdoor_ekf.launch.py")
    description = module.generate_launch_description()
    map_path = tmp_path / "map.yaml"
    map_path.write_text("image: map.pgm\n")
    device = tmp_path / "ttyUSB0"
    device.touch()
    can_alias = tmp_path / "can"
    rtk_alias = tmp_path / "rtk"
    can_alias.symlink_to(device)
    rtk_alias.symlink_to(device)
    context = context_for(description, map=map_path, can_serial_port=can_alias,
                          rtk_serial_port=rtk_alias)
    with pytest.raises(RuntimeError, match="serial ports must be different"):
        module.validate_arguments(context)


def test_rtk_port_and_datum_overrides_reach_node():
    module = load("location/launch/location_launch.py")
    nodes = capture_nodes(module)
    description = module.generate_launch_description()
    context = context_for(description, serial_port="/dev/rtk", baud_rate="230400",
                          datum_lat="30.0", datum_lon="107.0", use_sim_time="true")
    params = nodes[0]["parameters"][0]
    assert params["serial_port"].perform(context) == "/dev/rtk"
    assert params["baud_rate"].evaluate(context) == 230400
    assert params["datum_lat"].evaluate(context) == 30.0
    assert params["datum_lon"].evaluate(context) == 107.0
    assert params["use_sim_time"].evaluate(context) is True


def test_acceleration_and_speed_reach_both_controller_and_smoother_modes():
    module = load("wheeltec_nav2/launch/navigation_with_coverage_launch.py")
    nodes = capture_nodes(module)
    description = module.generate_launch_description()
    context = context_for(description, params_file="unused", coverage_params_file="unused",
                          max_forward_speed="0.18", max_reverse_speed="0.08",
                          max_yaw_rate="0.35", max_linear_accel="0.30", max_yaw_accel="0.45")
    for node in nodes:
        if node.get("name") == "controller_server":
            params = node["parameters"][-1]
            for name, expected in {"vx_max": .18, "vx_min": -.08, "wz_max": .35,
                                   "ax_max": .30, "ax_min": -.30, "az_max": .45}.items():
                assert params[f"FollowPath.{name}"].evaluate(context) == expected
        elif node.get("name") == "velocity_smoother":
            params = node["parameters"][-1]
            assert params["max_velocity"].evaluate(context) == [.18, 0.0, .35]
            assert params["min_velocity"].evaluate(context) == [-.08, 0.0, -.35]
            assert params["max_accel"].evaluate(context) == [.30, 0.0, .45]
            assert params["max_decel"].evaluate(context) == [-.30, 0.0, -.45]

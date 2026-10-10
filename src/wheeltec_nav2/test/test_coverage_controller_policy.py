"""Check the actual launch overrides for Humble MPPI coverage reversing."""

import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext

PACKAGE = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("value", ["true", "True", "false", "False"])
def test_coverage_reverse_policy_in_both_launch_modes(value):
    coverage = value.lower() == "true"
    spec = importlib.util.spec_from_file_location(
        "coverage_navigation_launch", PACKAGE / "launch/navigation_with_coverage_launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    controllers = []
    original_node, original_composable = module.Node, module.ComposableNode

    def capture(factory, **kwargs):
        if kwargs.get("name") == "controller_server":
            controllers.append(kwargs)
        return factory(**kwargs)

    module.Node = lambda **kwargs: capture(original_node, **kwargs)
    module.ComposableNode = lambda **kwargs: capture(original_composable, **kwargs)
    module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update({
        "use_coverage": value,
        "max_forward_speed": "0.4", "max_reverse_speed": "0.25",
        "max_yaw_rate": "0.5", "max_linear_accel": "0.4", "max_yaw_accel": "0.5",
    })
    assert len(controllers) == 2
    for node in controllers:
        overrides = node["parameters"][-1]
        assert overrides["FollowPath.PathAngleCritic.forward_preference"].evaluate(context) is (not coverage)
        assert overrides["FollowPath.PreferForwardCritic.enabled"].evaluate(context) is (not coverage)
        assert overrides["FollowPath.PathAlignCritic.use_path_orientations"].evaluate(context) is coverage
        assert overrides["FollowPath.vx_max"].evaluate(context) == .4
        assert overrides["FollowPath.vx_min"].evaluate(context) == -.25
        assert ("cmd_vel", "cmd_vel_nav") in node["remappings"]


def test_profile_uses_supported_humble_parameter():
    with (PACKAGE / "param/wheeltec_params/param_mini_diff_indoor.yaml").open() as stream:
        params = yaml.safe_load(stream)["controller_server"]["ros__parameters"]["FollowPath"]
    assert "mode" not in params["PathAngleCritic"]
    assert "forward_preference" in params["PathAngleCritic"]
    assert params["enforce_path_inversion"] is True


def test_rviz_separates_coverage_from_approach_and_footprint():
    with (PACKAGE / "rviz/map.rviz").open() as stream:
        displays = yaml.safe_load(stream)["Visualization Manager"]["Displays"]
    by_topic = {d["Topic"]["Value"]: d for d in displays if isinstance(d.get("Topic"), dict)}
    path = by_topic["/coverage_server/coverage_plan"]
    assert path["Enabled"] is True
    assert by_topic["/coverage_server/swaths"]["Enabled"] is False
    assert path["Color"] != by_topic["/plan"]["Color"]
    assert path["Color"] != by_topic["/local_costmap/published_footprint"]["Color"]

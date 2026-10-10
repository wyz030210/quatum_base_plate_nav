import os
import importlib.util
from pathlib import Path
from typing import List
import xml.etree.ElementTree as ET

import pytest
from launch import LaunchContext


SRC_DIR = Path(__file__).resolve().parents[2]


def read(relative_path):
    return (SRC_DIR / relative_path).read_text(encoding="utf-8")


def test_indoor_launch_excludes_rtk_and_global_ekf():
    text = read("robot_bringup/launch/indoor_amcl.launch.py")
    assert "vehicle_can_bridge" in text
    assert "msg_MID360_launch.py" in text
    assert "fast_lio" in text
    assert "ekf_odom_node" in text
    assert "nav2_indoor.launch.py" in text
    assert '"location"' not in text
    assert "ekf_map_node" not in text


def test_indoor_launch_exposes_safe_speed_interface():
    text = read("robot_bringup/launch/indoor_amcl.launch.py")
    for name in (
        "max_forward_speed",
        "max_reverse_speed",
        "max_yaw_rate",
        "max_linear_accel",
        "max_yaw_accel",
    ):
        assert name in text
    assert 'default_value="0.25"' in text
    assert 'default_value="0.10"' in text


def test_amcl_owns_map_to_odom_and_controller_uses_local_odom():
    params = read(
        "wheeltec_nav2/param/wheeltec_params/param_mini_diff_indoor.yaml"
    )
    assert "base_frame_id: base_link" in params
    assert "odom_frame_id: odom" in params
    assert "global_frame_id: map" in params
    assert "tf_broadcast: true" in params
    assert "set_initial_pose: false" in params
    assert "odom_topic: /odometry/filtered_odom" in params
    assert "velocity_smoother:" in params


def test_fast_lio_defers_odom_tf_to_local_ekf():
    source = read("fast_lio/src/FAST_LIO/src/laserMapping.cpp")
    config = read("fast_lio/src/FAST_LIO/config/mid360_indoor.yaml")
    assert 'odom_frame_id = "odom"' in source
    assert 'body_frame_id = "base_link"' in source
    assert "if (publish_odom_tf)" in source
    assert 'odom_frame_id: "odom"' in config
    assert 'body_frame_id: "base_link"' in config
    assert "publish_odom_tf: false" in config
    assert "pcd_save_en: false" in config


def test_driver_uses_real_wheel_radius_and_asymmetric_limits():
    source = read("vehicle_can_bridge/src/cmd_vel_to_can_node.cpp")
    launch = read("vehicle_can_bridge/launch/auto_control.launch.py")
    assert '"wheel_radius", 0.1275' in source
    assert '"max_forward_speed"' in source
    assert '"max_reverse_speed"' in source
    assert "max_forward_speed" in launch
    assert "max_reverse_speed" in launch
    assert "'wheel_radius': 0.05" not in launch


def test_driver_converts_twist_to_ackermann_steering_angle():
    source = read("vehicle_can_bridge/src/cmd_vel_to_can_node.cpp")
    header = read("vehicle_can_bridge/include/vehicle_can_bridge/ackermann_steering.hpp")
    launch = read("vehicle_can_bridge/launch/auto_control.launch.py")

    assert "steeringAngleFromTwist" in source
    assert "std::atan(wheel_base * yaw_rate / linear_speed)" in header
    assert '"max_steering_angle_deg", 30.0' in source
    assert '"min_steering_speed", 0.02' in source
    assert "max_steering_angle_deg" in launch
    assert "max_yaw_rate" in launch


def test_indoor_planner_and_controller_support_ackermann_reversing():
    params = read(
        "wheeltec_nav2/param/wheeltec_params/param_mini_diff_indoor.yaml"
    )

    assert 'motion_model_for_search: "REEDS_SHEPP"' in params
    assert "minimum_turning_radius: 0.52" in params
    assert "min_turning_r: 0.52" in params
    assert "enforce_path_inversion: true" in params


def test_indoor_scan_projection_filters_ground_in_base_frame():
    nav_launch = read("wheeltec_nav2/launch/nav2_indoor.launch.py")
    bringup_launch = read("robot_bringup/launch/indoor_amcl.launch.py")

    assert '"target_frame": "base_footprint"' in nav_launch
    for name in (
        "lidar_x",
        "lidar_y",
        "lidar_z",
        "lidar_yaw",
        "scan_min_height",
        "scan_max_height",
    ):
        assert name in nav_launch
        assert name in bringup_launch

    assert 'DeclareLaunchArgument("scan_min_height", default_value="0.05")' in nav_launch
    assert 'DeclareLaunchArgument("scan_max_height", default_value="0.80")' in nav_launch
    for launch_text in (nav_launch, bringup_launch):
        assert 'DeclareLaunchArgument("lidar_x", default_value="0.11")' in launch_text
        assert 'DeclareLaunchArgument("lidar_z", default_value="0.615")' in launch_text


def test_package_manifest_declares_indoor_dependencies():
    root = ET.parse(SRC_DIR / "robot_bringup/package.xml").getroot()
    deps = {node.text for node in root.findall("depend")}
    assert {"fast_lio", "vehicle_can_bridge", "wheeltec_nav2"} <= deps


def test_ackermann_behavior_trees_do_not_request_spin():
    tree_dir = SRC_DIR / "wheeltec_nav2" / "behavior_trees"
    for name in (
        "navigate_to_pose_ackermann.xml",
        "navigate_through_poses_ackermann.xml",
    ):
        text = (tree_dir / name).read_text(encoding="utf-8")
        assert "<Spin" not in text
        assert "<BackUp" in text


def test_livox_converter_is_executable():
    converter = SRC_DIR / "wheeltec_nav2" / "scripts" / "livox_to_pointcloud2.py"
    assert converter.is_file()
    assert os.access(converter, os.X_OK)


def test_invalid_use_rviz_is_rejected_before_hardware_starts(tmp_path):
    launch_path = SRC_DIR / "robot_bringup" / "launch" / "indoor_amcl.launch.py"
    spec = importlib.util.spec_from_file_location("indoor_amcl_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    map_file = tmp_path / "map.yaml"
    map_file.write_text("image: map.pgm\n", encoding="utf-8")
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "map": str(map_file),
            "use_rviz": "ture",
            "use_sim_time": "false",
            "max_forward_speed": "0.25",
            "max_reverse_speed": "0.10",
            "max_yaw_rate": "0.50",
            "max_linear_accel": "0.40",
            "max_yaw_accel": "0.50",
            "max_steering_angle_deg": "30.0",
            "min_steering_speed": "0.02",
            "scan_min_height": "0.05",
            "scan_max_height": "0.80",
        }
    )

    with pytest.raises(RuntimeError, match="use_rviz"):
        module.validate_arguments(context)


def test_velocity_smoother_speed_lists_evaluate_as_float_lists():
    launch_path = (
        SRC_DIR
        / "wheeltec_nav2"
        / "launch"
        / "navigation_with_coverage_launch.py"
    )
    spec = importlib.util.spec_from_file_location("navigation_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    captured = []
    real_parameter_value = module.ParameterValue

    def capture_parameter_value(value, value_type=None):
        result = real_parameter_value(value, value_type=value_type)
        if value_type == List[float]:
            captured.append(result)
        return result

    module.ParameterValue = capture_parameter_value
    module.generate_launch_description()

    context = LaunchContext()
    context.launch_configurations.update(
        {
            "max_forward_speed": "0.25",
            "max_reverse_speed": "0.10",
            "max_yaw_rate": "0.50",
            "max_linear_accel": "0.40",
            "max_yaw_accel": "0.50",
        }
    )

    assert [value.evaluate(context) for value in captured] == [
        [0.25, 0.0, 0.50],
        [-0.10, 0.0, -0.50],
        [0.40, 0.0, 0.50],
        [-0.40, 0.0, -0.50],
    ]

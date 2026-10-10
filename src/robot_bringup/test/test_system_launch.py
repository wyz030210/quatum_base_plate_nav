from pathlib import Path
import xml.etree.ElementTree as ET


PKG_DIR = Path(__file__).resolve().parents[1]


def test_package_manifest_declares_runtime_dependencies():
    package_xml = PKG_DIR / "package.xml"
    assert package_xml.exists()

    root = ET.parse(package_xml).getroot()
    deps = {element.text for element in root.findall("depend")}

    expected_deps = {
        "ament_index_python",
        "launch",
        "launch_ros",
        "livox_ros_driver2",
        "location",
        "nav2_mission",
        "robot_localization",
        "turn_on_wheeltec_robot",
        "wheeltec_nav2",
    }
    assert expected_deps <= deps


def test_cmake_installs_launch_directory():
    cmake = PKG_DIR / "CMakeLists.txt"
    assert cmake.exists()

    text = cmake.read_text(encoding="utf-8")
    assert "install(" in text
    assert "DIRECTORY launch" in text
    assert "DESTINATION share/${PROJECT_NAME}" in text


def test_system_launch_starts_existing_commands_in_requested_order():
    launch_file = PKG_DIR / "launch" / "system.launch.py"
    assert launch_file.exists()

    text = launch_file.read_text(encoding="utf-8")
    assert "enable_coverage" not in text
    assert "start_coverage_bridge" not in text
    assert "start_coverage_ui" not in text
    assert "TimerAction" in text

    ordered_markers = [
        "base_serial.launch.py",
        "msg_MID360_launch.py",
        "location_launch.py",
        "ekf_odom_node",
        "ekf_map_node",
        "nav2.launch.py",
        "coverage_navpath_bridge.launch.py",
        "coverage_region_selector.launch.py",
    ]

    positions = [text.index(marker) for marker in ordered_markers]
    assert positions == sorted(positions)


def test_system_launch_uses_package_relative_substitutions():
    launch_file = PKG_DIR / "launch" / "system.launch.py"
    text = launch_file.read_text(encoding="utf-8")

    assert "FindPackageShare" in text
    assert "PathJoinSubstitution" in text
    assert "get_package_share_directory" not in text
    assert "os.path.join" not in text
    assert "/home/" not in text


def test_system_launch_configures_stage_separators():
    launch_file = PKG_DIR / "launch" / "system.launch.py"
    text = launch_file.read_text(encoding="utf-8")

    assert "RCUTILS_CONSOLE_OUTPUT_FORMAT" not in text
    assert "SetEnvironmentVariable" not in text
    assert "LogInfo" in text
    assert "LAUNCH STAGE: " in text

    stage_labels = [
        "base_serial.launch.py",
        "msg_MID360_launch.py",
        "location_launch.py",
        "ekf_odom_node",
        "ekf_map_node",
        "nav2.launch.py",
        "coverage_navpath_bridge.launch.py",
        "coverage_region_selector.launch.py",
    ]

    for label in stage_labels:
        assert label in text

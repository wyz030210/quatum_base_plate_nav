"""Launch only the provider action from the production indoor launch."""
import importlib.util
import sys
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import SetLaunchConfiguration

path = Path(__file__).resolve().parents[2] / 'robot_bringup/launch/indoor_amcl.launch.py'
spec = importlib.util.spec_from_file_location('indoor_amcl_launch', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
provider = next(action for action in module.generate_launch_description().entities
                if getattr(action, 'node_executable', None) == 'localization_provider')
service = LaunchService()
service.include_launch_description(LaunchDescription([
    SetLaunchConfiguration('use_task_control', 'true'),
    SetLaunchConfiguration('use_sim_time', 'false'),
    SetLaunchConfiguration('task_map_id', sys.argv[1]),
    SetLaunchConfiguration('task_localization_profile', 'test-indoor'),
    provider,
]))
sys.exit(service.run())

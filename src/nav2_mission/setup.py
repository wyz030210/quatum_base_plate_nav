from glob import glob
from setuptools import find_packages, setup

package_name = 'nav2_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='car',
    maintainer_email='car@todo.todo',
    description='Nav2 patrol mission node',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'localization_provider = nav2_mission.localization_provider_node:main',
            'task_control_node = nav2_mission.task_control_node:main',
            'task_control = nav2_mission.task_cli:main',
            'field_preflight = nav2_mission.field_preflight:main',
            'patrol_mission_node = nav2_mission.patrol_mission_node:main',
            'facility_detection_node = nav2_mission.facility_detection_node:main',
            'coverage_navpath_bridge_node = nav2_mission.coverage_navpath_bridge_node:main',
            'coverage_region_selector_node = nav2_mission.coverage_region_selector_node:main',
        ],
    },
)

# Rectangle coverage software validation

2026-10-29 software verification on the branch containing integration `f26a1d9`:

- Three packages built: `nav2_mission_interfaces`, `nav2_mission`, `opennav_coverage`.
- 25 real ROS coverage boundary cases passed on exclusive domain 193, including
  two real C++ planner tests; repeated with continuous FollowPath feedback.
- 132 shared task/inspection/coverage/preview/launch/controller policy cases passed.
- 48 native OpenNav cases passed in the utils, robot, swath, path and server
  suites (53 aggregate entries including the five CTest wrappers), no failures
  or skips.
- Production and new-test flake8, Python parsing and `git diff --check` passed.

The concurrent planner edit regression first failed with an eight-second
parameter service timeout before the lock correction, then passed. A rejected
outer action first stranded the prepared reservation and failed the next task;
the added public regression passed after acknowledged disposal was implemented.

Run isolated ROS tests with `ROS_DOMAIN_ID=193`, `ROS_LOCALHOST_ONLY=1`, no
hardware nodes and matching build overlays. DDS sockets must be permitted by the
test environment. Use the persistent extracted GDAL Python binding if the
system `python3-gdal` package is not installed:

```bash
source /opt/ros/humble/setup.bash
source /home/wyz/ros2/new_base_plate_indoor_ws/install/setup.bash
# Build the current branch, then source its install/setup.bash.
export ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/coverage-test-logs
PYTHONPATH=src/nav2_mission:$PYTHONPATH python3 -m pytest -q -p no:cacheprovider \
  src/nav2_mission/test/test_coverage_task_ros.py \
  src/nav2_mission/test/test_coverage_parameter_lease_ros.py \
  src/nav2_mission/test/test_coverage_regressions.py \
  src/nav2_mission/test/test_coverage_preview.py
```

The task tests run the real manager and real coverage bridge, replacing only
downstream planning/navigation/controller/lifecycle/path-check and localization/
stop providers. The planner lease tests run the actual C++ OpenNav subprocess,
including a real computed path with an overlapping parameter transaction. They
exercise ROS service/action boundaries and assert public results and reports.

The suite covers full path/first-pose/two-check execution, report details and
fresh stop, cancellation during pending and accepted compute/approach/follow
goals, late parameter acquisition, malformed paths/numeric input, failure at
each stage, report I/O failure, runtime parameter freezing and outer rejection
reservation cleanup. Keep existing unified-control, inspection, launch and
Ackermann controller policy regressions in the same validation run.

No simulated or physical vehicle is started. These tests do not measure stopping
distance, localization accuracy, real controller tracking, cleaning quality or
area coverage. Those remain explicit field acceptance work in #9.

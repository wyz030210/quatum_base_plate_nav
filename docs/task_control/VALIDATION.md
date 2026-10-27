# Ticket #2 software validation

Validated on 2026-10-27 against target baseline
`fb222f9868375e9cf0d1f80b24a30c2e25741c41` and ROS 2 Humble.

- Generated `nav2_mission_interfaces` and built `nav2_mission` successfully.
- Public task control, real ROS service/action/CLI, and retained coverage
  regression/preview tests: **35 passed**.
- Snapshot verification and the six retained source pure-module suites:
  **46 passed**.
- New production modules pass flake8 (`--max-line-length=99`), compile checks,
  and `git diff --check`.

The signal tests first reproduced failure for both SIGINT and SIGTERM because
the default rclpy handlers invalidated the context before cancellation/stop
cleanup. Both actual subprocess cases pass after deferring context shutdown:
the owned navigation action ends, stop is confirmed, a shutdown report is
written, and the process exits with code 0.

ROS tests used `ROS_DOMAIN_ID=161`, `ROS_LOCALHOST_ONLY=1`, the original target
install as the dependency overlay, this worktree's generated interfaces, and
the extracted GDAL 3.4.1 binding. Downstream localization, command-state and
navigation action providers were substitutes. No hardware node or velocity
publisher was started by these tests.

The source remains in the persistent worktree. This host's rosidl/CMake parser
failed when its source path contained Chinese characters, so package discovery
used `/tmp/property-inspection-issue-2-source`, an ASCII symlink to the persistent
worktree, with separate temporary build/install caches. The normal build
instructions apply when the checkout path is ASCII; the symlink can be recreated
without copying source.

This verifies the ticket's single-point software integration. It does not verify
vehicle stopping performance, actual AMCL or RTK/EKF health producers, a real
Nav2 route, or low-speed vehicle acceptance. Those remain with the dependent
integration tickets and ticket #9.

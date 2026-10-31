# Ticket #4 software validation

Validated on2026-10-28, based on integration commit
`1e23e5d91515d2a7dbd082b319d77893f9fdce31` with ROS2 Humble.

- Generated task interfaces and built `nav2_mission` successfully.
- Public task core, site template validation/rules/reports, real ROS services,
  navigation action/client, CLI and existing coverage regression/preview tests:
  **87 passed** (59 core,11 real ROS,17 coverage).
- Frozen source archive integrity and six retained source suites: **46 passed**.
- Changed production modules pass flake8 (maximum line length99), compilation
  without source caches, and `git diff --check`.

Tests cover site coordinates/order freezing, the four source simulated messages,
navigation/action optional skips and required aborts, report errors, cancellation
before navigation/during pending acceptance/during dwell/between points, and
manual/localization loss taking precedence over optional-point failure.
Late accepted goals are canceled and no subsequent point is dispatched.
Shipped site templates remain invalid until coordinates are surveyed.

ROS tests used isolated domain187, `ROS_LOCALHOST_ONLY=1`, the original target
dependency overlay and this worktree's generated interfaces. External navigation,
localization and command-state providers were substitutes. No hardware or velocity
publisher was started. Runtime execution required ordinary escalation because the
sandbox restricts DDS sockets. An intermediate test-only fixture registration
error was fixed and the complete87-test suite rerun successfully.

The source checkout remains persistent; an ASCII symlink and disposable caches
under `/tmp` avoided the host's ROSIDL/CMake limitations on Chinese source paths.
These checks establish software behavior only. Site survey, actual vehicle stop
and localization behavior, and low-speed acceptance under ticket#9 remain open.

# Unified task control

Ticket #2 provides a real ROS entry point and an injectable, nonblocking public
`TaskManager.start(task_type, request)`, `cancel(task_id='')`, `status()` API.
The supervisor owns identity, authorization, cancellation, terminal status and
reports. The task-specific `TaskHandler.prepare/begin` and `Operation.poll/cancel`
interfaces allow later inspection rules and coverage execution to reuse it.
Only the single-point `inspection` handler is enabled in this slice.

## ROS services and CLI

`nav2_mission_interfaces` defines these request-scoped services:

| Service | Request | Response |
| --- | --- | --- |
| `/task_control/start` (`StartTask`) | `task_type`, `request_json` | `accepted`, `task_id`, `message` |
| `/task_control/cancel` (`CancelTask`) | `task_id` (empty selects current) | `accepted`, `task_id`, `message` |
| `/task_control/status` (`GetTaskStatus`) | empty | `status_json` |

`ros2 run nav2_mission task_control_node` starts the services, and publishes the
same status as JSON on `/task_control/status_snapshot`. It starts no navigation,
localization or hardware nodes. Run only with the approved whole-vehicle launch
and command safety integration; this ticket's tests use substitutes only.

Example request file:

```json
{"frame_id":"map","point":{"id":"gate","type":"security","x":1.0,"y":2.0,"yaw":0.2,"stay_time":2.0,"allow_skip":false}}
```

Coordinates and yaw (radians) must be finite, stay time nonnegative, and point type
one of `security`, `facility`, `environment`, `dock`. Unknown options are rejected.
This slice runs one point; `allow_skip=true` is explicitly rejected until the
multi-point skip/required rules are enabled in #4.
All arrival actions are simulated; dock arrival does not assert docking/charging.

```bash
ros2 run nav2_mission task_control start inspection --file /path/to/request.json
ros2 run nav2_mission task_control status
ros2 run nav2_mission task_control cancel TASK_ID
```

A start freezes its request and returns a UUID. A second start is rejected while
a task is active, stopping, or the command layer has not confirmed an initial
stop. Cancel acceptance means the entire task has been asked to end; query status
until `active=false` and `cleanup_confirmed=true`. Repeated and idle cancellation
are idempotent; a different active task ID is rejected. The latest task remains
queryable after termination. The CLI times out explicitly and directs callers to
query status before retrying an ambiguous start.

Phases are `dependency_check`, `navigating`, `inspecting`, `stopping`, `finished`.
An empty service request returns `idle` before any task. `result` remains null
while active and becomes `succeeded`, `failed`, or `canceled` after physical stop
and owned action terminal. `cleanup_error=cancel_timeout: ...` explicitly exposes
blocked cleanup; it never releases a pending/unknown action. Emergency/manual
or fault holds can remain asserted after cleanup: a report is still generated,
and new starts remain inhibited. No automatic resume/reset is attempted.

## Boundary adapters

`task_adapters.py` defines immutable `LocalizationState`, `StopState` and
`ExecutionUpdate` values. Inject adapters at the manager constructor; they are
system boundaries, not manager internals. The ROS localization adapter consumes
`std_msgs/String` JSON on configurable `localization_topic` (default
`/task_control/localization`):

```json
{"valid":true,"frame_id":"map","source_stamp_sec":1793097000.0,"source":"rtk_ekf","x":0.0,"y":0.0,"yaw":0.0,"error":""}
```

The producer must retain the actual localization evidence source stamp; receiving
or republishing a message must not manufacture new localization evidence. Both
source age (using the node's configured ROS clock) and local receive age (steady
clock) are checked, as is the requested map frame. No initial pose is ever set.
AMCL and RTK/EKF health producers are connected in #7. This slice accepts neither
an absent producer nor a TF-only readiness claim. The task node and providers
must use the same `use_sim_time` setting.

`RosNavigation` uses the configured `navigate_action_name` (`/navigate_to_pose`)
and leaves the behavior-tree override empty, retaining the target vehicle's
Ackermann navigation/recovery configuration. Each operation retains the pending
acceptance, accepted handle and result future. A late acceptance after cancel is
canceled; cancellation response alone is never terminal. Old operation futures
have no callbacks into the next task's state. Transport exceptions request stop
and keep uncertain ownership blocked.

The command contract is independent of task internals (producer owned by #3):

- `/task_control/stop` (`std_msgs/Bool`): true inhibits auto commands; false asks
  for a new run without clearing safety holds. The task waits for a fresh state
  acknowledging `stop_requested=false` before submitting navigation.
- `/task_control/heartbeat` (`std_msgs/Empty`): emitted every 50 ms while the
  supervisor runs. The command layer must require it in the task-enabled launch.
- `/task_control/state` (`std_msgs/String`): `ready`, `stopped`, `stop_requested`,
  `manual_override`, `emergency_stop`, `fault`, `feedback_age_sec`. Unavailable
  feedback age is null. Booleans are typed; malformed/nonfinite state fails safe.
- `/task_control/emergency_stop` and `/task_control/reset_stop` remain owned by
  the command layer. A task never clears an emergency stop, resets it or resumes
  an old run. Legacy emergency/manual inputs are combined at that layer.

`stopped` must mean continuously fresh near-zero source and receive feedback
since the current stop episode, not a cached zero. The manager additionally
requires receipt after this task's stop request, fresh receive age, fresh feedback
age, `stop_requested=true`, and terminal result for every owned action. It sends
stop continuously while stopping/idle. Firmware and command watchdogs handle
SIGKILL/missing heartbeat. SIGINT and SIGTERM request cancellation and stop while
keeping the ROS context alive to receive owned action results and fresh stop
feedback, for at most the configured cancellation timeout. An unresolved action
or absent stop feedback is never recorded as confirmed cleanup.

Parameters `state_timeout_sec=1`, `cancel_timeout_sec=5`, `task_timeout_sec=600`
are configuration defaults, not verified vehicle performance. Reports are UTF-8
JSON in `report_dir` (default `~/.ros/task_reports`), containing ID, frozen config,
phase history, UTC start/end times, monotonic elapsed seconds, result, errors,
simulation marker and cleanup outcome.
Report I/O errors remain visible in status with `result=failed`, `report_error`
and `cleanup_confirmed`; they are never a fully normal success.

## Verification

Tests are at the already approved public control seam, with injected external
localization/navigation/stop providers. Real ROS tests exercise generated service
transport, action acceptance/result/cancel and the CLI; they do not start hardware
or publish velocity. Use a dedicated domain and local-only transport:

```bash
source /opt/ros/humble/setup.bash
source /home/wyz/ros2/new_base_plate_indoor_ws/install/setup.bash
colcon build --packages-select nav2_mission_interfaces nav2_mission --allow-overriding nav2_mission
source install/setup.bash
export ROS_DOMAIN_ID=161 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/property-inspection-issue-2-ros-logs
PYTHONPATH=src/nav2_mission:$PYTHONPATH python3 -m pytest src/nav2_mission/test/test_task_control.py src/nav2_mission/test/test_task_control_ros.py -q
```

Real vehicle motion remains unverified here and is gated on later stop,
localization and whole-vehicle integration plus low-speed acceptance (#9).

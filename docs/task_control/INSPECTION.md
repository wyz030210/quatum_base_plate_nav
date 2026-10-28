# Site template inspection

The unified `inspection` task accepts either the existing direct-point request
or explicit point/template YAML files collected for the target site. It reads
and validates both files before admission and freezes the selected template's
ordered points in task status and the report. There are no fallback demo
coordinates and no task-layer navigation retries.

## Preparing a site

Copy `config/inspection_site_points.template.yaml` and
`config/inspection_site_missions.template.yaml` from the `nav2_mission` package
to an operator-owned directory. Fill every `pose.x`, `pose.y` and `pose.yaw` with
surveyed values in the chosen global frame (yaw in radians). The supplied null
coordinates deliberately fail validation until replaced. Point IDs use 1–64
ASCII letters, digits, underscores or hyphens, beginning with a letter or digit.

The source format is preserved: a top-level `points` list contains `id`, `name`,
`type`, `pose: {x, y, yaw}`, `stay_time` and `allow_skip`. Types are `security`,
`facility`, `environment` and `dock`; stay time is nonnegative seconds. A top-level
`missions` list contains `id`, `name`, an ordered nonempty `points` ID list,
`nav_bt: ""` and `stop_on_required_point_failure: true`. IDs cannot be repeated
within a document or selected mission. Text is trimmed as in the source loader.

The loader rejects missing/unreadable files, malformed/unsafe YAML, unknown
references, empty documents/missions, missing site fields, nonfinite/invalid
numbers, nonboolean skip flags and unsupported options at every schema level.
All definitions in the files are validated, including unselected templates.
`nav_bt` may be omitted or empty only: Nav2 selects the target vehicle's configured
Ackermann tree. Source Burger/Spin trees and arbitrary tree overrides are rejected.
The required-point failure rule is fixed; omitted
`stop_on_required_point_failure` defaults to true, and any other value is rejected.

Create a request JSON using absolute paths accessible to the supervisor:

```json
{
  "frame_id": "map",
  "points_file": "/path/to/site_points.yaml",
  "missions_file": "/path/to/site_missions.yaml",
  "mission_id": "site_patrol"
}
```

```bash
ros2 run nav2_mission task_control start inspection --file /path/to/request.json
ros2 run nav2_mission task_control status
ros2 run nav2_mission task_control cancel TASK_ID
```

These commands use the existing `/task_control/start`, `/task_control/status`
and `/task_control/cancel` services. Editing the files after acceptance does not
change that run. A direct request remains supported in its existing shape
`{"frame_id":"map","point":{...}}`; direct points may now use `allow_skip=true`.
Direct requests may omit `name` (defaults to ID) and `allow_skip` (defaults to false).
Direct points and site-file options cannot be mixed.

## Execution and reporting

Navigation must succeed before the arrival action begins. The four actions
retain the source simulated classifications/messages: security inspection,
facility/photo inspection, environment inspection, and dock arrival/task finish.
Stay time is polled without blocking service callbacks. A dock record always has
`charging_verified=false`; it provides no docking or charging evidence. No actual
camera, sensor, charger or cleaning actuator is connected by this ticket.

An optional point's terminal navigation/action failure is recorded as `skipped`
and the next point is visited once. A required point's failure ends the mission.
User cancellation, takeover, localization loss, safety loss and timeout end the
whole task before optional-point rules can dispatch another goal. Transport
uncertainty remains blocked until owned operations end and fresh stop is confirmed.
Cancellation while a goal response is pending still revokes a late accepted goal.

The arrival interface is replaceable via `Inspection(navigation, actions=...)`:
`actions.start(task_id, point, now)` returns an operation exposing `poll(now)`
and `cancel()`. The shipped implementation is deterministic simulation only.
Terminal `ExecutionUpdate` values report success/failure, raw `error`, and an
optional `details.message`. Unresolved transport errors use a nonterminal error
so the supervisor can cancel and retain ownership instead of skipping blindly.

The existing task report associates the task UUID with frozen site/template
configuration, UTC times, phase history, execution result, errors and cleanup.
`details.points` retains source report fields (mission/point IDs and names,
category, start/end/duration, navigation/action outcomes, status, skip rule and
message), plus task ID, raw error and explicit simulation/charging markers.
The summary distinguishes `succeeded`, `failed`, `skipped`, `canceled`,
`total_points` and `unvisited`. A canceled current point is never counted as an
optional skip; points not started remain unvisited. Cancel before navigation
still produces an empty point list and zero-count summary for the accepted template.

Report write errors remain queryable with `result=failed`, `report_error`, an
empty `report_path` and retained point outcomes; `execution_result` records the
execution outcome separately. Cleanup still requires fresh stop confirmation.
Handler `initial_details(config)` is an optional admission-time report hook;
preparation/initialization errors reject the request before authorization. The
supervisor also retains final operation details produced during stopping.

## Software demonstration

`test_inspection_tasks.py` exercises the approved start/cancel/status seam with
substitute localization, stop, navigation and failure-injected arrival providers.
Source configuration, sequential-rule, simulated-message and report examples are
adapted at this runtime seam; the unchanged source suites remain in the verified
source snapshot. `test_inspection_tasks_ros.py` drives the real generated services
and Nav2 client against a substitute action server, including success, abort,
late acceptance cancellation and cancellation during dwell. The task node starts
no localization, navigation or hardware nodes itself.

Use the build/isolation instructions in `README.md` and a dedicated localhost
ROS domain. Passing these tests is software evidence only. Site coordinates,
vehicle localization/stop performance and low-speed physical acceptance remain
unverified; this ticket gives no vehicle motion authorization.

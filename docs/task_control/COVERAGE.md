# Unified rectangle coverage

The `coverage` task uses the existing `/execute_coverage_navpath` bridge. The
manager owns one outer goal; that bridge alone submits `ComputeCoveragePath`,
`NavigateToPose` to the first pose, and `FollowPath` with the complete planned
path. No source coverage motion node is needed. Inspection remains registered
alongside coverage.

Submit a request through the existing task services or CLI:

```json
{
  "frame_id": "map",
  "polygons": [[[0, 0], [8, 0], [8, 8], [0, 8]]],
  "planner": {"operation_width": 0.7, "default_swath_angle": 0.3}
}
```

```bash
ros2 run nav2_mission task_control start coverage --file /path/to/rectangle.json
ros2 run nav2_mission task_control status
ros2 run nav2_mission task_control cancel TASK_ID
```

This slice accepts one simple rectangle, with four corners or an explicitly
closed fifth point, in the requested localization frame. Coordinates must be
finite and representable, edges nonzero and perpendicular, and opposite edges
equal. `planner` accepts only positive finite `operation_width` and finite
`default_swath_angle` (radians). Unknown fields are rejected. Polygon editing and
holes extend this interface in #6.

The manager freezes a deep copy on admission. The `PrepareCoverage` service
reserves that snapshot for an explicit outer goal UUID without changing the
compiled OpenNav action schema. The bridge rejects concurrent preparations,
freezes its runtime parameters while reserved/active, and verifies that the outer
goal has the reserved UUID. A canceled preparation and a rejected outer goal
retain ownership until reservation disposal is acknowledged.

Before planning, the bridge atomically sets only the requested width/angle and
`task_parameter_owner` on the configured `planner_node_name` (default
`/coverage_server`). It then reads back the full effective planner configuration
under that lease. Existing vehicle width/length, turning radius, swath spacing,
headland, path mode and other settings come from the planner's actual configured
values; reference constants do not overwrite them. The effective rectangle must
have room for twice the headland width plus the operation width. The planner
rejects ordinary parameter edits, owner takeover and lifecycle deactivation until
the matching owner releases the lease. A delayed acquire response is drained
before owner lookup/release, so it cannot acquire an abandoned lease later.
The planner serializes computation and parameter changes without taking its
NodeParameters mutex from inside the compute lock.

The bridge retains both existing costmap checks, before approach and immediately
before FollowPath. It also rejects nonfinite positions, inconsistent frames and
invalid quaternions in a returned path. It approaches the actual first pose,
using the target vehicle's configured navigation tree, and sends the untrimmed
path to the target controller. A revisited point near the vehicle does not imply
that earlier path segments were completed. A planning result alone cannot finish
the task, and no failure schedules another segment or recovery task.

`/coverage/execution_state` publishes correlated JSON containing the outer goal
and task IDs, predetermined child goal UUIDs, phases, child ownership states,
planned path, available feedback, error code and effective planner settings.
Child UUIDs are deterministic UUID5 values derived from the outer goal and stage
and are announced before dispatch, allowing the conflict detector to recognize
owned Nav2 actions. Consumers should correlate both IDs. Feedback includes
`stage`; a phase transition clears old approach/follow feedback. Available
distance and speed are execution feedback, not a measured area coverage rate.

Coverage status includes `preparing`, `planning`, `validating`, `approaching`,
`following`, `finalizing` and the common `stopping`/`finished` phases. Internal
failures expose the bridge error and request the common stop immediately, before
waiting for delayed child terminals or parameter release. Normal completion
requires complete FollowPath success, all owned children terminal, lease release
and a fresh common stop confirmation. Cancel is available during every stage.
Pending goal acceptance and late acceptance are retained and canceled; cancel
acknowledgement is not a terminal result. Unknown ownership remains blocked with
an observable cleanup error and rejects a new task.

Reports use the manager's `report_dir`. They include the task ID, frozen request,
full `details.effective_planner`, bridge phase history, planned path, available
feedback, child outcomes, execution result and cleanup outcome. Report I/O errors
remain visible as `result=failed`, `execution_result`, and `report_error` after
motion cleanup. No cleaning quality or area threshold is used as a success gate.

Build `nav2_mission_interfaces`, `nav2_mission` and the locally adapted
`opennav_coverage` together. The bridge and planner must both contain the lease
implementation. Keep the approved whole-vehicle localization and common command
safety integration; start no duplicate Nav2, coverage, hardware or source motion
nodes. Existing raw bridge/preview interfaces remain available for legacy use;
approved task operation must use the unified entry. Whole-vehicle entry policy
and raw third-party goal conflict handling join in later tickets.

Software verification is recorded in [COVERAGE_VALIDATION.md](COVERAGE_VALIDATION.md).
Physical execution, stopping performance and actual AMCL/RTK acceptance remain
pending the common localization/vehicle integration and #9 field conditions.

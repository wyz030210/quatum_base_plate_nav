# Session region editor and reference statistics

The unified `task_control_node` contains the RViz editor component. All service,
point and manager callbacks use its default mutually exclusive callback group.
The component calls the same manager start API as `task_control start coverage`;
it creates no motion action client. `coverage_navpath_bridge_node` is the sole
coverage motion owner. Keep the bridge's manager-required mode enabled.

Use RViz Publish Point in the task frame (`map` by default), with
`share/nav2_mission/rviz/coverage_task.rviz`. `/clicked_point` adds draft vertices.
The selected frame must match each clicked point. Session data is lost on restart.

```bash
ros2 run nav2_mission task_control region configure --request '{"frame_id":"map","operation_width":0.60,"default_swath_angle":1.57}'
# Draw the outer ring in RViz, then close it.
ros2 run nav2_mission task_control region close_outer
ros2 run nav2_mission task_control region begin_hole
# Draw a strictly interior hole, then close it. Repeat begin_hole as needed.
ros2 run nav2_mission task_control region close_hole
ros2 run nav2_mission task_control region status
# Inspect the returned region, width and angle before confirming with start.
ros2 run nav2_mission task_control region start
ros2 run nav2_mission task_control status
ros2 run nav2_mission task_control cancel
```

`region undo` removes the latest draft vertex; `region clear` discards the whole
session region. A closed outer ring is kept immutable until clear, while holes
may still be added. `region point --request '{"x":1,"y":2}'` is the CLI equivalent
of a clicked point. The project service `/task_control/region` (`RegionCommand`)
accepts the same operations and JSON parameters, returns accepted/message/task_id
and a complete session state. `/coverage/editor_state` publishes that state.
Rejected edits return a reason, including frame mismatch, invalid geometry and
active-task/stop-pending state. Configuration changes validate before replacing
the confirmed session values. Start copies the complete current region/config;
a concurrently submitted edit cannot alter the accepted request.

Coordinates must be finite numeric local-frame metres within +/-10,000,000;
boolean vertices, self-intersection, holes outside/touching/intersecting each
other or the outer ring, and regions whose width erosion is empty are rejected.
The erosion tests feasibility; it does not alter the operator's boundary.
Angles are radians within [-pi, pi]. Width must be finite and positive.
The request's shared validator is also used by direct coverage requests and by
the bridge, so CLI clients cannot bypass these constraints.

Visualization topics are `/coverage/region_markers`, `/coverage/planned_path`
and `/coverage/executed_trace` (MarkerArray with one line per valid segment).
The accepted frozen region is displayed while a task is active. Progress and
reference statistics appear in task status/reports and
`/coverage/reference_statistics`. `path_completion_ratio` comes from the planned
path length and controller remaining distance when available.
`trace_estimated_area_ratio` buffers only observed coverage-phase trajectories
by the frozen operation width and clips their union to the selected region,
excluding holes. It uses common localization JSON pose evidence in the task
frame, checks source age/validity, splits on missing/invalid evidence, and reports
sample and gap counts. It never draws a line across a data gap. These geometric
reference estimates do not measure brush operation or cleaning quality and are
not acceptance thresholds. `collision_count` is null because no collision
measurement provider is configured.

The pure `coverage/{models,region_editor,metrics}.py` modules and original region
and metrics test bodies were migrated from the complete frozen source archive
`docs/source_snapshots/property_inspection-20261027.tar.gz`. Changes add strict
hole contact checks, bounded input validation, safe closed-ring undo, and swept
area calculation across separate evidence segments. The source ROS motion node
and its AMCL dependencies are not part of this component.

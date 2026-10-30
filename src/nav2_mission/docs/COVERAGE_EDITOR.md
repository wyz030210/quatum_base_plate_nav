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

`region undo` removes the latest draft vertex. After closing a ring, undo first
reopens the most recently closed hole, or the outer ring when there are no
holes; a further undo removes its latest vertex. Undo on an empty hole draft
abandons that draft and returns to the closed outer ring. Other rings are preserved.
`region clear` discards the whole session region. `region point --request '{"x":1,"y":2}'` is the CLI equivalent
of a clicked point. The project service `/task_control/region` (`RegionCommand`)
accepts the same operations and JSON parameters, returns accepted/message/task_id
and a complete session state. `/coverage/editor_state` publishes that state.
Rejected edits return a reason, including frame mismatch, invalid geometry and
active-task/stop-pending state. Configuration changes validate before replacing
the confirmed session values. Start copies the complete current region/config;
a concurrently submitted edit cannot alter the accepted request.

Coordinates must be finite numeric local-frame metres within +/-10,000,000;
boolean vertices, self-intersection, holes outside/touching/intersecting each
other or the outer ring, regions whose width erosion is empty, and suspected
narrow passages are rejected. For the latter, after removing collinear vertices,
the validator samples every boundary segment at intervals no greater than one
eighth of the operation width, and casts an inward normal up to that width. A
hit on another boundary rejects the region, except where the hit is on the
adjacent edge at a convex corner or both points are on the exterior of a wholly
convex shell. The latter permits smoothly rounded as well as sharp convex tips;
a convex taper with a narrow tip is consequently allowed if the overall erosion
is nonempty. Hits involving holes are still checked. This catches representative short, curved and
tapered passages as well as long parallel ones, while allowing normal sharp
corners and circular holes in the tested cases. The test is conservative: it may
reject geometries whose local passage is not intended for travel. It is also
finite: narrow features between samples, features shorter than the sample
spacing, and configurations hidden by the convex exemptions may pass. Near
floating-point tolerance or densely faceted corners, results can differ from an
ideal continuous width calculation. It does not certify vehicle traversability;
operators must inspect the region and planned path.

To bound work in the shared manager callback, inputs are rejected as too complex
when they have more than 64 holes, more than 4,096 vertices, more than 4,096
normal probes, or a probe-count times segment-count product above 1,000,000.
Widths at or below 8 micrometres also exceed the numerical sampling limit. Each
probe intersects the whole polygon boundary, so cost depends on both the probe
and vertex counts. The checks do not change the selected boundary.
Angles are radians within [-pi, pi]. Width must be finite and positive.
The request's shared validator is also used by direct coverage requests and by
the bridge, so CLI clients cannot bypass these constraints. Direct requests use
`{"frame_id":"map","polygons":[[[0,0],[12,0],[12,12],[0,12]],[[8,8],[9,8],[9,9],[8,9]]],"planner":{"operation_width":0.7,"default_swath_angle":0.4}}`.
The first ring is the outer boundary; subsequent rings are holes. Rings are
normalized and closed without changing the selected area. If a direct request
omits width, geometry is checked immediately and width feasibility is checked
after the bridge acquires and reads the authoritative planner settings.

Before any planning goal, the bridge repeats geometry/width validation against
the leased effective width and checks that erosion by the effective headland
plus half the operation width leaves usable area. Headland is applied only when
the bridge requests its generation. These feasibility checks do not replace the
planner or either existing complete-path validation. Vehicle width, turning
radius and other fixed settings remain the configured planner's values.

Visualization topics are `/coverage/region_markers`, `/coverage/planned_path`
and `/coverage/executed_trace` (MarkerArray with one line per valid segment).
The accepted frozen region is displayed while a task is active. Progress and
reference statistics appear in task status/reports and
`/coverage/reference_statistics`. `path_completion_ratio` comes from the planned
path length and controller remaining distance when available.
It uses only FollowPath feedback, and preserves the last available value; a
terminal success does not fabricate a final zero-distance feedback sample.
`trace_estimated_area_ratio` buffers only observed coverage-phase trajectories
by the leased effective `operation_width_m` and clips their union to the selected region,
excluding holes. It uses common localization JSON pose evidence in the task
frame, checks source and pose age/validity, and splits on missing/invalid evidence
or changed source/map/profile/evidence identity. It reports sample and gap counts,
including explicitly invalid evidence before the first valid sample, and
`trace_complete=false` when data is absent or a gap was observed. It never draws
a line across a data gap. These geometric
reference estimates do not measure brush operation or cleaning quality and are
not acceptance thresholds. `collision_count` is null because no collision
measurement provider is configured.

The pure `coverage/{models,region_editor,metrics}.py` modules and original region
and metrics test bodies were migrated from the complete frozen source archive
`docs/source_snapshots/property_inspection-20261027.tar.gz`. Changes add strict
hole contact checks, bounded input validation, safe closed-ring undo, and swept
area calculation across separate evidence segments. The source ROS motion node
and its AMCL dependencies are not part of this component.

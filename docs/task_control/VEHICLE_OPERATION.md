# Vehicle task operation

The indoor `robot_bringup/indoor_amcl.launch.py` and outdoor
`robot_bringup/outdoor_ekf.launch.py` accept `use_task_control:=true`. This
starts one task manager and one localization provider, requires the common
command-entry heartbeat, and enables the existing coverage planner and bridge.
The selected RViz configuration sends 2D Goal Pose to
`/task_control/goal_pose`; it does not send an unmanaged Nav2 goal. Region clicks
belong to the managed editor. The legacy auto-executing region selector is
inactive in this mode. Do not separately start old patrol, coverage selector,
source simulator or another manager. With task mode disabled the original
ordinary navigation and legacy coverage settings remain available.

## Provision a site

Survey the actual map, coordinate frame, inspection points and templates; the
shipped null-coordinate examples are intentionally invalid. Follow
[INSPECTION.md](INSPECTION.md) and [COVERAGE.md](COVERAGE.md). Verify the
vehicle's map registration, RTK datum and heading calibration independently;
a matching map checksum alone does not establish localization accuracy. The
full source, frame, timing and stop prerequisites are in
[LOCALIZATION.md](LOCALIZATION.md).

Choose and record an explicit profile name and the expected 64-character
lowercase map SHA256. The provider computes the digest from the received ROS
OccupancyGrid, including its metadata and occupancy values. To obtain a
candidate without running a vehicle or sending a movement command, start only a
map server for the selected YAML and a temporary provider in an isolated ROS
domain, activate the map server, and inspect the provider output:

```bash
# Terminal 1, with the intended workspace sourced. No hardware or navigation nodes.
export ROS_DOMAIN_ID=195 ROS_LOCALHOST_ONLY=1
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/absolute/site/map.yaml -p use_sim_time:=false
# Terminal 2, same isolated domain
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 run nav2_mission localization_provider --ros-args -p source:=amcl -p 'map_id:="0000000000000000000000000000000000000000000000000000000000000000"' -p localization_profile:=preflight -p use_sim_time:=false
# Terminal 3, same isolated domain
ros2 topic echo --once /task_control/localization
```

Read `observed_map_id` from the JSON string, verify the loaded map against the
site's independently approved map, and provision that digest as `task_map_id`.
The provider remains invalid in this preflight; it does not authorize movement.
Use a separate domain from any live vehicle and shut the temporary nodes down
before starting the vehicle. A future map edit changes the digest and requires a
new review. Never copy `observed_map_id` automatically into a running profile.

For a real vehicle set `MAP_SHA256` and `SITE_PROFILE` to the independently
verified values, select wall time explicitly, and provision both values, e.g.:

```bash
ros2 launch robot_bringup indoor_amcl.launch.py map:=/absolute/site/map.yaml use_sim_time:=false use_task_control:=true task_map_id:="$MAP_SHA256" task_localization_profile:="$SITE_PROFILE"
# Or the outdoor_ekf.launch.py entry with the registered map and site datum.
```

These launch commands start the vehicle's existing hardware nodes. Run them only
under the vehicle operating procedure and after the physical acceptance checks.
The overall task timeout defaults to 600 seconds; set `task_timeout_sec` for the
planned site operation, including planning, approach and full path execution.
Cleanup is controlled separately by `cancel_timeout_sec` and stays blocked if
termination or fresh stopped evidence is missing.
All components must use the same clock; use `use_sim_time:=true` only when all
sources and `/clock` are deliberately configured for simulation.

## Start, switch and stop

Use the same `task_control` client for inspection, coverage and ordinary pose
navigation. A navigation request is a finite map-frame pose in metres/radians:

```bash
ros2 run nav2_mission task_control start navigation --request '{"frame_id":"map","point":{"x":1.0,"y":2.0,"yaw":0.0}}'
ros2 run nav2_mission task_control status
ros2 run nav2_mission task_control cancel TASK_ID
```

To draw a coverage region in task-mode RViz, use Publish Point to place the
outer boundary, then close it with `region close_outer`. For each exclusion,
run `region begin_hole`, place its vertices and run `region close_hole`.
`region undo` removes the latest edit and `region clear` resets the session.
The task RViz shows the submitted region, accepted path and observed trace.
Set the site-specific width and swath angle, inspect validity, then submit:

```bash
ros2 run nav2_mission task_control region configure --request '{"frame_id":"map","operation_width":0.7,"default_swath_angle":0.3}'
ros2 run nav2_mission task_control region status
ros2 run nav2_mission task_control region close_outer
# Optional: region begin_hole, draw exclusion, then region close_hole.
ros2 run nav2_mission task_control region start
```

The editor is in memory and refuses edits or submission during an active task or
unconfirmed stop. Check each command's `accepted`, `message` and returned
`state`; invalid geometry must be corrected before submission. Session state is
not restored after restart.

An accepted cancellation starts cleanup; it is not proof of stopping. Query
status until `active` is false and `cleanup_confirmed` is true before issuing
a new task. Active or stopping tasks reject replacement requests. A manual
control takeover terminates the task; returning to automatic does not resume it.
A held software emergency stop requires its inputs to be cleared and explicit
`/task_control/reset_stop`, and reset does not itself start a task. Real motion
requires valid localization, fresh stopped/feedback evidence, a ready common
command gate and all site-specific safety conditions.

The manager associates its own Nav2 and coverage goals by UUID and monitors the
action graph and status for unknown active goals. Nav2's BT-generated FollowPath
UUID has no parent field in ROS status. Its attribution is a limited inference
under the configured single-node BT client topology; an unexpected client or
known unknown goal terminates the task and prevents admission. The raw
`/goal_pose`, `/navigate_to_pose`, `/navigate_through_poses`, `/follow_path` and
coverage action interfaces remain reachable to other ROS participants. This is
conflict detection, not access control: discovery and cross-topic ordering can
leave attribution ambiguous, and the same node name is not authentication.
Restrict the deployment to the approved clients and monitor reported errors;
never claim that arbitrary third-party commands are prevented.

Simulation or substitute-based tests do not establish real stopping distances,
localization registration or field readiness. Actual low-speed vehicle
acceptance remains open under #9.

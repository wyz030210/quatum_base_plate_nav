# Coverage control window

The control window is installed as `ros2 run nav2_mission coverage_control_gui`.
It requires a graphical session and `python3-tk` (declared in package.xml).
Use the launch entry points below so its region service and dedicated preview
planner are present. Closing the window preserves the region while its editor
node is alive; restarting the editor discards session data.

## No-vehicle map preview

After building and sourcing this branch, run:

```bash
export ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1
ros2 launch wheeltec_nav2 coverage_preview.launch.py
```

The default map is the installed `wheeltec_nav2/map/indoor_map.yaml`. Override
`map:=/absolute/path/map.yaml`, `params_file:=/absolute/path/coverage_params.yaml`
or `rviz_config:=/absolute/path/config.rviz` as needed. The launch starts a map
server, a static preview frame, an edit-only region service, a compute-only
planner, the control window and RViz. It starts no task supervisor, localization,
vehicle driver, controller or velocity publisher. Standalone region `start`
requests are explicitly rejected. `use_coverage_gui:=false` and
`use_rviz:=false` allow headless software checks.

On a fresh launch the window reads the preview planner's actually loaded
`operation_width` and `default_swath_angle`, then initializes the shared region
configuration. Fields remain empty/disabled until those values are available;
there is no UI fallback to 0.6 m. Explicit existing session changes are preserved
when only the window is reopened, including partial CLI configuration.

In RViz select **Publish Point**, draw the outer ring, then use **闭合外边界**.
Use **开始孔洞** and **闭合孔洞** for exclusions; **撤销** and **清空** operate on
the shared region session. Set the operation width and swath angle in radians;
valid edits apply after 500 ms without input, or immediately with Enter/**应用**.
The status line shows the confirmed values. **生成覆盖路径** also applies the
visible values before dispatch, even if the debounce has not completed.
Invalid values are not applied. Editing clears the old displayed preview;
late results for a changed region are discarded. Planning failures display the
action status/error code; the planner log contains the detailed geometric reason.
The preview path is `/coverage_ui/preview_path`, shown as Coverage Preview in RViz.

These edits affect this runtime session and subsequent planning/task requests;
they do not rewrite YAML or alter an already active task. Inputs are locked
while an operation/task is active. A complete restart reloads the selected YAML.
ROS normally resolves the default file in the installed package, not `src`.
After editing the source YAML rebuild/source its package, or select it explicitly:

```bash
ros2 launch wheeltec_nav2 coverage_preview.launch.py params_file:=/absolute/source/coverage_params.yaml
```

Vehicle launch entries use `coverage_params_file:=...` for the same purpose.

Preview uses a dedicated planner under `/coverage_preview`. It leases that
planner's requested width/angle, reads its effective spacing/headland/turn radius,
then releases ownership after the action terminates. It does not mutate the task
execution planner at `/coverage_server`. A timeout attempts cancellation and
waits for the terminal result; if termination or parameter release is uncertain,
restart the preview launch rather than assuming its planner is available.

Closing the window cancels its pending preview; it does not cancel a managed
vehicle task. Map center-sample diagnostics are only a visual aid. Preview does
not automatically route around map obstacles and does not replace the execution
bridge's costmap/footprint checks. A generated path is not evidence of physical
tracking, stopping or cleaning performance.

## Vehicle coverage startup

The indoor and outdoor vehicle launch entries automatically open the same
window when coverage is enabled (`use_coverage:=true`, also enabled by
`use_task_control:=true`). `use_coverage_gui:=false` disables the window and its
preview planner for a headless deployment. This flag is forwarded through the
vehicle, localization and navigation launch chain.

With `use_task_control:=true`, the window also has **启动覆盖任务** and
**取消任务并停车** buttons. Start requires an explicit confirmation and submits
the session through `/task_control/region` into the existing task manager.
Cancellation uses `/task_control/cancel`, independently of a running preview;
wait for task status `active=false` and `cleanup_confirmed=true`. Opening or
closing the window never starts, resumes or cancels an execution task.
The existing localization, mutual exclusion, planner snapshot and stop gates
remain authoritative; the window cannot bypass them. Preview is refused while
a task is active or cleanup is pending.

Without the task manager, the window is preview-only and uses an edit-only
service; the legacy auto-executing point selector is disabled while the window
is enabled, so drawing/closing a region never triggers vehicle motion. The old
selector remains available explicitly with `use_coverage_gui:=false` in legacy
mode. Launch only one editor and control window per robot/domain.

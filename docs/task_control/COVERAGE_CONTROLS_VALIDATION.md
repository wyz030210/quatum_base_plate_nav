# Coverage control window validation

Validated on ROS 2 Humble with localhost-only ROS domains, fake vehicle
boundaries for execution tests, and the real OpenNav planner for static preview.
No hardware nodes or vehicle movement were used.

- `nav2_mission`, `wheeltec_nav2` and `robot_bringup` built into a separate overlay;
  the installed launch files and Python entry points were exercised.
- 122 tests passed across the new controls, real manager/bridge cancellation,
  indoor/outdoor launch propagation, existing region geometry, legacy preview
  and Ackermann controller policy. Four existing Shapely deprecation warnings
  remain. Targeted final launch/control checks were repeated after cleanup edits.
- A standalone launch loaded the actual `indoor_map.yaml`; the installed control
  backend submitted a closed 5 m square in a free map area and the real dedicated
  planner returned 40 path poses. Center samples did not hit occupied or unknown
  cells. This was a static preview, not navigation or footprint clearance.
- A graphical window started and exited on SIGINT with exit code 0 and no
  traceback. The full installed preview launch then automatically opened both
  the window and RViz; the prior five-vertex test selection was restored.
- Controls do not start a task on startup. A preview parameter change leaves
  the separate execution planner unchanged. Changed regions discard pending
  results; failed planning never publishes a successful preview. Task cancellation
  is dispatched even while preview is pending. The real manager/bridge boundary
  test verifies active-task edit refusal, cancellation and confirmed cleanup.

Commands and operator behavior are in
[COVERAGE_CONTROLS.md](../../src/nav2_mission/docs/COVERAGE_CONTROLS.md).
GUI visibility requires a desktop/display session and `python3-tk`; headless
deployment uses `use_coverage_gui:=false`. Parameter release or action termination
uncertainty requires restarting the dedicated preview launch. The region geometry
and physical acceptance limitations in the other task-control documents remain.

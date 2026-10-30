# Field acceptance preparation (#9)

This command records software and configuration evidence; it does not launch
hardware or vehicle nodes, send velocity, start/cancel tasks, reset stops or
claim field acceptance. The command reads the ROS graph and the three JSON
status topics, if available. It makes no service calls. Collection can run with
no ROS installation; unavailable information is explicit. It does not check
physical performance or decide whether the vehicle may move.

Build the integrated workspace and source its ROS environment. On the deployed
machine, from a writable evidence directory, substitute the actual checkout and
configuration paths (leave unknown options out rather than inventing values):

```bash
ros2 run nav2_mission field_preflight \
  --repo /path/to/deployed/source \
  --source amcl --profile actual_profile \
  --map-config /path/to/map.yaml \
  --vehicle-config /path/to/vehicle.yaml \
  --datum-config /path/to/datum.yaml \
  --config /path/to/other_active_configuration.yaml \
  --build-evidence /path/to/build_versions.yaml > preflight.json
```

Use `rtk_ekf` for that selected source, and repeat separately for the other
actual profile at the site. Supply the real active files, including parameter
files for geometry, limits, localization, launch and firmware where available;
paths and SHA-256 hashes identify the files, but do not prove those files were
loaded. Include the map image and its hash separately if needed. The command
records ROS distribution and overlay paths; capture deployed package, firmware,
build and dependency versions in a YAML evidence file, with provenance. It does
not independently determine installed binary revisions from the source Git SHA.
Record build/version uncertainty, uncommitted changes, site and operator details
in the evidence. Configuration files may contain sensitive data: review the
output before sharing it. Parsing errors leave content null and retain a hash
when the file was readable.

Read `ros_graph`, the localization, safety and task snapshots, and error details.
A missing topic or unavailable query is not evidence of a healthy or unhealthy
vehicle. `selected` records operator input; `observed` records a single received
status message. Compare source, profile, map ID, frame and freshness with the
actual deployment. These are instantaneous observations, not proof of
continuous health, registration or safety; ROS remappings can change graph names.
The configured geometry and limits are not measured vehicle performance. Keep
actual datum/heading registration, physical geometry, measured stopping results,
and all unexecuted field cases null or `not_run`/`blocked` until observed.

Before any later vehicle operation, the responsible operator must establish the
site conditions and independently review [vehicle task operation](VEHICLE_OPERATION.md),
the current [localization](LOCALIZATION.md),
[inspection](INSPECTION.md), [coverage](COVERAGE.md), task control and vehicle
safety instructions. Record software regression commands, versions, exit codes,
logs and the source/installed-build provenance in `software_checks` separately
from field results. Do not infer firmware watchdog or real stopping performance
from substitute tests. Where present, reuse the available Ackermann simulation;
otherwise retain the substitute integration results and explicitly record that
no simulation was available.

When actual site testing is authorized and possible, save a separate evidence
copy for each profile and record start/end time, task ID, phases, result,
cleanup/stop confirmation, report paths, measurements and observations for:

- Ordered multi-point inspection, simulated actions, optional/required failures,
  cancellation during navigation and dwell, and point/summary reports.
- Coverage including an excluded hole, path validation, first-pose approach,
  entire path tracking and fresh confirmed stop. Preview alone is not execution.
- Cancellation at each coverage stage; manual takeover and no automatic resume;
  held emergency stop and explicit reset; invalid stop confirmation and source loss.
- Each real AMCL and RTK/EKF source independently, its freshness, loss and
  recovery, map/ENU registration, datum and heading calibration.
- Command and feedback interruption and actual measured stop; independently
  establish firmware/lower-layer behavior for converter process loss.
- Task switching, ordinary navigation and accessible reports.

Mark a case `blocked` or `not_run` when prerequisites or evidence are missing;
record failures honestly. Do not mark an entire case passed on a partial result.
Do not interpret simulated inspection as real sensing/charging, or a configured
limit as a measurement. No cleaning-quality or no-miss claim is implied. Ticket
#9 remains open until its actual field acceptance criteria have been fulfilled.

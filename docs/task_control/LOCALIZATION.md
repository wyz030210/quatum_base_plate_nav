# Unified localization and stop evidence

The `localization_provider` executable publishes schema 1 JSON on
`/task_control/localization` for both `source:=amcl` and `source:=rtk_ekf`.
It never sets an initial pose, broadcasts TF, initializes an estimator or creates
another task coordinator. Task start/cancel/status remain the shared services.

## Required profile configuration

The provider and `task_control_node` must receive the same nonempty `map_id` and
`localization_profile`; set `require_localization_schema:=true` on the manager.
These values are manager metadata, separate from the strict inspection/coverage
handler configuration. Omitted request identity defaults to the selected manager
profile; an explicit mismatch is rejected. Accepted tasks freeze source, profile,
map identity and evidence generation. Recovery cannot resume an old task.

`map_id` is SHA256 of the canonical JSON produced by `map_digest`: map frame,
width/height/resolution, origin position/quaternion and occupancy data. Publication
and load timestamps are excluded. The expected digest must be provisioned from
the selected, independently verified map. `observed_map_id` in provider status is
available even while invalid for preflight comparison; it is never auto-accepted.
A matching frame name or map digest does not establish RTK-to-map registration.
That registration, datum and heading calibration require the #9 field evidence.

All participants must use the same clock mode (`use_sim_time=false` on the
vehicle). Source message stamps are checked against that ROS clock; ingress
freshness uses a monotonic clock. A clock regression clears the AMCL witness and
dynamic evidence. Do not feed old recordings restamped as current measurements.

Provider parameters (all logged at startup):

| Parameter | Default / meaning |
| --- | --- |
| `source` | `amcl` or `rtk_ekf` |
| `global_frame_id`, `odom_frame_id` | `map`, `odom` |
| `base_frame_id`, `raw_base_frame_id` | `base_footprint`, `base_link` |
| `scan_frame_id` | `base_footprint` |
| `map_id`, `localization_profile` | empty; readiness stays closed until configured |
| `source_timeout_sec` | 1.0 s for raw/local/global odometry, TF and source-status transport |
| `future_tolerance_sec` | 0.1 s |
| `lifecycle_timeout_sec` | 1.0 s for AMCL lifecycle and parameter replies |
| `expected_amcl_node` | `/amcl` |
| `amcl_transform_tolerance_sec` | 1.0 s; must match AMCL's live parameter |
| `rtk_position_timeout_sec`, `rtk_quality_timeout_sec`, `rtk_heading_timeout_sec` | independently 1.0 s |
| `rtk_datum_lat`, `rtk_datum_lon` | 29.4898045, 106.5676103 |
| `rtk_calibration_id` | `east_clockwise_ant0.319`, the existing driver's convention |

Topic names are standard ROS names and may be remapped as one consistent profile:

| Input | Type / requirement |
| --- | --- |
| `/map` | OccupancyGrid, reliable transient-local depth 1 |
| `/Odometry` | raw FAST-LIO Odometry, progressing original stamps |
| `/odometry/filtered_odom` | local EKF Odometry, coherent odom/base frames |
| `/tf`, `/tf_static` | current coherent map-to-base chain; static TF alone cannot establish freshness |
| `/amcl_pose`, `/scan` | AMCL durable pose witness and LaserScan source |
| `/amcl/get_state`, `/amcl/get_parameters` | asynchronous active-state and frame/tolerance checks |
| `/localization/rtk_source_state` | RTK driver schema 1 original-source evidence |
| `/odometry/filtered_map` | RTK/EKF global Odometry |

The common status retains `x/y/yaw`, `pose_stamp_sec`, `source_stamp_sec`,
`source_age_sec`, frame IDs, source, map/profile and `evidence_generation`.
The pose comes from the coherent current TF chain. Source age is the oldest
required dynamic evidence, including ingress age. Repeated provider publication
never renews that evidence; the manager adapter adds transport elapsed time.
Explicit invalid states carry a reason and null unavailable pose/age fields.
Legacy schema 0 consumers/producers remain usable only when the manager does not
require schema 1.

## AMCL stationary behavior

A valid pose published by the live expected AMCL before provider startup is a
valid established-localization witness. It has no short pose-publication timeout:
stationary AMCL need not publish another corrected pose. Readiness additionally
requires fresh raw/local odometry, current composed TF, active lifecycle and
matching AMCL frame/transform-tolerance parameters.

A scan-processing witness joins an individual dynamic map-to-odom edge with the
observed scan whose stamp equals edge stamp minus AMCL transform tolerance
(with 1 microsecond rounding allowance). Scan epochs must advance and both
original and ingress ages are bounded. Fresh odom-to-base TF, unrelated edges,
static map-to-odom, future-dated old edges or restamped AMCL poses cannot renew it.
Map change, observed lifecycle reset and AMCL pose-writer replacement clear the
established witness; returning to the previous configuration does not reuse it.
Humble's Python subscriptions do not expose a per-message publisher GID. Endpoint
and live parameter checks verify the deployed topology; they do not authenticate
individual TF edges. The launch must preserve one map-to-odom authority.

## RTK original measurements

The existing `rtk_localization_node` adds a periodic positive/negative source
status; `/gps/fix`, antenna correction and `/odometry/gps` retain their legacy
behavior. Position, fixed GGA quality and true HDT heading have independent
sequences, validity, monotonic ages and reasons. A status sequence is not a
measurement. Joined `published_*` fields identify the evidence used by the last
GPS odometry output, so heading arriving after the last position is insufficient.

GGA/RMC UTC is normalized to integer microseconds. A duplicate epoch, even with
different formatting or another sentence type, cannot renew position freshness.
Quality requires independently advancing GGA fixed (`quality=4`) evidence and
must be within 1 s of the accepted position epoch; advancing RMC cannot repair
old/float/no-fix GGA quality. Recognized negative position/heading/quality states
invalidate immediately. Source expiry is independently 1 s in the driver; the
provider may impose stricter independent limits. Two position and GGA epochs
establish initial progression. Disconnect/reconnect clears usable measurements,
changes connection generation and requires newly acquired heading and advancing
position/quality; in-process UTC high-water marks survive reconnect.

RMC date is not mandatory. Midnight rollover is accepted only across a continuously
observed 60 s boundary with a gap no larger than the 1 s evidence deadline.
Large UTC jumps or ambiguous rollover after outage fail closed and may require
explicit driver/profile reinitialization. This is ingress/progression provenance,
not absolute UTC date validation or authentication of a continuously advancing
raw NMEA replay. HDT contains no measurement epoch; its honest provenance is
validated serial ingress in the current connection, independently bounded in age.
A receiver protocol carrying heading epochs would be needed for stronger proof.

## Common stopped-state gate

`cmd_vel_to_can` task mode (`task_control_required=true`) defaults
`raw_feedback_required=true`. It continues using EKF twist for near-zero velocity
and additionally requires fresh, finite, progressing raw odometry on
`raw_feedback_topic=/Odometry`, with `raw_feedback_frame_id=odom` and
`raw_feedback_child_frame_id=base_link`. A different profile must configure/remap
those consistently. The source stamp and monotonic receive age use the existing
`feedback_timeout_sec`. Invalid/missing raw evidence blocks readiness, stop
confirmation and reset even if the EKF continues publishing predicted zero.
FAST-LIO raw twist and `/vehicle/twist` are not used as measured vehicle velocity.
Legacy task-disabled navigation retains the optional raw gate default of false.

Cancellation must drain all owned pending/active goals and receive a new stopped
hold from source-fresh feedback. Old zero samples, cancel acknowledgement,
localization restoration and return from manual to automatic cannot restart a
terminated task. Held software emergency stop still requires its existing clear
inputs and explicit reset; reset does not authorize motion.

## Software validation boundary

Tests run only on an exclusive localhost DDS domain, with fake sensors/downstream
Nav2 actions, PTY serial acquisition and a captured `/can_tx`. They retain the
real provider, manager, coverage bridge and command conversion. No physical CAN
bridge, LiDAR, serial receiver or vehicle launch is started. Timeouts and thresholds
are configurable starting values; these tests do not establish physical stopping
performance, actual map registration or full vehicle acceptance. Those checks
remain open under #9.

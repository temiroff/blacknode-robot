# blacknode-robot

<video src="https://github.com/user-attachments/assets/80a9b797-ecf7-47d3-b6d3-baad7c0ea170" controls width="860"></video>

**Generic robot setup nodes for [Blacknode](https://github.com/temiroff/Blacknode).**

Install this Blacknode **extension package** to add robot hardware setup to the
visual workflow editor: find USB
serial robot devices, fix Linux serial permissions, launch and stop a driver
process, and emit reusable robot profiles for downstream control nodes —
drivable from workflows or AI agents over MCP.

This package owns the user-facing robot abstraction:

- find USB serial robot devices
- explain Linux serial permissions
- describe how a robot driver should start
- start/stop a driver process
- build, save, duplicate, load, and calibrate reusable robot profiles

Robot-specific packages provide protocol driver descriptors and hardware
bridges. Transport packages such as
`blacknode-ros2` verify and control the standard interface exposed by the
driver.

## Nodes

Coding agents should read [`AGENTS.md`](AGENTS.md) before changing this package.
It defines the package boundary, calibration identity contract, motion safety,
and verification commands.

| Node | What it does |
|---|---|
| `RobotDriverDescriptor` | Declares a driver command template and standard topics |
| `Robot` | Selects a robot, automatically finds its connection, applies calibration, and optionally starts its driver |
| `RobotJointDefinition` | Defines one named joint, servo ID, range, zero, and direction |
| `RobotJointList` | Combines any number of joint definitions; another socket appears as the list fills |
| `RobotDefinition` | Builds a reusable robot profile and driver contract visually |
| `RobotProfileSave` | Saves a profile under `robots/<profile_id>/profile.json` |
| `RobotProfileLoad` | Loads a profile and the calibration for a connected hardware ID |
| `RobotProfileList` | Lists built-in and locally saved profiles |
| `RobotProfileDuplicate` | Copies a built-in or local profile under a new editable name |
| `RobotCalibrationRecorder` | Safely records released-arm limits and a home pose for one physical robot |
| `RobotDriverLauncher` | Starts/stops a driver process from the descriptor |
| `RobotConnectionDashboard` | Shows USB, driver, ROS interface, live joint positions, home references, safe ranges, and calibration source in one view |

Changing the generic `Robot.profile_id` invalidates the old dashboard. Press
**Run** to apply it: if its generated driver command differs, Blacknode safely
stops the prior managed process before starting the selected profile. A
`PROFILE DEFAULTS` dashboard has no saved calibration for that profile and
hardware ID.

For one robot, use only `Robot`; it automatically discovers available hardware
and uses `selection: 0`. Duplicate it and choose `selection: 1` for a second
robot. If the discovered order is reversed, swap those two indexes. Camera and
future sensor facades use the same `selection` convention.
The selected entry's serial number (or port path when no serial is available)
becomes the robot's `hardware_id`; discovery's index-0 shortcut values never
override a different selected entry.

The Properties panel keeps transport controls under **Advanced**. They are not
required for normal setup: `probe_open` actively opens candidate serial ports
for diagnostics, vendor/product IDs narrow discovery to a USB adapter model,
and `hardware_filter` pins a node to one stable adapter identity. Stable
identity is recommended after calibration or for unattended motion so two
identical robots cannot silently exchange roles. Hidden compatibility nodes
still provide low-level diagnostics for old workflows, but USB is an
implementation of a robot connection rather than the public abstraction.

The **SO-ARM101 Leader Follower** template uses robot indexes `0` and `1`,
separate driver run IDs, and `/leader` and `/follower` ROS topic prefixes. It
releases only the leader, starts the follower controller in disarmed preview,
and requires saved calibration for both physical devices. For a permanent
installation, promote `hardware_filter` from Advanced and bind each role to its
adapter serial.
Its default configuration uses `tracking_mode=direct` at 60 Hz with no
deadband or relative step limiter. The Feetech driver batches
all joint reads and all goal writes into synchronized bus transactions, while
calibration limits, stale-stream suppression, and explicit arming still apply.

## Contract

The generic pipeline is:

```text
Robot -> discovered connection + calibrated profile + managed driver
```

Driver descriptors use `{serial_port}` as the placeholder for the discovered
USB path:

```text
python scripts/my_robot_driver.py --port {serial_port}
```

The resulting robot profile carries:

- `state_topic`
- `command_topic`
- optional `config_topic`
- `usb`
- `driver`
- `interface`

Robot-specific packages should fill in the descriptor. Transport packages should
verify and use the interface.

## Robot Selection and Drivers

Use the generic `Robot` node in new workflows. Its dropdown includes built-in
and locally saved profiles; it discovers the selected connection, applies the
matching calibration, and checks, starts, or stops the driver itself. The old
discovery and profile-loader types remain hidden for workflow compatibility.

`RobotDefinition.driver_script` is also a dropdown populated from installed
`drivers/*_driver.py` files when Blacknode starts. Adding a driver file and
restarting Blacknode makes it selectable without changing the node. A custom
executable or non-Python launch path can still use `protocol=custom` with an
explicit `command_template`.

The curated SO-ARM101 preset uses the same profile schema as a visual custom
robot. Supporting another arm on the bundled Feetech protocol normally means
assembling joint nodes and saving a profile; it does not require changing
Python. Only a genuinely new wire protocol needs another bus driver.

## Custom Robot Profiles

Open **Editable SO-ARM101 Profile** as a working example. Each
`RobotJointDefinition` names a stable joint and sets its servo ID, provisional
range, center tick, and direction. `RobotJointList` preserves their order;
`RobotDefinition` creates the profile; and `RobotProfileSave` makes it reusable.

To reuse the same mechanical definition under another identity, use
`RobotProfileDuplicate` with `source_profile_id=so_arm101` and choose a new ID.
For structural changes, copy the **Editable SO-ARM101 Profile** workflow and
edit its visible joint nodes before saving. Profile and joint IDs normalize to lowercase
`snake_case`, limited to 64 characters, and must be unique. Display names are
free-form and can change without breaking workflows.

Connect `Robot.hardware` to `RobotDefinition.hardware`. The definition copies
the real USB vendor ID and product ID reported internally; these four-digit
values identify the hardware manufacturer/product and are not random robot IDs.
Manual `vendor_id` and `product_id` values remain advanced overrides. The
adapter serial—or its device path when no serial exists—selects the calibration
for one physical assembly.

Local robot data is deliberately separate from the package source:

```text
robots/
  my_robot/
    profile.json
    calibrations/
      usb_serial_or_device_id.json
```

Set `BLACKNODE_ROBOTS_DIR` to move this library elsewhere. The default
`robots/` directory is ignored by Git because calibrations describe a specific
physical assembly. Copy or version it deliberately when sharing a machine
configuration.

### Guided Calibration

Open **Robot Guided Calibration** after saving a profile:

1. Load the profile and start discovery with the robot connected.
2. Press **Release + live pose** on Manual Move and physically support the arm.
3. Confirm live joint values are changing, then press **Start recording**.
4. Slowly move every joint through the safe physical range you intend to use.
   Do not force a hard stop.
5. Put the robot in the pose that should read as zero and press **Capture Home**.
6. Press **Stop recording** whenever you want to pause extrema collection
   without losing samples. Current pose remains live; press **Resume recording**
   to continue.
7. Press **Save calibration**. The recorder applies the configured safety
   margin inside the observed extrema and saves it under the hardware ID.
8. Press **Hold position** only while the arm is supported and the workspace is
   clear.

Recording never commands movement. It refuses to start while torque is on, and
it will not save until every configured joint has been observed and a home pose
has been captured. Its CURRENT pose, observed ranges, sample count, dashboard,
report, and connected Output nodes update through the live runtime. `Robot`
automatically applies the matching
device calibration when given the discovery hardware output; another physical
robot with the same profile keeps a separate calibration.

While recording, the most strongly moving joint is labeled **CAPTURING**. Its
row turns blue, and a newly extended limit flashes amber with `MIN ↓`, `MAX ↑`,
or `RANGE`. This distinguishes ordinary motion inside an already observed range
from a sample that actually changed the saved extrema.

### SO-ARM101 (`preset: so_arm101`)

Drives a real [SO-ARM101](https://github.com/TheRobotStudio/SO-ARM100) —
6x Feetech STS3215 serial-bus servos (`shoulder_pan`, `shoulder_lift`,
`elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`; servo IDs 1-6; 1 Mbps)
— through `drivers/feetech_bus_driver.py`. The preset defaults to
`transport=auto`: it uses native `rclpy` when available and otherwise uses
rosbridge. The transport can still be forced for advanced deployments. In
rosbridge mode the serial connection remains local to the driver machine.

```bash
pip install -r packages/blacknode-robot/requirements.txt   # servo SDK + roslibpy
```

1. Plug in the arm, then run `Robot` with `action=check` to confirm its
   connection is found and accessible. This confirms the adapter is enumerated,
   not that robot power or servo communication is healthy; use the driver
   connection state for that.
2. Load the **SO-ARM101 Motion Test** template
   (`templates/so-arm101-motion-test.json`): `Robot` (`so_arm101`, starts the driver) →
   `ROS2Status` → `ROS2JointState` → `ROS2SetJoint`
   (`armed=false` by default) → `ROS2MotionDashboard`.
3. Press **Run**. With `armed=false` this only proves the pipeline: the
   driver starts, `ROS2Status` selects native ROS 2 or rosbridge, and
   `ROS2JointState` shows the arm's live pose. **The arm must not move
   or twitch during this step** — see Safety below for why.
4. Set a `joint` name and `armed=true` on `ROS2SetJoint`, recook. It
   syncs to the current pose, ramps to the target, and the dashboard shows
   before/after.

### Manual Move + Live Pose

The motion-test template includes `ROS2ManualMove` between connection status and
motion. Its safe default is **Monitor only**, which changes no torque state and
starts an explicitly labeled live pose monitor.

- Press **Release + live pose** to disable servo torque while the driver keeps
  publishing joint positions. Support the arm before pressing it; it may go limp.
- Move the supported arm by hand. The Teach node's unconnected dashboard output
  refreshes from the runtime monitor and shows the latest pose.
- Press **Hold position** to hold again. The driver reads every servo and
  writes those exact positions as goals before enabling torque. Motion is
  blocked during the transition. The selected button and dashboard report the
  actual current mode rather than only the last requested action.
- **Stop all** remains the emergency-safe shutdown: it stops the driver and
  disables torque, so live state publishing also stops.

### Safety

- **Torque-enable sequencing.** Feetech STS servos snap toward whatever is
  already sitting in `Goal_Position` the instant `Torque_Enable` switches on
  — that register is not guaranteed to already equal the physical position.
  `feetech_bus_driver.py` always reads every servo's current position first,
  writes that same value back as `Goal_Position` while torque is still off,
  and only then enables torque — so there is nothing for the servo to snap
  toward. If the arm moves or twitches the moment the driver starts, stop and
  investigate before arming anything.
- **Shutdown behavior.** By default the driver disables torque on every
  servo when it stops (clean shutdown, crash, or `RobotDriverLauncher`
  `action=stop`) — the arm goes limp rather than holding its last position
  indefinitely with no watchdog. Override with
  `--no-torque-off-on-exit` in the preset's `command_template` (or a custom
  `RobotDriverDescriptor`) only if holding position is actually the safer
  failure mode for your specific mounting.
- **Rosbridge reconnects without restarting the hardware driver.** A dropped
  WebSocket no longer leaves an alive-but-silent process. The driver keeps the
  serial session open, waits for `roslibpy` to reconnect, then republishes its
  safety configuration and current joint pose before resuming state updates.
  Command writes and state reads share one bus lock so Feetech packet
  transactions cannot overlap. Confirmed command writes consume each servo's
  status response, and malformed short read packets retain the last valid pose
  instead of terminating the driver. Late driver exits retain their error text
  in runtime status instead of disappearing as a generic offline process.
- **The editor's "Stop all" reaches the driver process too.** `robot.py`
  exposes `runtime_status()`/`stop_runtime_services()` (registered in the
  main Blacknode editor's `_RUNTIME_MODULES`), so pressing "Stop all" sends
  `SIGTERM` to every driver this session launched — which is what actually
  triggers the torque-off-on-exit shutdown above. Before this was wired in,
  "Stop all" only stopped camera/tracking/reasoning stream helpers and left
  the robot driver (and torque) running silently in the background.
- **Joint limits are placeholders until calibrated.** The `min_deg`/`max_deg`
  values in the SO-ARM101 base profile are not a verified safe envelope for
  your assembly. Use **Robot Guided Calibration** with torque released to
  record intended physical ranges and a safety margin before commanding broad
  motion. Never discover limits by driving an armed joint into its hard stop.
- **Calibration details.** `feetech_bus_driver.py` initially assumes each servo's
  raw center tick (2048 of 4095) is that joint's mechanical zero and that
  none of the joints are mirror-mounted. Saved profiles carry direction and
  center information, while device calibrations supply measured home ticks and
  safe ranges automatically. The `--home-ticks` and `--invert` CLI flags remain
  available as advanced driver-level overrides.
- **Verify the control-table addresses before ever writing.** Run
  `python packages/blacknode-robot/drivers/feetech_bus_driver.py --dry-run
  --port <serial_port> --joints "<name:id:min:max,...>"` first: it only reads
  `Present_Position` for every servo ID and prints the result — it never
  touches `Goal_Position` or `Torque_Enable`. Confirm every servo responds
  with a plausible tick (0-4095) before trusting the driver with real writes.

Keep a physical power cutoff within reach and clear the workspace before arming.

## License

Apache-2.0, same as Blacknode.

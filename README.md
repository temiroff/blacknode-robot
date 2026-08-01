# blacknode-robot

`blacknode-robot/core` owns stable robot contracts, profiles, discovery,
hardware-identity calibration, capability bindings, and process descriptors.
`blacknode-robot/devices` owns connected hardware representation,
configuration, discovery, lifecycle, and health.
`blacknode-robot/telemetry` owns normalized temperatures, voltage, faults,
joint state, and device status.
Physical driver implementations are selectable `blacknode-drivers` components.

```text
Feetech driver
    -> robot device registry
    -> normalized robot telemetry
    -> UI, diagnostics, and safety
```

Serial, CAN, USB, register maps, and vendor SDK communication stay inside
`blacknode-drivers`. Device and telemetry consumers depend on normalized robot
contracts rather than physical transports.

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
- bind semantic capabilities to replaceable package components
- inspect each provider as available, unavailable, or unhealthy
- represent connected devices, configuration, lifecycle, and health
- publish normalized temperatures, voltage, faults, and device status

The device-service operator guide is in [`docs/devices.md`](docs/devices.md).

## Canonical robot state

`blacknode-robot/contracts` owns the transport-neutral state model:

```text
Driver-specific feedback
    -> JointState / DeviceState / FaultState
        -> workflows
        -> telemetry
        -> motion safety
        -> ROS 2 and UI adapters
```

Canonical joint position and velocity use radians and radians per second.
ROS `sensor_msgs/msg/JointState`, vendor registers, MQTT payloads, and editor
monitor records are adapter representations of this model.

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
| `ComputeDevice` | Selects a registered computer by stable ID and exposes current credential-free state from its paired Runtime |
| `DeviceInspect` | Splits current device state into environment, ROS 2 graph, capability candidates, unclassified interfaces, and complete inventory |
| `RobotDriverDescriptor` | Declares a driver command template and standard topics |
| `Robot` | Selects a robot, automatically finds its connection, applies calibration, and optionally starts its driver |
| `RobotJointDefinition` | Defines one named joint, servo ID, range, zero, and direction |
| `RobotJointList` | Combines any number of joint definitions; another socket appears as the list fills |
| `RobotDefinition` | Builds a reusable robot profile and driver contract visually |
| `RobotProfileSave` | Saves a profile under `robots/<profile_id>/profile.json` |
| `RobotProfileLoad` | Loads a profile and the calibration for a connected hardware ID |
| `RobotProfileList` | Lists built-in and locally saved profiles |
| `RobotProfileDuplicate` | Copies a built-in or local profile under a new editable name |
| `RobotCalibrationControl` | Selects a profile-bound hardware provider, safely releases or holds the robot, and streams normalized live joint feedback |
| `RobotCalibrationMockProvider` | Hidden hardware-free implementation of the calibration-control provider contract for development and contract testing |
| `RobotCalibrationRecorder` | Safely records released-arm limits and a home pose for one physical robot |
| `RobotCapabilityBinding` | Binds one semantic capability to a replaceable package component, adapter, configuration, and optional hardware identity |
| `RobotAttachment` | Describes one mounted camera, depth camera, LiDAR, IMU, GPS, microphone, or custom peripheral with its ROS 2 interface, frame, transform, provider, and hardware identity |
| `RobotAttachmentList` | Collects physical attachments for storage in a reusable robot profile |
| `RobotCapabilityList` | Collects capability bindings for a robot profile |
| `RobotCapabilityProfile` | Attaches capability providers and stable hardware identity to a reusable robot profile |
| `RobotCapabilityInspect` | Resolves a profile against installed components and live provider reports as available, unavailable, or unhealthy |
| `RobotDriverLauncher` | Starts/stops a driver process from the descriptor |
| `RobotConnectionDashboard` | Shows USB, driver, ROS interface, live joint positions, home references, safe ranges, and calibration source in one view |
| `RobotMonitor` | Opens a read-only live canvas view for a registered robot's connection, motion state, profile and calibration identity, joint coverage, telemetry, streams, and joints |
| `RobotServo` | Represents one servo connected to a Robot monitor target, with live position, velocity, raw ticks, calibrated limits, torque, temperature, voltage, faults, and explicitly armed slider control |
| `RobotROSCapabilityDiscover` | Infers generic camera, depth camera, LiDAR, IMU, GPS, battery, joint-state, and mobile-base candidates from standard live ROS 2 message types without binding or commanding hardware |
| `RobotROSInterfaceCheck` | Matches a live ROS graph to a supported robot interface profile without publishing commands |

## Live compute-device state

Open **Inspect a Compute Device**, choose a registered computer on the
`ComputeDevice` node, and press **Run once**. The editor reads current state
through the authenticated paired Runtime immediately before the cook.
`DeviceInspect` exposes the live ROS 2 graph, generic capability candidates,
unclassified interfaces, and complete inventory through typed outputs.

A remote computer can be registered before Blacknode Runtime is installed.
Use **Devices → Add device → Remote SSH**, confirm the host fingerprint, and
press **Confirm and inspect**. Install or pair the Runtime to enable live graph
reads. The workflow saves the stable device ID and display identity. The SSH
password is discarded after setup and is not required for routine live reads.
The graph never stores a password, pairing token, or arbitrary shell command.

This live discovery path is read-only. Continuous streams and physical control use a
paired Runtime or another managed provider with freshness, shutdown, limits,
and explicit arming safeguards.

## Generic ROS capability discovery

Open the **Discover ROS Robot Capabilities** template and press **Run once**
while the robot's existing ROS 2 bringup is active. The template checks the ROS
runtime, inventories the graph, and displays capability candidates,
unclassified topics, and the complete read-only inventory.

Connect the typed output of `ROS2TopicList` to
`RobotROSCapabilityDiscover.topics`. Connect `ROS2NodeList` and
`ROS2ServiceList` when the complete graph inventory is useful. Discovery uses
standard ROS message types as its primary evidence and returns versioned
capability candidates with confidence, state topics, command topics, and the
evidence behind each result.

Discovery is read-only. Every candidate requires confirmation before it becomes
a provider binding, and a command-only topic is never reported as measured
hardware state. Topics that cannot be classified remain in `unclassified` for
interface inspection or a future reusable provider.

## Servo debugging

Open **Servo Debug Monitor**, select a registered robot on the Robot Monitor
node, and connect any number of `RobotServo` nodes to its `robot` output. Each
Servo node selects an ID independently while all cards share one live telemetry
connection to the robot. Robot Monitor combines that stream with authenticated
device status so the active profile, calibration, hardware identity, and
reported-versus-expected joint coverage stay visible while Robot Hardware or a
deployment owns the bus.

Robot Monitor also lists compatible serial robots attached directly to the
editor computer under **Local USB**. Blacknode rediscovers the opaque target,
matches its saved profile and calibration, and opens its bound provider for
read-only monitoring. The selector does not accept arbitrary device paths, and
one shared stream supplies the monitor and every connected Servo card.

The Robot Monitor and direct Servo cards expose the same local profile picker:
**Auto** uses the strongest hardware-bound match, a saved profile can be chosen
explicitly, and **None · raw read-only** asks an installed driver provider to
discover responding servo IDs. Raw mode shows register ticks and passive
diagnostics under generic names such as `servo_2`; it does not infer semantic
joint names, calibrated angles, limits, direction, or robot topology. Target
preview, calibration, torque changes, and motion remain unavailable until a
profile is selected. Raw discovery scans IDs 1–32 by default and labels this
bounded range in its report.

For local USB hardware, choose the robot and its calibrated profile directly on
the Servo node. The slider begins as a preview. Press **Arm** on that Servo node
to synchronize every joint to its current position and enable live control of
the selected joint; press **Disarm** to release torque. The generic motion
gateway retains arbitration, calibrated limits, fresh-feedback checks,
hardware-warning checks, and exclusive ownership while the profile-selected
provider handles the bus.

The canonical `command` output remains available for advanced workflows that
route Servo targets through another compatible motion controller.

When a calibration is active, the card displays its safe range and marks the
limits as calibrated. Robot Monitor and every Servo card show response state,
raw position, temperature, per-servo voltage, status flags, decoded hardware
warnings, and bus timeout and packet-error counters when the selected provider
reports them. A responding servo carrying a hardware warning remains visible
as **Warning** rather than being labeled missing. Unavailable measurements are
labeled **Not reported** rather than synthesized. Voltage warnings also prompt
the operator to verify that the connected supply matches the robot and servo
voltage rating before enabling torque.

## Complete robot bringup

Open **Complete Robot Bringup** and press **Run**. Its `Robot` node discovers
the connected hardware, selects the profile already bound to that physical
device, applies its calibration, and starts the driver with motion disarmed.
The remaining nodes inventory ROS 2 and report each declared capability as
available, unavailable, or unhealthy.

Automatic profile selection prefers a calibration saved for the connected
hardware, then an exact physical identity, then an exact USB match. When only
one profile exists, Blacknode selects it directly. If multiple profiles remain
equally possible, the report asks for one profile selection instead of choosing
a motion-capable definition by guesswork.

Joint-based profiles automatically expose `position_feedback` and
`joint_group` capability bindings using their configured state and command
topics. Other robot shapes declare only their own capabilities and ROS 2
interfaces. An arm does not imply a base, and a camera does not imply LiDAR.
The bringup workflow inventories and checks the declared interfaces; it never
authorizes or sends motion.

Changing the generic `Robot.profile_id` invalidates the old dashboard. Press
**Run** to apply it: if its generated driver command differs, Blacknode safely
stops the prior managed process before starting the selected profile. A
`PROFILE DEFAULTS` dashboard has no saved calibration for that profile and
hardware ID.

The Robot node shows live **Profile** and **Calibration** dropdowns on the
canvas. The Profile picker lists built-in and saved profiles rather than the
internal automatic-matching mode. Saving a profile refreshes these choices
while the editor is running; opening either dropdown also rescans
`robots/*/profile.json`. Selecting a different profile clears any calibration
belonging to the previous profile so deployment cannot silently reuse the wrong
physical identity.

For one robot, use only `Robot`; the default `profile_id=auto` discovers the
available hardware and uses `selection: 0`. Duplicate it and choose
`selection: 1` for a second robot. Bind each physical device to its saved
profile or calibration so selection remains stable when USB enumeration order
changes. Camera and future sensor facades use the same `selection` convention.
The selected entry's serial number (or port path when no serial is available)
becomes the robot's `hardware_id`; discovery's index-0 shortcut values never
override a different selected entry.

For remote deployment, declare the device capabilities and choose calibration
in the editor's **Deployments** panel. **Activate on device** binds the saved
calibration to the paired device while it is connected and disarmed. Staging
then embeds that exact robot profile and calibration in the workflow. The
remote `Robot` node applies the embedded home positions and safe ranges only
when discovery returns the same physical hardware identity.

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
- `attachments`, including stable attachment IDs, provider bindings, ROS 2 topics and message types, TF parent/child frames, mount transforms, and physical hardware identity

Robot-specific packages should fill in the descriptor. Transport packages should
verify and use the interface.

## Robot Selection and Drivers

Use the generic `Robot` node in new workflows. Its dropdown starts with
`Auto`, followed by built-in and locally saved profiles. It discovers the
selected connection, applies the matching calibration, and checks, starts, or
stops the driver itself. The old discovery and profile-loader types remain
hidden for workflow compatibility.

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
The profile-only template does not select a live `Robot`: add one and connect
its `hardware` output only when the profile should copy USB identity from a
currently attached device.

To reuse the same mechanical definition under another identity, use
`RobotProfileDuplicate` with `source_profile_id=so_arm101` and choose a new ID.
For structural changes, copy the **Editable SO-ARM101 Profile** workflow and
edit its visible joint nodes before saving. Profile and joint IDs normalize to lowercase
`snake_case`, limited to 64 characters, and must be unique. Display names are
free-form and can change without breaking workflows.

Open **Robot Sensor Attachments** to describe a camera, LiDAR, and IMU mounted
on one robot. Each `RobotAttachment` stores its physical identity, replaceable
provider, ROS 2 topic and message type, TF parent/child frames, and translation
plus roll/pitch/yaw mount transform. Add more attachment sockets through
`RobotAttachmentList`, then save the resulting profile. Edit the example topic
and frame names to match the live graph before checking or starting providers.

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

1. Enter a clear **Calibration name**, such as `Workshop arm` or
   `Left SO-ARM101`.
2. Load the profile and start discovery with the robot connected.
3. Press **Release + live pose** on Robot Calibration Control and physically
   support the robot. The node resolves the provider bound by the selected
   profile and opens its normalized calibration session.
4. Confirm live joint values are changing, then press **Start recording**.
5. Slowly move every joint through the safe physical range you intend to use.
   Do not force a hard stop.
6. Put the robot in the pose that should read as zero and press **Capture Home**.
7. Press **Stop recording** whenever you want to pause extrema collection
   without losing samples. Current pose remains live; press **Resume recording**
   to continue.
8. Press **Save calibration**. The recorder applies the configured safety
   margin inside the observed extrema and saves the name with its hardware ID.
9. Press **Hold position** only while the arm is supported and the workspace is
   clear. The controller reads and seeds every current joint position before
   enabling holding torque.

Recording never commands movement. It refuses to start while torque is on, and
it will not save until every configured joint has been observed and a home pose
has been captured. Its CURRENT pose, observed ranges, sample count, dashboard,
report, and connected Output nodes update through the live runtime. `Robot`
automatically applies the matching
device calibration when given the discovery hardware output; another physical
robot with the same profile keeps a separate calibration.

The calibration dashboard lists every joint from the selected profile
immediately. Unsampled joints remain visible as **not observed**, and the
resolved physical hardware ID stays in the dashboard header.

The workflow contains no vendor or transport node. A profile binds
`calibration_control` to a package component, and that component supplies the
hardware-specific session. Missing providers report an unavailable state while
profile discovery and calibration files remain usable.

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

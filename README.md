# blacknode-robot

**Generic robot setup nodes for [Blacknode](https://github.com/temiroff/Blacknode).**

This is a Blacknode **extension package** — it does not run on its own. It
plugs robot hardware setup into the Blacknode visual workflow editor: find USB
serial robot devices, fix Linux serial permissions, launch and stop a driver
process, and emit one standard robot profile for downstream control nodes —
drivable from workflows or AI agents over MCP.

This package owns the user-facing robot abstraction:

- find USB serial robot devices
- explain Linux serial permissions
- describe how a robot driver should start
- start/stop a driver process
- emit one standard robot profile for downstream control nodes

It intentionally does not know every robot protocol. Robot-specific packages
provide driver descriptors and hardware bridges. Transport packages such as
`blacknode-ros2` verify and control the standard interface exposed by the
driver.

## Nodes

| Node | What it does |
|---|---|
| `RobotUSBDiscovery` | Finds `/dev/serial/by-id/*`, `/dev/ttyACM*`, `/dev/ttyUSB*` and reports access fixes |
| `RobotDriverDescriptor` | Declares a driver command template and standard topics |
| `RobotDriverPreset` | Fills in a driver descriptor for a known, tested robot (currently: SO-ARM101) |
| `RobotDriverLauncher` | Starts/stops a driver process from the descriptor |
| `RobotDiscovery` | Runs the generic setup path and outputs a robot profile |

## Contract

The generic pipeline is:

```text
USB device -> driver descriptor -> driver process -> standard robot profile
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

## Presets

`RobotDriverPreset` is a curated shortcut ahead of `RobotDriverDescriptor`:
pick a known, tested robot from its `preset` dropdown and it fills in a
ready-to-launch `driver` dict (same shape `RobotDriverDescriptor` produces),
including a `command_template` that points at a bundled driver script in
`drivers/`. Wire its `driver` output straight into `RobotDiscovery` (or
`RobotDriverLauncher`) — no manual command template required.

Supporting a new robot is additive: one more entry in the preset table
(`nodes/presets.py`), and — only if it's a new wire protocol, not just a new
arm — one more `<protocol>_bus_driver.py` script in `drivers/`. Everything
else (USB discovery, launch/stop, and the ROS 2 `JointState` contract) is
already generic.

### SO-ARM101 (`preset: so_arm101`)

Drives a real [SO-ARM101](https://github.com/TheRobotStudio/SO-ARM100) —
6x Feetech STS3215 serial-bus servos (`shoulder_pan`, `shoulder_lift`,
`elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`; servo IDs 1-6; 1 Mbps)
— through `drivers/feetech_bus_driver.py`. The preset defaults to native
`rclpy` for Linux; set `transport=rosbridge` for Windows. In rosbridge mode the
same driver publishes state and receives commands through `roslibpy`, while the
serial connection remains local to Windows.

```bash
pip install -r packages/blacknode-robot/requirements.txt   # servo SDK + roslibpy
```

1. Plug in the arm, then run `RobotUSBDiscovery` to confirm its USB-serial
   adapter is found and accessible (fix any permission issue it reports
   first).
2. Load the **SO-ARM101 Motion Test** template
   (`templates/so-arm101-motion-test.json`): `RobotDriverPreset` (preset
   `so_arm101`) → `RobotDiscovery` (starts the driver process) →
   `ROS2NativeStatus` → `ROS2NativeJointState` → `ROS2NativeSetJoint`
   (`armed=false` by default) → `ROS2MotionDashboard`.
3. Press **Run**. With `armed=false` this only proves the pipeline: the
   driver starts, `ROS2NativeStatus` reports ready, and
   `ROS2NativeJointState` shows the arm's live pose. **The arm must not move
   or twitch during this step** — see Safety below for why.
4. Set a `joint` name and `armed=true` on `ROS2NativeSetJoint`, recook. It
   syncs to the current pose, ramps to the target, and the dashboard shows
   before/after.

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
- **The editor's "Stop all" reaches the driver process too.** `robot.py`
  exposes `runtime_status()`/`stop_runtime_services()` (registered in the
  main Blacknode editor's `_RUNTIME_MODULES`), so pressing "Stop all" sends
  `SIGTERM` to every driver this session launched — which is what actually
  triggers the torque-off-on-exit shutdown above. Before this was wired in,
  "Stop all" only stopped camera/tracking/reasoning stream helpers and left
  the robot driver (and torque) running silently in the background.
- **Joint limits are placeholders.** The `min_deg`/`max_deg` sweep baked into
  the `so_arm101` preset (`nodes/presets.py`) is a starting range, not a
  verified safe envelope for your specific arm. Confirm it physically —
  slowly, with `armed=true` and small deltas — before trusting it, and narrow
  it in `nodes/presets.py` if needed.
- **Calibration escape hatch.** `feetech_bus_driver.py` assumes each servo's
  raw center tick (2048 of 4095) is that joint's mechanical zero and that
  none of the joints are mirror-mounted. Real arms often aren't that clean
  after assembly. Use `--home-ticks "name:ticks,..."` to override a joint's
  true zero and `--invert "name,name"` to flip a joint's sign — both are
  plumbed through `command_template` if you need to customize a preset.
- **Verify the control-table addresses before ever writing.** Run
  `python packages/blacknode-robot/drivers/feetech_bus_driver.py --dry-run
  --port <serial_port> --joints "<name:id:min:max,...>"` first: it only reads
  `Present_Position` for every servo ID and prints the result — it never
  touches `Goal_Position` or `Torque_Enable`. Confirm every servo responds
  with a plausible tick (0-4095) before trusting the driver with real writes.

Keep a physical power cutoff within reach and clear the workspace before arming.

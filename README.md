# blacknode-robot

`blacknode-robot` owns transport-neutral robot contracts, connected-device state, profiles, hardware-bound calibration, capability bindings, normalized telemetry, and driver launch descriptors.

Physical bus communication lives in `blacknode-drivers`; motion control lives in `blacknode-motion`; ROS transport lives in `blacknode-ros2`.

## Components

| Component | Purpose |
|---|---|
| `core` / `contracts` | Robot, joint, device, and fault contracts |
| `profiles` / `models` | Reusable profiles, driver descriptors, presets, and launch models |
| `calibration` | Safe calibration capture tied to stable hardware identity |
| `capabilities` | Discovery, attachments, provider bindings, and readiness inspection |
| `devices` | Connected hardware lifecycle, health, and safe provider interfaces |
| `telemetry` | Normalized joint, voltage, temperature, fault, and status data |
| `authorization` | Optional consequential-action gating |

## Operator flow

Use the Blacknode editor as the primary surface:

1. Open **Devices** to add or inspect a computer.
2. Open **Packages** and enable the required robot and driver components.
3. Load `complete-robot-bringup.json` to discover hardware, select a profile, apply its matching calibration, and start the driver disarmed. Each Robot node keeps its own calibration hardware selection and uses that stable identity to select the connected device, so leader/follower workflows can bind two arms independently.
4. Use Robot Monitor for read-only state and `RobotServo` for preview; arm only after identity, calibration, limits, and fresh feedback are correct.
5. Use the guided calibration and editable-profile templates when defining a new physical assembly.

Core nodes include `Robot`, `ComputeDevice`, `PhysicalRobot`, `RobotDeployment`, `RobotStream`, `DeviceInspect`, profile load/save/duplicate nodes, calibration control/recording, capability and attachment nodes, `RobotMonitor`, and `RobotServo`. Build device workflows as `ComputeDevice` → `PhysicalRobot` → `RobotDeployment` → `RobotStream`. Each node owns one selection and passes credential-free live inspection state forward. Connect `RobotStream.topic` and `RobotStream.message_type` to generic ROS 2 stream inputs, then route the ROS message into a map, camera, LiDAR, IMU, or other compatible viewer or processing node. Local profiles and hardware-bound calibration copies live outside package source under `~/.blacknode/robots/` by default, or under `BLACKNODE_ROBOTS_DIR` when configured. Releases before 0.5.6 stored them under the process working directory's `robots/` folder; that location remains readable for compatibility.

The `Robot` calibration input also accepts Blacknode's explicit calibration-import envelope. SO-ARM motor range JSON selected through an operator app is validated against the profile, bound to the discovered USB serial, converted to Blacknode's native degree-and-safety format, and copied into the persistent Blacknode directory. Runtime use has no dependency on the tool that originally produced the JSON.

## Actuator Setup workflow

Press the first shelf button, **Actuator Setup**, select a USB port and press
**Scan**. A separate connected node appears for each responding servo, showing its
ID, model, raw position, voltage, temperature, torque, hardware warnings and
read-only hardware settings. Rescanning updates existing cards and flags missing
IDs. Choose another baud rate under **Advanced** if no actuator responds. The scan
covers IDs 0–253 in one bounded operation. Discovery needs no robot profile.

For assignment, support the arm, power off, connect only the new actuator and
scan again after powering on. Enter **New ID** (1–253) on that servo's card.
Keep that actuator supported and press **Set ID**. The button performs a fresh
scan, programs the ID and verifies the result. The ID controls have no confirmation
checkboxes or manual scan-expiry step; changing the ID retires saved calibration.
Legacy clients retain their expiring, single-use confirmation token contract.
Programming requires torque off and a supported, warning-free actuator.
If torque is on or unknown, confirm the isolated actuator is supported against
gravity and use **Release torque**, then scan again.

Saved calibrations for this USB hardware identity are preserved under each
profile's `calibrations/retired/` directory before an assignment attempt. They
are excluded from automatic calibration selection, including after an uncertain
write result. Power off and reconnect the complete arm, rescan, and resolve
missing IDs. Expand **Calibration and motion test**, select a robot profile whose
joint IDs match the finished assembly, and press **Calibrate**. Record new
hand-guided calibration, then **Open motion test** for the selected servo's
explicitly armed testing within calibrated limits.

The node's ordinary cook is inert. Operator controls delegate through the
USB-matched `_bn_robot_actuator_setup_provider` contract: `scan(config)`
returns `actuators`; `assign(config, expected, new_id)` verifies the physical
change and returns `assigned`, `old_id`, `new_id`, `actuators` and `report`.
`release(config, expected)` independently verifies torque off and returns
`released`, `actuators` and `report`.
The calibration panel uses optional `read_position(config, servo_id)` feedback
to follow hand movement in raw ticks before arming. Each capture reads fresh
feedback; incomplete points remain drafts across reloads. **Arm** validates and
saves a complete range, with **Save states** also available separately.
Protocol handling stays in `blacknode-drivers`. A mock provider supplies the
same contract for hardware-free development. Current physical ID programming
supports STS3215 through a Local USB adapter; managed-device setup is not exposed.

Each card also captures released **Min / Max** endpoints, optional **Home**, and
named measured poses. Either endpoint tick direction is accepted; an interior
Home or a calculated midpoint supplies the test origin while original captures
are preserved. **Arm** checks and saves the range before starting the
test slider, with exact captured endpoint limits and direct position targets. Capable
providers enforce finite speed and acceleration in their position controller. The
managed test releases torque on lost controls, invalid feedback or Stop. Saved
points and poses follow the physical USB identity, provider and servo ID and are
retired on ID changes. Whole-robot calibration remains separately available.

## Safety

- Motion is disarmed by default.
- Calibration is recorded only while torque is released and the robot is physically supported.
- Calibrations bind to stable hardware identity and are never substituted silently.
- Freshness, calibrated limits, hardware warnings, and shutdown behavior remain enforced at provider and driver boundaries.
- Managed hardware drivers receive an owner watchdog and a graceful-stop signal. On Windows they remain isolated from console-close termination long enough to release torque, and Stop All keeps their transport available until torque-off is confirmed.
- Device pairing tokens never belong in workflows, logs, process arguments, or tracked files.

## Verification

```powershell
python -m pytest packages/blacknode-robot/tests
Get-ChildItem packages\blacknode-robot\templates\*.json | ForEach-Object { blacknode validate $_.FullName }
```

Device-service operations are documented in [docs/devices.md](docs/devices.md). See [AGENTS.md](AGENTS.md) for calibration and motion safeguards.

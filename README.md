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
3. Load `complete-robot-bringup.json` to discover hardware, select a profile, apply its matching calibration, and start the driver disarmed.
4. Use Robot Monitor for read-only state and `RobotServo` for preview; arm only after identity, calibration, limits, and fresh feedback are correct.
5. Use the guided calibration and editable-profile templates when defining a new physical assembly.

Core nodes include `Robot`, `ComputeDevice`, `DeviceInspect`, profile load/save/duplicate nodes, calibration control/recording, capability and attachment nodes, `RobotMonitor`, and `RobotServo`. Local profiles and calibration data live outside package source under `robots/` and are intentionally ignored by Git.

## Safety

- Motion is disarmed by default.
- Calibration is recorded only while torque is released and the robot is physically supported.
- Calibrations bind to stable hardware identity and are never substituted silently.
- Freshness, calibrated limits, hardware warnings, and shutdown behavior remain enforced at provider and driver boundaries.
- Device pairing tokens never belong in workflows, logs, process arguments, or tracked files.

## Verification

```powershell
python -m pytest packages/blacknode-robot/tests
Get-ChildItem packages\blacknode-robot\templates\*.json | ForEach-Object { blacknode validate $_.FullName }
```

Device-service operations are documented in [docs/devices.md](docs/devices.md). See [AGENTS.md](AGENTS.md) for calibration and motion safeguards.

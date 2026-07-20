# blacknode-robot Agent Instructions

This is an independent extension-package repository. Check and commit its Git
state separately from the Blacknode core checkout that may contain it.

## Scope

Keep generic USB discovery, permission checks, driver descriptors and launch,
robot profiles, hardware-bound calibration, and capability bindings here.
Keep physical drivers in `blacknode-drivers`, generic controllers in
`blacknode-controllers`, and ROS transport/control nodes in `blacknode-ros2`.

## Safety rules

- Keep all motion disarmed by default and preserve an explicit preview path.
- Before enabling torque, read every joint and write its current position as the
  goal. Any startup twitch is a failure that must be investigated.
- Default shutdown must stop launched drivers and disable torque. Warn that the
  arm may fall under gravity; never weaken this behavior silently.
- Keep physical assembly definitions separate from calibrations. Bind each
  calibration to stable hardware identity and never substitute a generic one.
- Record calibration only while torque is released. Never discover limits by
  driving an armed joint into a hard stop.
- Enforce freshness and calibrated limits at the driver boundary, even if an
  upstream controller also checks them.
- Serialize bus transactions and make repeated idempotent control messages safe.
- Retain actionable driver errors in runtime status after process exit.

## Development rules

- Preserve generic profile and driver contracts; keep model-specific behavior
  behind descriptors/drivers.
- Guard optional SDK and rosbridge imports so package discovery still works.
- Update `blacknode-package.toml`, templates, tests, and README with new public
  nodes or dependencies.
- Mark templates with every package they require and keep safe defaults visible.

## Verification

From the Blacknode root:

```powershell
python -m pytest packages/blacknode-robot/tests
Get-ChildItem packages\blacknode-robot\templates\*.json | ForEach-Object { blacknode validate $_.FullName }
```

Use mocks for routine tests. Report physical calibration, torque, or motion
paths as untested unless they were deliberately exercised on supported hardware.

## Documentation voice

Describe Blacknode hardware discovery, profiles, calibration, drivers, and safe
operation directly. Mention external names only for implemented hardware or
protocol contracts; avoid product comparisons.

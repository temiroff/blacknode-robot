# Blacknode robot devices and telemetry

The `blacknode-robot/devices` and `blacknode-robot/telemetry` components provide
connected-device contracts, safe lifecycle management, and normalized status
streams.

For compatibility with existing deployed robots, the service unit,
configuration directory, and environment variables retain their established
`blacknode-hardware` names during this package migration.

This package provides the stable boundary between Blacknode workflows and
physical devices. Workflows use capabilities such as `mobile_base`, `camera`,
`range_sensor`, `battery`, and `servo_bus`; replaceable adapters implement
those capabilities for a particular transport or device.

The initial foundation is intentionally hardware-independent:

- capability and device-state contracts
- disarmed-by-default motion authorization
- command freshness and velocity-limit validation
- a configurable I2C mecanum-base adapter
- a generic joint-group contract for real actuator adapters
- an editor node for capability inspection

The next implementation layer can add adapters for USB, serial, I²C, GPIO,
CAN, ROS 2, or a network hardware service without changing workflow names.

## Install

From the Blacknode repository root:

```powershell
blacknode packages install .\packages\blacknode-robot
```

Or install the package checkout directly during development:

```powershell
pip install -e .\packages\blacknode-robot
```

No device SDK is required for package discovery or contract inspection.

For a guided Ubuntu setup:

```bash
./setup_ubuntu.sh
./check.sh --software-only
```

The setup script installs packages and creates `.venv`. It reports the boot
configuration needed for I2C but does not silently rewrite system files.

Discover connected serial devices:

```bash
./discover.sh
```

For any number of connected serial robots, use the automatic setup:

```bash
./configure.sh --all --install
```

Blacknode deduplicates `/dev/serial/by-id/...` and `/dev/tty...` aliases,
checks every distinct serial adapter with position reads, and keeps only buses
where servos respond. It detects servo IDs 1 through 20 by default. Increase
the read-only scan range when a robot uses higher IDs:

```bash
./configure.sh --all --install --servos 40
```

The command shows each stable serial path and detected servo IDs, then asks for
a friendly name such as `Left arm` or `Packing arm`. Names remain attached to
the same serial hardware on later scans. For unattended setup, provide names in
discovery order:

```bash
./configure.sh --all --install \
  --name "Left arm" \
  --name "Packing arm" \
  --no-prompt
```

Each robot receives its own stable device identity, configuration directory,
pairing token, active-calibration file, systemd service, and available HTTP
port starting at `8765`. Port `8766` remains reserved for the Blacknode runtime,
so the first robot services use `8765`, `8767`, `8768`, and subsequent ports.
The final pairing checklist prints each friendly name beside its URL
and private token so the robots can be added to the Blacknode editor one at a
time. The discovery probe does not enable torque, write goal positions, or move
a robot.

Inspect or operate every configured robot later:

```bash
./configure.sh --all --show
./pair.sh --all --show
./service.sh --all status
./service.sh --all restart
./service.sh --all check
```

Single-robot configuration remains available:

```bash
./configure.sh \
  --name "Left arm" \
  --servos 6 \
  --port /dev/serial/by-id/DEVICE_PATH
./pair.sh
./install-service.sh
```

Perform the first real hardware check with a read-only actuator probe:

```bash
./probe.sh --servos 6
```

`--servos 6` scans serial IDs 1 through 6. Without `--servos`, the probe
scans IDs 1 through 20. It reads present positions only and does not enable
torque, write goal positions, or move the arm. Override the connection
defaults when required:

```bash
BLACKNODE_SERIAL_PORT=/dev/ttyACM0 \
BLACKNODE_SERIAL_BAUDRATE=1000000 \
./probe.sh --servos 6
```

## Configure the device

Save the detected serial hardware for the device service:

```bash
./configure.sh --servos 6
```

The configuration is saved atomically to:

```text
.blacknode-hardware/device.json
```

This directory is ignored by Git. Running `configure.sh` again updates the
same configuration, so the device can be reconfigured at any time:

```bash
./configure.sh --show
./configure.sh --servos 7
./configure.sh --port auto
./configure.sh --port /dev/serial/by-id/DEVICE_PATH --baudrate 1000000
./configure.sh --device-id arm-01
```

Settings not included in a reconfiguration command are preserved. Configuration
and status monitoring are read-only and never enable torque or send goal
positions. The serial monitor reads each servo's physical torque-enable register
and reports torque as enabled if any configured servo is holding. Mixed servo
states are reported as a warning. If any register cannot be read, status reports
the physical torque state as unknown and identifies the affected joint and servo.
If the persistent service is already installed, apply configuration changes with:

```bash
./service.sh restart
```

The authenticated service also exposes an explicit torque-release safety
operation. It only writes `0` to each configured servo's torque-enable register;
it never sends positions or enables motion. The editor presents this as
**Release torque** when physical torque is on or unknown and no deployment is
running. It distinguishes verified-off results from successful torque-off writes
whose register readback remains unavailable. Support the robot before using it
because joints may move or drop.

## Stream telemetry with MQTT

Robot Hardware has a transport-neutral telemetry bus. It emits versioned,
sequenced, timestamped `robot-state` envelopes and keeps the physical adapter
independent from the selected transport. Joint positions are reported in
degrees; joint velocities are derived in degrees per second when the provider
does not report velocity directly. Future battery, camera metadata, temperature,
current, and other capability streams use the same envelope.

MQTT is an optional telemetry adapter. Install it in the Hardware environment:

```bash
./.venv/bin/python -m pip install -e '.[mqtt]'
```

For a package-managed checkout, enable the optional adapter and apply its
dependencies:

```bash
blacknode packages enable blacknode-robot telemetry --adapter mqtt
blacknode packages setup blacknode-robot
```

Copy `mqtt.example.env` beside the robot's `device.json` as `mqtt.env`.
Persistent systemd units read that file automatically when it exists:

```bash
config_dir="$PWD/.blacknode-hardware"
cp mqtt.example.env "$config_dir/mqtt.env"
install -m 0600 /dev/null "$config_dir/mqtt.password"
# Write the broker password into mqtt.password, then edit mqtt.env.
./service.sh restart
```

For multi-robot installations, place one `mqtt.env` beside each generated
`device.json`. Each robot can use a distinct broker account and topic prefix.
The service publishes:

```text
{topic-prefix}/{url-escaped-device-id}/telemetry/robot-state
```

Each JSON message contains:

```json
{
  "kind": "blacknode.telemetry",
  "schema_version": 1,
  "device_id": "robot-01",
  "stream": "robot-state",
  "sequence": 42,
  "source_time": 1785052800.125,
  "receive_time": 1785052800.127,
  "payload": {
    "connected": true,
    "armed": false,
    "torque_enabled": false,
    "joint_positions": {"shoulder": 12.5},
    "joint_velocities": {"shoulder": 1.2},
    "position_units": "degrees",
    "velocity_units": "degrees_per_second"
  }
}
```

MQTT messages are telemetry-only and are never retained. Broker failure is
reported under `/health` and does not interrupt robot status, disarming, or
local control. Broker credentials are read from environment files and password
files; do not place them in the broker URL. Use `mqtts://` for TLS outside a
trusted local network.

## Pair the device

Create a private device token before installing the persistent service:

```bash
./pair.sh
```

The token is stored at `.blacknode-hardware/auth.token`, outside Git, with
owner-only file permissions. Save the displayed token for the Blacknode editor
device connection. Pairing can be inspected or replaced later:

```bash
./pair.sh --show
./pair.sh --rotate
```

Rotation immediately invalidates the previous token and restarts an active
hardware service. `/health` remains public for reachability checks.
`/status`, `/capabilities`, and `/rpc` require:

```text
Authorization: Bearer PAIRING_TOKEN
```

## Development

```powershell
python -m pytest packages\blacknode-robot\tests
```

The included I2C adapter is the first physical hardware path; it requires
`smbus2` and an explicit device profile. Additional actuator and sensor
protocols belong in separate adapters and must implement the same contracts.
Serial device discovery uses `pyserial` and is installed with the package.

## Run once on a Raspberry Pi

For a foreground test that stops when the terminal closes:

```bash
./start.sh
```

Press `Ctrl+C` before installing the persistent service.

## Install the persistent service

Install and start the systemd service:

```bash
./pair.sh
./install-service.sh
```

The installer:

- validates `.blacknode-hardware/device.json`
- validates the private pairing token
- generates the service with the current repository path and Linux user
- enables automatic startup after reboot
- restarts the service after a failure
- allows the assigned TCP port when UFW is active
- waits for the HTTP service and connected hardware to pass validation

Re-running `install-service.sh` safely updates the existing service definition.
Use the service manager afterward:

```bash
./service.sh status
./service.sh check --require-hardware
./service.sh restart
./service.sh logs
./service.sh follow
./service.sh stop
./service.sh start
```

For a second, fully isolated robot stack on the same Linux computer, use a
separate checkout and repeat the same instance identity for configuration and
service management:

```bash
export BLACKNODE_HARDWARE_INSTANCE=instance-2
export BLACKNODE_RUNTIME_PORT=8768
./setup_ubuntu.sh
./configure.sh --all --install \
  --serial-port /dev/serial/by-id/NEW_ROBOT
```

The instance produces service names such as
`blacknode-hardware-instance-2-<device>.service`. Configuration, pairing
tokens, calibration state, Python environment, and repository files remain in
that checkout. Port discovery preserves the ports in its own saved manifest
and avoids ports occupied by other Runtime and Hardware stacks. Service
cleanup matches the checkout's exact systemd `WorkingDirectory`, so rerunning
one stack never disables services belonging to another checkout.

`status` shows systemd state and performs a live HTTP check. `logs` shows the
latest 100 journal entries; `follow` streams new entries until `Ctrl+C`.

The service uses:

```text
host: 0.0.0.0
port: 8765
```

Automatic multi-robot setup assigns `8765`, skips the runtime service at
`8766`, and then assigns `8767` and subsequent ports in stable device order.
Each service status also reports its exact stable serial path. Remote
deployment uses that path to bind a one-robot workflow to the paired robot
instead of falling back to USB discovery index `0`.
The authenticated `release` and `resume` RPC methods transfer that serial port
between the read-only device monitor and the deployed robot driver. Releasing
one robot does not stop the other configured robot services.
While a deployment owns the port, status checks never open the serial bus.
For POSIX device paths, the service reports whether the stable path is present,
so reconnect and unplug events remain visible without competing with the
deployed driver.
When UFW is active, `install-service.sh` adds an exact TCP
allow rule for each assigned robot port. It does not open unused ports. Set
`BLACKNODE_CONFIGURE_UFW=0` to manage firewall rules separately:

```bash
BLACKNODE_CONFIGURE_UFW=0 ./configure.sh --all --install
```

For the split leader/follower templates on separate computers, opt in to the
leader rosbridge rule as part of setup:

```bash
./configure.sh --all --install --teleop
```

When UFW is active, `--teleop` also permits TCP `9091`. Rosbridge does not use
the hardware pairing token, so use this option only on a trusted robot LAN and
restrict the generated rule to the follower host or trusted subnet when your
network requires tighter access.

When automatic firewall configuration is disabled, allow the selected port
manually:

```bash
sudo ufw allow 8765/tcp
sudo ufw reload
```

From your PC, the public health URL can be opened directly:

```text
http://PI_IP_ADDRESS:8765/health
```

For example:

```text
http://192.168.1.87:8765/health
```

Use the pairing token for protected endpoints. From PowerShell:

```powershell
$token = Read-Host "Pairing token"
$headers = @{ Authorization = "Bearer $token" }
Invoke-RestMethod http://PI_IP_ADDRESS:8765/status -Headers $headers
Invoke-RestMethod http://PI_IP_ADDRESS:8765/capabilities -Headers $headers
```

`/status` refreshes all configured servo positions on every request and
reports both raw ticks and nominal degree values. It also reports the active
calibration's name, profile, hardware identity, joint count, and digest.
`/health` advertises `calibration_activation` in `features` when this API is
available.

Calibration activation is an explicit, authenticated operation. In the
Blacknode editor, select the paired device under **Deployments**, choose a
saved calibration, and press **Use this calibration**. The service accepts the
profile and calibration only while the device is connected and disarmed, and
only when their servo IDs exactly match the configured bus topology. The
activation is stored in `active-calibration.json` beside the device
configuration and remains active across service restarts. A changed device ID
or bus topology invalidates the stored activation.

An editor message saying that calibration activation is unsupported means the
device service is older than this API. Update this checkout on the device and
restart it:

```bash
./service.sh restart
```

The service remains read-only: calibration activation records the identity
and safety contract used for deployment preflight, but it cannot arm or move
the servos.

To verify the firewall and listener on the Pi:

```bash
sudo ufw status
ss -ltnp | grep 8765
```

From Windows PowerShell:

```powershell
Test-NetConnection PI_IP_ADDRESS -Port 8765
```

The health endpoint can work while status reports `"connected": false`; that
means the service is reachable but one or more configured servos did not
respond. Run `./probe.sh --servos 6` to check the serial connection directly.

Pairing authenticates access but plain HTTP does not encrypt the token or
traffic. Keep the service on a trusted LAN. Use a VPN or HTTPS before internet
exposure, deployment uploads, or motion control.

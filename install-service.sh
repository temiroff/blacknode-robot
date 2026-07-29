#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
venv_python="$repo_dir/.venv/bin/python"
config="${BLACKNODE_HARDWARE_CONFIG:-$repo_dir/.blacknode-hardware/device.json}"
token_file="${BLACKNODE_AUTH_TOKEN_FILE:-$repo_dir/.blacknode-hardware/auth.token}"
host="${BLACKNODE_HARDWARE_HOST:-0.0.0.0}"
port="${BLACKNODE_HARDWARE_PORT:-8765}"
service_user="$(id -un)"
instance="${BLACKNODE_HARDWARE_INSTANCE:-}"
if [[ -n "$instance" && ! "$instance" =~ ^[a-z0-9][a-z0-9-]{0,31}$ ]]; then
  echo "BLACKNODE_HARDWARE_INSTANCE must contain lowercase letters, numbers, or hyphens."
  exit 2
fi
unit_name="blacknode-hardware${instance:+-$instance}.service"
unit_path="/etc/systemd/system/$unit_name"
print_only=false
all_devices=false
configure_teleop=false
ufw_checked=false
ufw_active=false

configure_ufw_port() {
  local service_port="$1"
  local device_name="$2"
  if [[ "${BLACKNODE_CONFIGURE_UFW:-1}" == "0" ]]; then
    return
  fi
  if [[ "$ufw_checked" == false ]]; then
    ufw_checked=true
    if command -v ufw >/dev/null 2>&1 \
      && sudo ufw status 2>/dev/null | grep -qi '^Status: active'; then
      ufw_active=true
    else
      echo "UFW is inactive or unavailable; no firewall rule is needed."
    fi
  fi
  if [[ "$ufw_active" == true ]]; then
    echo "Allowing TCP port $service_port through UFW for $device_name..."
    sudo ufw allow "$service_port/tcp" comment "Blacknode robot: $device_name"
  fi
}

while (($#)); do
  case "$1" in
    --all)
      all_devices=true
      ;;
    --print)
      print_only=true
      ;;
    --teleop)
      configure_teleop=true
      ;;
    *)
      echo "Usage: ./install-service.sh [--all] [--print] [--teleop]"
      exit 2
      ;;
  esac
  shift
done

if [[ "$(uname -s)" != "Linux" ]]; then
  echo "This installer is intended for Ubuntu/Linux."
  exit 1
fi
if [[ "$(id -u)" -eq 0 ]]; then
  echo "Run this installer as your normal user, not as root."
  exit 1
fi
if [[ ! -x "$venv_python" ]]; then
  echo "Blacknode Hardware is not set up yet. Run ./setup_ubuntu.sh first."
  exit 1
fi
if ! command -v systemctl >/dev/null 2>&1; then
  echo "systemd is not available on this device."
  exit 1
fi

if [[ "$all_devices" == true ]]; then
  manifest="$repo_dir/.blacknode-hardware/devices.json"
  if [[ ! -f "$manifest" ]]; then
    echo "No multi-robot configuration found."
    echo "Run ./configure.sh --all first."
    exit 1
  fi
  mapfile -t device_rows < <(
    "$venv_python" "$repo_dir/scripts/configure_devices.py" \
      --root "$repo_dir/.blacknode-hardware" --list
  )
  if (( ${#device_rows[@]} == 0 )); then
    echo "No configured robots found."
    exit 1
  fi

  installed_units=()
  for row in "${device_rows[@]}"; do
    IFS=$'\t' read -r key device_name device_id service_port device_config device_token device_unit <<< "$row"
    if [[ ! -f "$device_token" ]]; then
      echo "No pairing token found for $device_id."
      echo "Run ./pair.sh --all first."
      exit 1
    fi
    echo "Validating $device_id..."
    "$venv_python" "$repo_dir/scripts/configure_device.py" \
      --config "$device_config" --show
    "$venv_python" "$repo_dir/scripts/pair_device.py" \
      --config "$device_config" --token-file "$device_token" --validate

    unit_file="$(mktemp --suffix=.service)"
    "$venv_python" "$repo_dir/scripts/render_systemd_unit.py" \
      --repo "$repo_dir" \
      --user "$service_user" \
      --host "$host" \
      --port "$service_port" \
      --config "$device_config" \
      --auth-token-file "$device_token" > "$unit_file"
    if command -v systemd-analyze >/dev/null 2>&1; then
      systemd-analyze verify "$unit_file"
    fi
    if [[ "$print_only" == true ]]; then
      printf '\n%s\n' "Generated $device_unit"
      printf '%s\n' "==============================="
      command cat "$unit_file"
    else
      unit_target="/etc/systemd/system/$device_unit"
      sudo install -m 0644 "$unit_file" "${unit_target}.new"
      sudo mv -f -- "${unit_target}.new" "$unit_target"
      installed_units+=("$device_unit")
    fi
    rm -f -- "$unit_file"
  done

  if [[ "$print_only" == true ]]; then
    exit 0
  fi

  if systemctl is-active --quiet "$unit_name" \
    || systemctl is-enabled --quiet "$unit_name" 2>/dev/null; then
    echo "Stopping the previous single-robot service to release its port..."
    sudo systemctl disable --now "$unit_name"
  fi
  mapfile -t previous_fleet_units < <(
    systemctl list-unit-files 'blacknode-hardware-*.service' --no-legend \
      | awk '{print $1}'
  )
  for previous_unit in "${previous_fleet_units[@]}"; do
    previous_working_directory="$(
      systemctl show "$previous_unit" --property=WorkingDirectory --value 2>/dev/null || true
    )"
    if [[ "$previous_working_directory" != "$repo_dir" ]]; then
      continue
    fi
    keep_unit=false
    for device_unit in "${installed_units[@]}"; do
      if [[ "$previous_unit" == "$device_unit" ]]; then
        keep_unit=true
        break
      fi
    done
    if [[ "$keep_unit" == false ]]; then
      echo "Disabling disconnected robot service $previous_unit..."
      sudo systemctl disable --now "$previous_unit" || true
    fi
  done
  sudo systemctl daemon-reload
  for device_unit in "${installed_units[@]}"; do
    sudo systemctl enable "$device_unit"
    sudo systemctl restart "$device_unit"
  done
  for row in "${device_rows[@]}"; do
    IFS=$'\t' read -r key device_name device_id service_port device_config device_token device_unit <<< "$row"
    configure_ufw_port "$service_port" "$device_name"
  done
  if [[ "$configure_teleop" == true ]]; then
    configure_ufw_port 9091 "Blacknode leader teleoperation"
  fi
  for row in "${device_rows[@]}"; do
    IFS=$'\t' read -r key device_name device_id service_port device_config device_token device_unit <<< "$row"
    "$venv_python" "$repo_dir/scripts/service_check.py" \
      --url "http://127.0.0.1:$service_port" \
      --token-file "$device_token" \
      --wait 15 --require-hardware
  done
  echo
  echo "All robot services are installed, enabled, and validated."
  echo "Use ./service.sh --all status to inspect them."
  exit 0
fi

if [[ ! -f "$config" ]]; then
  echo "No hardware configuration found."
  echo "Run ./configure.sh --servos 6 first."
  exit 1
fi
if [[ ! -f "$token_file" ]]; then
  echo "No pairing token found."
  echo "Run ./pair.sh first."
  exit 1
fi
echo "Validating hardware configuration..."
"$venv_python" "$repo_dir/scripts/configure_device.py" --config "$config" --show
echo
echo "Validating pairing credentials..."
"$venv_python" "$repo_dir/scripts/pair_device.py" \
  --config "$config" \
  --token-file "$token_file" \
  --validate

unit_file="$(mktemp --suffix=.service)"
unit_new="${unit_path}.new"
cleanup() {
  rm -f -- "$unit_file"
}
trap cleanup EXIT

"$venv_python" "$repo_dir/scripts/render_systemd_unit.py" \
  --repo "$repo_dir" \
  --user "$service_user" \
  --host "$host" \
  --port "$port" \
  --config "$config" \
  --auth-token-file "$token_file" > "$unit_file"

if command -v systemd-analyze >/dev/null 2>&1; then
  systemd-analyze verify "$unit_file"
fi

if [[ "$print_only" == true ]]; then
  printf '\n'
  printf '%s\n' "Generated $unit_name"
  printf '%s\n' "==============================="
  command cat "$unit_file"
  exit 0
fi

echo
echo "Installing $unit_name..."
sudo install -m 0644 "$unit_file" "$unit_new"
sudo mv -f -- "$unit_new" "$unit_path"
sudo systemctl daemon-reload
sudo systemctl enable "$unit_name"
if ! sudo systemctl restart "$unit_name"; then
  echo "The service could not start. Stop any manually running ./start.sh and retry."
  sudo systemctl --no-pager --full status "$unit_name" || true
  exit 1
fi

if ! "$repo_dir/service.sh" check --wait 15 --require-hardware; then
  echo
  echo "The service was installed, but validation did not fully pass."
  echo "Run ./service.sh logs to inspect it."
  exit 1
fi
single_device_name="$(
  "$venv_python" "$repo_dir/scripts/configure_device.py" --config "$config" --show \
    | awk -F ': ' '/^Name: / {print $2; exit}'
)"
configure_ufw_port "$port" "${single_device_name:-$unit_name}"
if [[ "$configure_teleop" == true ]]; then
  configure_ufw_port 9091 "Blacknode leader teleoperation"
fi

echo
echo "Service installed and enabled at boot."
echo "Re-run ./install-service.sh anytime to update its configuration."
echo "Use ./service.sh status, restart, check, or logs."

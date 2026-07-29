#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
venv_python="$repo_dir/.venv/bin/python"
config="${BLACKNODE_HARDWARE_CONFIG:-$repo_dir/.blacknode-hardware/device.json}"
token_file="${BLACKNODE_AUTH_TOKEN_FILE:-$repo_dir/.blacknode-hardware/auth.token}"
unit_name="blacknode-hardware.service"
port="${BLACKNODE_HARDWARE_PORT:-8765}"
device_ip="${BLACKNODE_DEVICE_IP:-}"
action="${1:-}"
token_existed=false
all_devices=false

if [[ "${1:-}" == "--all" ]]; then
  all_devices=true
  shift
  action="${1:-}"
fi

if [[ ! -x "$venv_python" ]]; then
  echo "Blacknode Hardware is not set up yet. Run ./setup_ubuntu.sh first."
  exit 1
fi
if [[ -z "$device_ip" ]]; then
  device_ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
fi
device_ip="${device_ip:-DEVICE_IP}"

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
  for row in "${device_rows[@]}"; do
    IFS=$'\t' read -r key device_name device_id service_port device_config device_token device_unit <<< "$row"
    echo
    echo "Editor device"
    echo "============="
    echo "Name: $device_name"
    echo "Address: http://$device_ip:$service_port"
    existed=false
    if [[ -f "$device_token" ]]; then existed=true; fi
    "$venv_python" "$repo_dir/scripts/pair_device.py" \
      --config "$device_config" \
      --token-file "$device_token" \
      "$@"
    if [[ "$action" == "--rotate" || "$existed" == false ]] \
      && command -v systemctl >/dev/null 2>&1 \
      && systemctl is-active --quiet "$device_unit"; then
      echo "Restarting $device_id to apply pairing..."
      sudo systemctl restart "$device_unit"
      "$venv_python" "$repo_dir/scripts/service_check.py" \
        --url "http://127.0.0.1:$service_port" \
        --token-file "$device_token" \
        --wait 15 --require-hardware
    fi
  done
  exit 0
fi

if [[ ! -f "$config" ]]; then
  echo "No hardware configuration found."
  echo "Run ./configure.sh --servos 6 first."
  exit 1
fi
echo "Editor address: http://$device_ip:$port"
if [[ -f "$token_file" ]]; then
  token_existed=true
fi

cd "$repo_dir"
"$venv_python" "$repo_dir/scripts/pair_device.py" \
  --config "$config" \
  --token-file "$token_file" \
  "$@"

credentials_changed=false
if [[ "$action" == "--rotate" || "$token_existed" == false ]]; then
  credentials_changed=true
fi

if [[ "$credentials_changed" == true ]] && command -v systemctl >/dev/null 2>&1; then
  if systemctl is-active --quiet "$unit_name"; then
    echo
    echo "Restarting the hardware service to apply pairing..."
    sudo systemctl restart "$unit_name"
    "$repo_dir/service.sh" check --wait 15 --require-hardware
  else
    echo
    echo "Pairing is ready. Run ./install-service.sh to install the persistent service."
  fi
fi

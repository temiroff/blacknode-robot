#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
venv_dir="$repo_dir/.venv"
host="${BLACKNODE_HARDWARE_HOST:-0.0.0.0}"
port="${BLACKNODE_HARDWARE_PORT:-8765}"
config="${BLACKNODE_HARDWARE_CONFIG:-$repo_dir/.blacknode-hardware/device.json}"

if [[ ! -f "$venv_dir/bin/activate" ]]; then
  echo "Blacknode Hardware is not set up yet. Run ./setup_ubuntu.sh first."
  exit 1
fi

if [[ ! -f "$config" ]]; then
  echo "No hardware configuration found."
  echo "Run ./configure.sh --servos 6 first."
  exit 1
fi

# shellcheck disable=SC1091
source "$venv_dir/bin/activate"
echo "Starting Blacknode Hardware service"
echo "Listening on http://$host:$port"
echo "Health endpoint: http://$(hostname -I | awk '{print $1}'):$port/health"
echo "Press Ctrl+C to stop."
service_args=(--host "$host" --port "$port" --config "$config")
if [[ -n "${BLACKNODE_DEVICE_ID:-}" ]]; then
  service_args+=(--device-id "$BLACKNODE_DEVICE_ID")
fi
exec python "$repo_dir/scripts/hardware_service.py" "${service_args[@]}" "$@"

#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
venv_python="$repo_dir/.venv/bin/python"
instance="${BLACKNODE_HARDWARE_INSTANCE:-}"
if [[ -n "$instance" && ! "$instance" =~ ^[a-z0-9][a-z0-9-]{0,31}$ ]]; then
  echo "BLACKNODE_HARDWARE_INSTANCE must contain lowercase letters, numbers, or hyphens."
  exit 2
fi
unit_name="blacknode-hardware${instance:+-$instance}.service"
port="${BLACKNODE_HARDWARE_PORT:-8765}"
token_file="${BLACKNODE_AUTH_TOKEN_FILE:-$repo_dir/.blacknode-hardware/auth.token}"
all_devices=false

usage() {
  echo "Usage: ./service.sh [--all] COMMAND"
  echo
  echo "Commands:"
  echo "  status              Show systemd status and service health"
  echo "  start               Start and validate the service"
  echo "  stop                Stop the service"
  echo "  restart             Restart and validate the service"
  echo "  check [OPTIONS]     Check HTTP and hardware status"
  echo "  logs                Show the latest service logs"
  echo "  follow              Follow service logs"
}

if [[ "$(uname -s)" != "Linux" ]]; then
  echo "This service manager is intended for Ubuntu/Linux."
  exit 1
fi
if [[ ! -x "$venv_python" ]]; then
  echo "Blacknode Hardware is not set up yet. Run ./setup_ubuntu.sh first."
  exit 1
fi

if [[ "${1:-}" == "--all" ]]; then
  all_devices=true
  shift
fi

command_name="${1:-}"
if [[ -z "$command_name" ]]; then
  usage
  exit 2
fi
shift

if [[ "$all_devices" == true ]]; then
  manifest="$repo_dir/.blacknode-hardware/devices.json"
  if [[ ! -f "$manifest" ]]; then
    echo "No multi-robot configuration found. Run ./configure.sh --all first."
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

  check_all() {
    local wait_seconds="0"
    if (($#)); then
      wait_seconds="$1"
      shift
    fi
    for row in "${device_rows[@]}"; do
      IFS=$'\t' read -r key device_name device_id service_port device_config device_token device_unit <<< "$row"
      check_args=(
        --url "http://127.0.0.1:$service_port"
        --token-file "$device_token"
        --require-hardware
      )
      if [[ "$wait_seconds" != "0" ]]; then check_args+=(--wait "$wait_seconds"); fi
      "$venv_python" "$repo_dir/scripts/service_check.py" "${check_args[@]}" "$@"
    done
  }

  case "$command_name" in
    status)
      for row in "${device_rows[@]}"; do
        IFS=$'\t' read -r key device_name device_id service_port device_config device_token device_unit <<< "$row"
        echo "$device_name · http://127.0.0.1:$service_port"
        sudo systemctl --no-pager --full status "$device_unit" || true
      done
      check_all
      ;;
    start|restart)
      for row in "${device_rows[@]}"; do
        IFS=$'\t' read -r key device_name device_id service_port device_config device_token device_unit <<< "$row"
        sudo systemctl "$command_name" "$device_unit"
      done
      check_all 15 "$@"
      ;;
    stop)
      for row in "${device_rows[@]}"; do
        IFS=$'\t' read -r key device_name device_id service_port device_config device_token device_unit <<< "$row"
        sudo systemctl stop "$device_unit"
      done
      echo "All Blacknode Hardware services stopped."
      ;;
    check)
      check_all 0 "$@"
      ;;
    logs)
      journal_args=()
      for row in "${device_rows[@]}"; do
        IFS=$'\t' read -r key device_name device_id service_port device_config device_token device_unit <<< "$row"
        journal_args+=(-u "$device_unit")
      done
      sudo journalctl "${journal_args[@]}" -n 100 --no-pager
      ;;
    follow)
      journal_args=()
      for row in "${device_rows[@]}"; do
        IFS=$'\t' read -r key device_name device_id service_port device_config device_token device_unit <<< "$row"
        journal_args+=(-u "$device_unit")
      done
      sudo journalctl "${journal_args[@]}" -f
      ;;
    -h|--help|help)
      usage
      ;;
    *)
      echo "Unknown command: $command_name"
      usage
      exit 2
      ;;
  esac
  exit 0
fi

check_service() {
  check_args=(--url "http://127.0.0.1:$port")
  if [[ -f "$token_file" ]]; then
    check_args+=(--token-file "$token_file")
  fi
  "$venv_python" "$repo_dir/scripts/service_check.py" "${check_args[@]}" "$@"
}

case "$command_name" in
  status)
    sudo systemctl --no-pager --full status "$unit_name" || true
    echo
    check_service
    ;;
  start)
    sudo systemctl start "$unit_name"
    check_service --wait 15 "$@"
    ;;
  stop)
    sudo systemctl stop "$unit_name"
    echo "Blacknode Hardware service stopped."
    ;;
  restart)
    sudo systemctl restart "$unit_name"
    check_service --wait 15 "$@"
    ;;
  check)
    check_service "$@"
    ;;
  logs)
    sudo journalctl -u "$unit_name" -n 100 --no-pager
    ;;
  follow)
    sudo journalctl -u "$unit_name" -f
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    echo "Unknown command: $command_name"
    usage
    exit 2
    ;;
esac

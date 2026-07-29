#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
venv_dir="$repo_dir/.venv"
instance="${BLACKNODE_HARDWARE_INSTANCE:-}"
if [[ -n "$instance" && ! "$instance" =~ ^[a-z0-9][a-z0-9-]{0,31}$ ]]; then
  echo "BLACKNODE_HARDWARE_INSTANCE must contain lowercase letters, numbers, or hyphens."
  exit 2
fi
instance_args=()
if [[ -n "$instance" ]]; then
  instance_args=(--instance "$instance")
fi

if [[ ! -f "$venv_dir/bin/activate" ]]; then
  echo "Blacknode Hardware is not set up yet. Run ./setup_ubuntu.sh first."
  exit 1
fi

# shellcheck disable=SC1091
source "$venv_dir/bin/activate"
cd "$repo_dir"
if [[ "${1:-}" == "--all" ]]; then
  shift
  install_all=false
  configure_teleop=false
  configure_args=()
  for argument in "$@"; do
    if [[ "$argument" == "--install" ]]; then
      install_all=true
    elif [[ "$argument" == "--teleop" ]]; then
      configure_teleop=true
    else
      configure_args+=("$argument")
    fi
  done
  restore_previous_fleet=false
  restore_fleet_on_failure() {
    exit_code=$?
    if (( exit_code != 0 )) && [[ "$restore_previous_fleet" == true ]]; then
      echo
      echo "Automatic configuration failed; restarting the previous hardware services..."
      "$repo_dir/service.sh" --all start || true
    fi
    exit "$exit_code"
  }
  if [[ "$install_all" == true \
    && -f "$repo_dir/.blacknode-hardware/devices.json" \
    && "$(uname -s)" == "Linux" \
    && -x "$repo_dir/service.sh" ]]; then
    echo "Stopping configured hardware services briefly so every serial bus can be rescanned..."
    "$repo_dir/service.sh" --all stop || true
    restore_previous_fleet=true
    trap restore_fleet_on_failure EXIT
  fi
  python "$repo_dir/scripts/configure_devices.py" \
    --root "$repo_dir/.blacknode-hardware" \
    "${instance_args[@]}" \
    "${configure_args[@]}"
  if [[ "$install_all" == true ]]; then
    "$repo_dir/pair.sh" --all
    install_args=(--all)
    if [[ "$configure_teleop" == true ]]; then
      install_args+=(--teleop)
    fi
    "$repo_dir/install-service.sh" "${install_args[@]}"
    echo
    echo "Editor pairing checklist"
    echo "========================"
    "$repo_dir/pair.sh" --all --show
  fi
  restore_previous_fleet=false
  trap - EXIT
  exit 0
fi
exec python "$repo_dir/scripts/configure_device.py" "$@"

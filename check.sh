#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
venv_dir="$repo_dir/.venv"
bus="${BLACKNODE_I2C_BUS:-1}"
address="${BLACKNODE_I2C_ADDRESS:-0x7A}"
probe="--probe-address"

if [[ "${1:-}" == "--software-only" ]]; then
  probe=""
fi

if [[ ! -f "$venv_dir/bin/activate" ]]; then
  echo "Blacknode Hardware is not set up yet. Run ./setup_ubuntu.sh first."
  exit 1
fi

# shellcheck disable=SC1091
source "$venv_dir/bin/activate"
echo "Blacknode Hardware check"
echo "========================"
if [[ -n "$probe" ]]; then
  python "$repo_dir/scripts/hardware_doctor.py" \
    "$probe" \
    --bus "$bus" \
    --address "$address"
else
  python "$repo_dir/scripts/hardware_doctor.py"
fi
echo
if [[ -n "$probe" ]]; then
  echo "Hardware check completed. No motor commands were sent."
else
  echo "Software-only check completed. No hardware was probed."
fi

#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
venv_dir="$repo_dir/.venv"

if [[ ! -f "$venv_dir/bin/activate" ]]; then
  echo "Blacknode Hardware is not set up yet. Run ./setup_ubuntu.sh first."
  exit 1
fi

# shellcheck disable=SC1091
source "$venv_dir/bin/activate"
echo "Blacknode Hardware device discovery"
echo "==================================="
python -m serial.tools.list_ports -v || true
echo
echo "Serial device paths:"
shopt -s nullglob
paths=(/dev/serial/by-id/* /dev/ttyACM* /dev/ttyUSB*)
if (( ${#paths[@]} == 0 )); then
  echo "No serial devices found. Connect the device and run ./discover.sh again."
else
  printf '  %s\n' "${paths[@]}"
fi
echo
echo "Automatically configure every responding robot:"
echo "  ./configure.sh --all --install"

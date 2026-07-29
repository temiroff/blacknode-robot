#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
venv_dir="$repo_dir/.venv"
port="${BLACKNODE_SERIAL_PORT:-}"
baudrate="${BLACKNODE_SERIAL_BAUDRATE:-1000000}"
ids="${BLACKNODE_SERVO_IDS:-1-20}"

usage() {
  echo "Usage: ./probe.sh [--servos COUNT]"
  echo
  echo "  -s, --servos COUNT  Probe servo IDs 1 through COUNT."
  echo "  -h, --help          Show this help."
}

while (($#)); do
  case "$1" in
    -s|-servos|--servos|-dof|--dof)
      if (($# < 2)); then
        echo "Missing servo count after $1."
        usage
        exit 2
      fi
      if [[ ! "$2" =~ ^[1-9][0-9]*$ ]] || ((10#$2 > 253)); then
        echo "Servo count must be a whole number from 1 to 253."
        exit 2
      fi
      ids="1-$((10#$2))"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      usage
      exit 2
      ;;
  esac
done

if [[ ! -f "$venv_dir/bin/activate" ]]; then
  echo "Blacknode Hardware is not set up yet. Run ./setup_ubuntu.sh first."
  exit 1
fi

if [[ -z "$port" ]]; then
  for candidate in /dev/serial/by-id/* /dev/ttyACM* /dev/ttyUSB*; do
    if [[ -e "$candidate" ]]; then port="$candidate"; break; fi
  done
fi

if [[ -z "$port" ]]; then
  echo "No serial device found. Run ./discover.sh after connecting the device."
  exit 1
fi

# shellcheck disable=SC1091
source "$venv_dir/bin/activate"
echo "Read-only serial actuator probe"
echo "Port: $port"
echo "Baudrate: $baudrate"
echo "Servo IDs: $ids"
python "$repo_dir/scripts/serial_probe.py" --port "$port" --baudrate "$baudrate" --ids "$ids"

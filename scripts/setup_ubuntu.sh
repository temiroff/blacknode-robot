#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_dir"

echo "Blacknode Hardware setup"
echo "========================"

if [[ "$(uname -s)" != "Linux" ]]; then
  echo "This setup script is intended for Ubuntu/Linux."
  exit 1
fi

if [[ "$(id -u)" -eq 0 ]]; then
  echo "Run this script as your normal user, not as root."
  exit 1
fi

echo "Installing system prerequisites..."
sudo apt-get update
sudo apt-get install -y i2c-tools python3-pip python3-venv

if getent group i2c >/dev/null 2>&1; then
  sudo usermod -aG i2c "$USER"
  echo "Added $USER to the i2c group. Log out and back in before hardware access."
else
  echo "The i2c group is unavailable. Check that the I2C kernel interface is enabled."
fi

echo "Creating the local Python environment..."
python3 -m venv .venv
# shellcheck disable=SC1091
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e . smbus2 pyserial feetech-servo-sdk

echo
echo "Running readiness checks..."
python scripts/hardware_doctor.py || true

echo
echo "Setup finished. If /dev/i2c-1 is missing, enable I2C in /boot/firmware/config.txt:"
echo "  dtparam=i2c_arm=on"
echo "Then reboot and run:"
echo "  ./check.sh"
echo
echo "To discover and probe serial hardware, run:"
echo "  ./discover.sh"
echo "  ./probe.sh --servos 6"
echo
echo "After configuring the device, install the persistent service:"
echo "  ./configure.sh --servos 6"
echo "  ./pair.sh"
echo "  ./install-service.sh"

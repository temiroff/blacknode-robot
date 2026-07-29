"""Create, display, rotate, or validate the local device pairing token."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from blacknode_robot.devices.auth import load_auth_token, save_auth_token, token_fingerprint
from blacknode_robot.devices.device_config import DEFAULT_CONFIG_PATH, load_device_config


def print_pairing(token: str, token_path: Path, config_path: Path) -> None:
    config = load_device_config(config_path)
    print("Blacknode Hardware pairing")
    print("==========================")
    print(f"Name: {config['name']}")
    print(f"Device ID: {config['device_id']}")
    print(f"Token file: {token_path}")
    print(f"Fingerprint: {token_fingerprint(token)}")
    print(f"Pairing token: {token}")
    print()
    print("Keep this token private. Save it for the Blacknode editor device connection.")


def main() -> int:
    default_config = Path(os.environ.get("BLACKNODE_HARDWARE_CONFIG", DEFAULT_CONFIG_PATH))
    default_token = Path(
        os.environ.get(
            "BLACKNODE_AUTH_TOKEN_FILE",
            default_config.parent / "auth.token",
        )
    )
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--rotate", action="store_true", help="replace the current token")
    action.add_argument("--show", action="store_true", help="display the current token")
    action.add_argument("--validate", action="store_true", help="validate without displaying the token")
    parser.add_argument("--config", type=Path, default=default_config)
    parser.add_argument("--token-file", type=Path, default=default_token)
    args = parser.parse_args()

    load_device_config(args.config)
    if args.rotate:
        token_path, token = save_auth_token(args.token_file)
        print_pairing(token, token_path, args.config)
        print("Previous pairing credentials are no longer valid.")
        return 0

    if args.validate:
        token = load_auth_token(args.token_file)
        print(f"Pairing token is valid. Fingerprint: {token_fingerprint(token)}")
        return 0

    if args.show:
        print_pairing(load_auth_token(args.token_file), args.token_file, args.config)
        return 0

    if args.token_file.exists():
        print_pairing(load_auth_token(args.token_file), args.token_file, args.config)
        print("A pairing token already exists. Use --rotate to replace it.")
        return 0

    token_path, token = save_auth_token(args.token_file)
    print_pairing(token, token_path, args.config)
    print("New pairing credentials created.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

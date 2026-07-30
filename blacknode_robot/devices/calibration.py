"""Persist and validate one active calibration for a physical hardware service."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import tempfile
import threading
from typing import Any


CALIBRATION_STORE_VERSION = 1
CALIBRATION_SCHEMA_VERSION = 1


class CalibrationError(ValueError):
    """A calibration cannot be safely activated for this device."""


class CalibrationStore:
    """Hardware-bound active calibration with exact servo-topology validation."""

    def __init__(
        self,
        path: str | Path,
        *,
        device_id: str,
        servos: list[dict[str, Any]],
    ) -> None:
        self.path = Path(path)
        self.device_id = str(device_id or "").strip()
        self.servos = _normalize_device_servos(servos)
        self._lock = threading.RLock()
        self._active: dict[str, Any] | None = None
        self._error = ""
        self._load()

    def status(self) -> dict[str, Any]:
        with self._lock:
            if self._active is None:
                return {"active": False, "error": self._error}
            return {
                "active": True,
                "name": str(
                    self._active.get("name")
                    or self._active.get("hardware_id")
                    or ""
                ),
                "profile_id": self._active["profile_id"],
                "hardware_id": self._active["hardware_id"],
                "target_device_id": self._active["target_device_id"],
                "activated_at": self._active["activated_at"],
                "joint_count": len(self._active["topology"]),
                "digest": self._active["digest"],
                "topology": dict(self._active["topology"]),
                "joints": {
                    str(name): dict(values)
                    for name, values in dict(
                        self._active["calibration"].get("joints") or {}
                    ).items()
                    if isinstance(values, dict)
                },
                "error": "",
            }

    def activate(
        self,
        profile: dict[str, Any],
        calibration: dict[str, Any],
    ) -> dict[str, Any]:
        record = _activation_record(
            profile,
            calibration,
            device_id=self.device_id,
            device_servos=self.servos,
        )
        with self._lock:
            self._write(record)
            self._active = record
            self._error = ""
            return self.status()

    def deactivate(self) -> dict[str, Any]:
        with self._lock:
            if self.path.exists():
                self.path.unlink()
            self._active = None
            self._error = ""
            return self.status()

    def _load(self) -> None:
        if not self.path.exists():
            return
        try:
            value = json.loads(self.path.read_text(encoding="utf-8"))
            if not isinstance(value, dict):
                raise CalibrationError("active calibration must be a JSON object")
            if value.get("version") != CALIBRATION_STORE_VERSION:
                raise CalibrationError(
                    f"active calibration version must be {CALIBRATION_STORE_VERSION}"
                )
            if value.get("target_device_id") != self.device_id:
                raise CalibrationError("active calibration belongs to a different device")
            topology = value.get("topology")
            expected_device_topology = {
                str(item["id"]): item["name"] for item in self.servos
            }
            if value.get("device_topology") != expected_device_topology:
                raise CalibrationError(
                    "active calibration servo topology does not match this device"
                )
            if not isinstance(topology, dict) or set(topology) != set(expected_device_topology):
                raise CalibrationError("active calibration profile topology is invalid")
            if not value.get("profile_id") or not value.get("hardware_id"):
                raise CalibrationError("active calibration identity is incomplete")
            self._active = value
        except (OSError, json.JSONDecodeError, CalibrationError) as exc:
            self._active = None
            self._error = str(exc)

    def _write(self, payload: dict[str, Any]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path: Path | None = None
        try:
            with tempfile.NamedTemporaryFile(
                "w",
                encoding="utf-8",
                dir=self.path.parent,
                prefix=f".{self.path.name}.",
                suffix=".tmp",
                delete=False,
            ) as temporary:
                json.dump(payload, temporary, indent=2, sort_keys=True)
                temporary.write("\n")
                temporary.flush()
                os.fsync(temporary.fileno())
                temporary_path = Path(temporary.name)
            os.replace(temporary_path, self.path)
        finally:
            if temporary_path is not None and temporary_path.exists():
                temporary_path.unlink()


def _normalize_device_servos(servos: list[dict[str, Any]]) -> list[dict[str, Any]]:
    normalized: list[dict[str, Any]] = []
    seen: set[int] = set()
    for item in servos:
        servo_id = item.get("id") if isinstance(item, dict) else None
        name = str(item.get("name") or "").strip() if isinstance(item, dict) else ""
        if isinstance(servo_id, bool) or not isinstance(servo_id, int):
            raise CalibrationError("device servo IDs must be whole numbers")
        if not 1 <= servo_id <= 253 or servo_id in seen or not name:
            raise CalibrationError("device servo topology is invalid")
        seen.add(servo_id)
        normalized.append({"id": servo_id, "name": name})
    if not normalized:
        raise CalibrationError("device servo topology is empty")
    return sorted(normalized, key=lambda item: item["id"])


def _activation_record(
    profile: dict[str, Any],
    calibration: dict[str, Any],
    *,
    device_id: str,
    device_servos: list[dict[str, Any]],
) -> dict[str, Any]:
    if not isinstance(profile, dict) or not isinstance(calibration, dict):
        raise CalibrationError("profile and calibration must be JSON objects")
    profile_id = str(profile.get("id") or "").strip()
    calibration_profile_id = str(calibration.get("profile_id") or "").strip()
    hardware_id = str(calibration.get("hardware_id") or "").strip()
    if not profile_id or profile_id != calibration_profile_id:
        raise CalibrationError("calibration profile_id does not match the selected profile")
    if calibration.get("schema_version") != CALIBRATION_SCHEMA_VERSION:
        raise CalibrationError(
            f"calibration schema_version must be {CALIBRATION_SCHEMA_VERSION}"
        )
    if not hardware_id:
        raise CalibrationError("calibration hardware_id is required")

    profile_joints = profile.get("joints")
    calibration_joints = calibration.get("joints")
    if not isinstance(profile_joints, list) or not isinstance(calibration_joints, dict):
        raise CalibrationError("profile and calibration joint definitions are required")

    topology: dict[str, str] = {}
    profile_names: set[str] = set()
    for joint in profile_joints:
        if not isinstance(joint, dict):
            raise CalibrationError("each profile joint must be an object")
        name = str(joint.get("id") or "").strip()
        servo_id = joint.get("servo_id")
        if (
            not name
            or isinstance(servo_id, bool)
            or not isinstance(servo_id, int)
            or str(servo_id) in topology
        ):
            raise CalibrationError("profile joint topology is invalid")
        topology[str(servo_id)] = name
        profile_names.add(name)

    device_ids = {str(item["id"]) for item in device_servos}
    if set(topology) != device_ids:
        raise CalibrationError(
            "profile servo IDs do not exactly match the configured device servo IDs"
        )
    if set(calibration_joints) != profile_names:
        raise CalibrationError(
            "calibration joints do not exactly match the selected profile"
        )

    for name, values in calibration_joints.items():
        if not isinstance(values, dict):
            raise CalibrationError(f"calibration for {name} must be an object")
        home_ticks = values.get("home_ticks")
        safe_min = values.get("safe_min_deg")
        safe_max = values.get("safe_max_deg")
        if (
            isinstance(home_ticks, bool)
            or not isinstance(home_ticks, int)
            or not 0 <= home_ticks < 4096
        ):
            raise CalibrationError(f"calibration home_ticks is invalid for {name}")
        if (
            isinstance(safe_min, bool)
            or isinstance(safe_max, bool)
            or not isinstance(safe_min, (int, float))
            or not isinstance(safe_max, (int, float))
            or float(safe_min) >= float(safe_max)
        ):
            raise CalibrationError(f"calibration safe range is invalid for {name}")

    canonical = json.dumps(
        {"profile": profile, "calibration": calibration},
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return {
        "version": CALIBRATION_STORE_VERSION,
        "name": str(calibration.get("name") or hardware_id).strip()[:96],
        "target_device_id": device_id,
        "profile_id": profile_id,
        "hardware_id": hardware_id,
        "activated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "topology": topology,
        "device_topology": {
            str(item["id"]): item["name"] for item in device_servos
        },
        "digest": hashlib.sha256(canonical).hexdigest()[:16],
        "profile": profile,
        "calibration": calibration,
    }

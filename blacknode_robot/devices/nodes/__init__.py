"""Hardware capability inspection nodes."""

from __future__ import annotations

from blacknode.node import Any, Bool, Dict, List, Text, node


_CATEGORY = "Robot"


@node(
    name="HardwareCapabilities",
    component="devices",
    category=_CATEGORY,
    description="Report the connected-device capabilities requested by a robot profile.",
    inputs={
        "device_id": Text(default="device"),
        "capabilities": List(default=["mobile_base"]),
        "refresh": Any,
    },
    outputs={"available": Bool, "device_id": Text, "capabilities": List, "state": Dict, "report": Text},
)
def hardware_capabilities(ctx: dict) -> dict:
    device_id = str(ctx.get("device_id") or "device")
    capabilities = [str(value) for value in (ctx.get("capabilities") or [])]
    state = {
        "device_id": device_id,
        "connected": False,
        "armed": False,
        "capabilities": capabilities,
        "provider": "unconfigured",
    }
    return {
        "available": False,
        "device_id": device_id,
        "capabilities": capabilities,
        "state": state,
        "report": "no hardware adapter configured; select an adapter in the device profile",
    }

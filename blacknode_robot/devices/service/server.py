"""Small standard-library JSON-RPC HTTP server for local device testing."""

from __future__ import annotations

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

from .runtime import HardwareRuntime
from ..version import service_version


class HardwareRequestHandler(BaseHTTPRequestHandler):
    runtime: HardwareRuntime
    auth_token: str | None = None

    def log_message(self, _format: str, *_args: Any) -> None:
        return

    def _send(
        self,
        status: int,
        payload: dict[str, Any],
        headers: dict[str, str] | None = None,
    ) -> None:
        data = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        for name, value in (headers or {}).items():
            self.send_header(name, value)
        self.end_headers()
        self.wfile.write(data)

    def _authorized(self) -> bool:
        if self.auth_token is None:
            return True
        from ..auth import authorization_matches

        return authorization_matches(self.headers.get("Authorization"), self.auth_token)

    def _require_authorization(self) -> bool:
        if self._authorized():
            return True
        self._send(
            401,
            {"ok": False, "error": "authentication required"},
            {"WWW-Authenticate": "Bearer"},
        )
        return False

    def _require_mutation_authorization(self) -> bool:
        if self.auth_token is None:
            self._send(
                403,
                {
                    "ok": False,
                    "error": "pairing authentication is required for calibration changes",
                },
            )
            return False
        return self._require_authorization()

    def _json_body(self, *, limit: int = 1024 * 1024) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0 or length > limit:
            raise ValueError("request body must be JSON and no larger than 1 MB")
        value = json.loads(self.rfile.read(length))
        if not isinstance(value, dict):
            raise ValueError("request body must be a JSON object")
        return value

    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/health":
            self._send(
                200,
                {
                    "ok": True,
                    "service": "blacknode-hardware",
                    "api_version": 2,
                    "software_version": service_version(),
                    "features": [
                        "calibration_activation",
                        *self.runtime.service_features(),
                    ],
                    "telemetry": self.runtime.telemetry_status(),
                    "auth_required": self.auth_token is not None,
                },
            )
            return
        if not self._require_authorization():
            return
        if self.path == "/status":
            self._send(200, self.runtime.status())
        elif self.path == "/capabilities":
            self._send(200, self.runtime.capabilities())
        elif self.path == "/calibration":
            self._send(200, self.runtime.calibration_status())
        else:
            self._send(404, {"ok": False, "error": "not found"})

    def do_POST(self) -> None:  # noqa: N802
        if self.path == "/calibration":
            if not self._require_mutation_authorization():
                return
            try:
                request = self._json_body()
                result = self.runtime.activate_calibration(
                    request.get("profile"),
                    request.get("calibration"),
                )
                self._send(200, {"ok": True, "calibration": result})
            except Exception as exc:
                self._send(400, {"ok": False, "error": str(exc)})
            return
        if self.path != "/rpc":
            self._send(404, {"ok": False, "error": "not found"})
            return
        if not self._require_authorization():
            return
        try:
            request = self._json_body()
            result = self.runtime.call(str(request.get("method", "")), request.get("params") or {})
            self._send(200, {"jsonrpc": "2.0", "id": request.get("id"), "result": result})
        except Exception as exc:
            self._send(200, {"jsonrpc": "2.0", "id": None, "error": {"message": str(exc)}})

    def do_DELETE(self) -> None:  # noqa: N802
        if self.path != "/calibration":
            self._send(404, {"ok": False, "error": "not found"})
            return
        if not self._require_mutation_authorization():
            return
        try:
            result = self.runtime.deactivate_calibration()
            self._send(200, {"ok": True, "calibration": result})
        except Exception as exc:
            self._send(400, {"ok": False, "error": str(exc)})


def create_server(
    runtime: HardwareRuntime,
    host: str = "127.0.0.1",
    port: int = 8765,
    auth_token: str | None = None,
) -> ThreadingHTTPServer:
    handler = type(
        "BoundHardwareRequestHandler",
        (HardwareRequestHandler,),
        {"runtime": runtime, "auth_token": auth_token},
    )
    return ThreadingHTTPServer((host, port), handler)


def serve(
    runtime: HardwareRuntime,
    host: str = "127.0.0.1",
    port: int = 8765,
    auth_token: str | None = None,
) -> None:
    server = create_server(runtime, host, port, auth_token=auth_token)
    print(f"blacknode-hardware listening on http://{host}:{port}")
    try:
        server.serve_forever()
    finally:
        server.server_close()
        runtime.close()

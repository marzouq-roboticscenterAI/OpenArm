"""A tiny, dependency-free web server to demo and control the rig.

Serves a single-page dashboard (``static/index.html``) plus a small JSON API
backed by a shared :class:`roverservo.Rover`. Built on the standard-library
:mod:`http.server` with a threading mixin so the fast telemetry polling never
blocks the control endpoints.

Run it with ``python -m roverservo`` (see :mod:`roverservo.__main__`).

API summary (all POST bodies are JSON; all responses are JSON):

    GET  /api/state                 -> full rig snapshot
    POST /api/estop                 -> stop BOTH devices now

    POST /api/rover/enable
    POST /api/rover/disable
    POST /api/rover/clear_errors
    POST /api/rover/mode            {"mode": "ackermann|tilt|spin|park"}
    POST /api/rover/light           {"on": true|false}
    POST /api/rover/drive           {"linear":.., "angular":.., "steer":..}
    POST /api/rover/stop

    POST /api/servo/enable
    POST /api/servo/disable
    POST /api/servo/check
    POST /api/servo/fault_reset
    POST /api/servo/move            {"pulses": .., "velocity": ..}
    POST /api/servo/raise           {"pulses": .., "velocity": ..}
    POST /api/servo/lower           {"pulses": .., "velocity": ..}
    POST /api/servo/jog             {"velocity": ..}      (deadman: refresh to hold)
    POST /api/servo/stop
    POST /api/servo/estop
"""

from __future__ import annotations

import json
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable

from .robot import Rover

_STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
_CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".svg": "image/svg+xml",
    ".ico": "image/x-icon",
}


def make_handler(rig: Rover) -> type[BaseHTTPRequestHandler]:
    """Build a request-handler class bound to a specific :class:`Rover`."""

    def num(v, default=0.0) -> float:
        try:
            return float(v)
        except (TypeError, ValueError):
            return default

    def integer(v, default=0) -> int:
        try:
            return int(v)
        except (TypeError, ValueError):
            return default

    # POST routes: path -> handler(body_dict) -> optional dict merged into reply.
    routes: dict[str, Callable[[dict], object]] = {
        "/api/estop": lambda b: rig.estop_all(),

        "/api/rover/enable": lambda b: rig.rover_enable(),
        "/api/rover/disable": lambda b: rig.rover_disable(),
        "/api/rover/clear_errors": lambda b: rig.rover_clear_errors(integer(b.get("code", 0))),
        "/api/rover/mode": lambda b: rig.rover_set_mode(str(b.get("mode", "ackermann"))),
        "/api/rover/light": lambda b: rig.rover_light(bool(b.get("on", False))),
        "/api/rover/drive": lambda b: rig.rover_drive(
            linear=num(b.get("linear")), angular=num(b.get("angular")),
            steer=num(b.get("steer"))),
        "/api/rover/stop": lambda b: rig.rover_stop(),

        "/api/servo/enable": lambda b: rig.servo_enable(),
        "/api/servo/disable": lambda b: rig.servo_disable(),
        "/api/servo/check": lambda b: {"state": rig.servo_check()},
        "/api/servo/fault_reset": lambda b: rig.servo_fault_reset(),
        "/api/servo/move": lambda b: rig.servo_move(
            integer(b.get("pulses")), velocity=integer(b.get("velocity", 10_000))),
        "/api/servo/raise": lambda b: rig.servo_raise(
            integer(b.get("pulses", 10_000)), velocity=integer(b.get("velocity", 10_000))),
        "/api/servo/lower": lambda b: rig.servo_lower(
            integer(b.get("pulses", 10_000)), velocity=integer(b.get("velocity", 10_000))),
        "/api/servo/jog": lambda b: rig.servo_jog(integer(b.get("velocity"))),
        "/api/servo/stop": lambda b: rig.servo_stop(),
        "/api/servo/estop": lambda b: rig.servo_estop(),
    }

    class Handler(BaseHTTPRequestHandler):
        server_version = "roverservo/0.1"

        # keep the console clean — comment out to see every request
        def log_message(self, fmt: str, *args) -> None:  # noqa: A003
            pass

        # -- helpers -------------------------------------------------------- #
        def _send_json(self, obj: object, code: int = 200) -> None:
            payload = json.dumps(obj).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)

        def _send_file(self, relpath: str) -> None:
            # prevent path traversal; only serve from the static dir
            safe = os.path.normpath(relpath).lstrip("/\\")
            full = os.path.join(_STATIC_DIR, safe)
            if not full.startswith(_STATIC_DIR) or not os.path.isfile(full):
                self._send_json({"error": "not found"}, 404)
                return
            ext = os.path.splitext(full)[1].lower()
            with open(full, "rb") as fh:
                body = fh.read()
            self.send_response(200)
            self.send_header("Content-Type", _CONTENT_TYPES.get(ext, "application/octet-stream"))
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        # -- verbs ---------------------------------------------------------- #
        def do_GET(self) -> None:  # noqa: N802
            path = self.path.split("?", 1)[0]
            if path == "/api/state":
                try:
                    self._send_json(rig.snapshot())
                except Exception as exc:  # noqa: BLE001
                    self._send_json({"error": str(exc)}, 500)
                return
            if path in ("/", "/index.html"):
                self._send_file("index.html")
                return
            self._send_file(path)

        def do_POST(self) -> None:  # noqa: N802
            path = self.path.split("?", 1)[0]
            handler = routes.get(path)
            if handler is None:
                self._send_json({"error": f"unknown endpoint {path}"}, 404)
                return
            body: dict = {}
            length = int(self.headers.get("Content-Length", 0) or 0)
            if length:
                raw = self.rfile.read(length)
                try:
                    parsed = json.loads(raw or b"{}")
                    if isinstance(parsed, dict):
                        body = parsed
                except json.JSONDecodeError:
                    self._send_json({"error": "invalid JSON body"}, 400)
                    return
            try:
                result = handler(body)
                reply = {"ok": True}
                if isinstance(result, dict):
                    reply.update(result)
                self._send_json(reply)
            except Exception as exc:  # noqa: BLE001 — report to the UI, keep serving
                self._send_json({"ok": False, "error": str(exc)}, 400)

    return Handler


def serve(rig: Rover, host: str = "0.0.0.0", port: int = 8080) -> None:
    """Run the web server until Ctrl-C. Does not itself connect ``rig``."""
    httpd = ThreadingHTTPServer((host, port), make_handler(rig))
    shown_host = "localhost" if host in ("0.0.0.0", "") else host
    print(f"\n  roverservo dashboard:  http://{shown_host}:{port}\n"
          f"  (serving on {host}:{port} — Ctrl-C to stop)\n")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down...")
    finally:
        httpd.shutdown()

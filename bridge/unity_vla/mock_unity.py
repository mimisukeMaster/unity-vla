"""A stand-in for Unity's SimServer, speaking the same protocol.

Used by the test-suite and to try the bridge without Unity:
    uv run python -m unity_vla.mock_unity --port 5555
Joints follow their targets with a first-order lag; images are synthetic.
"""

from __future__ import annotations

import argparse
import json
import socket
import threading
from pathlib import Path

import numpy as np

from . import protocol
from .protocol import Message

REPO = Path(__file__).resolve().parents[2]


class MockSim:
    def __init__(self, width=320, height=180):
        self.w, self.h = width, height
        self.q = np.zeros(7)
        self.g = 0.0
        self.target = np.zeros(8)
        self.t = 0.0
        self.control_dt = 1 / 15
        self.seed = 0
        self.goal = {}
        self._fk = None

    def handle(self, msg: Message) -> Message:
        h = msg.header
        cmd = h.get("cmd")
        if cmd == "hello":
            return Message({
                "ok": True, "protocol": protocol.PROTOCOL_VERSION, "unity_version": "mock",
                "joint_names": [f"joint{i}" for i in range(1, 8)],
                "cameras": {"exterior": [self.w, self.h], "wrist": [self.w, self.h]},
            })
        if cmd == "reset":
            self.seed = int(h["seed"])
            self.control_dt = float(h["control_dt"])
            home = np.asarray(h.get("home_qpos") or np.zeros(7), float)
            self.q, self.g, self.t = home.copy(), 0.0, 0.0
            self.target = np.concatenate([home, [0.0]])
            self.goal = h.get("scene", {}).get("goal", {})
            return Message({"ok": True})
        if cmd == "step":
            a = np.asarray(h["action"], float)
            self.target = a
            alpha = 0.5
            self.q += alpha * (a[:7] - self.q)
            self.g += alpha * (a[7] - self.g)
            self.t += self.control_dt
            return Message({"ok": True, "sim_time": self.t})
        if cmd == "observe":
            return Message(
                {"ok": True, "joint_position": self.q.tolist(), "gripper_position": self.g,
                 "success": False, "sim_time": self.t, "task": self.goal},
                {"exterior": self._image(0), "wrist": self._image(1)},
            )
        if cmd == "set_joints":
            self.q = np.asarray(h["qpos"], float)
            self.g = float(h.get("gripper", 0.0))
            return Message({"ok": True})
        if cmd == "link_poses":
            return Message({"ok": True, "links": self._ref_links()})
        if cmd == "close":
            return Message({"ok": True})
        return Message({"ok": False, "error": f"unknown command {cmd}"})

    def _image(self, k):
        y = np.linspace(0, 255, self.h, dtype=np.float32)[:, None]
        x = np.linspace(0, 255, self.w, dtype=np.float32)[None, :]
        r = (x + 40 * np.sin(self.q[0] + k)) % 256
        g = (y + 40 * np.sin(self.q[1])) % 256
        b = np.full_like(r * g, 255 * self.g)
        return np.stack([r + 0 * g, g + 0 * r, b], -1).astype(np.uint8)

    def _ref_links(self):
        if self._fk is None:
            self._fk = json.loads((REPO / "tools" / "fk_reference.json").read_text())["cases"]
        best = min(self._fk, key=lambda c: float(np.sum((np.asarray(c["q"]) - self.q) ** 2)))
        return best["links"]


def serve(port: int, stop: threading.Event | None = None, ready: threading.Event | None = None):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(1)
    srv.settimeout(0.2)
    if ready:
        ready.set()
    sim = MockSim()
    try:
        while not (stop and stop.is_set()):
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            with conn:
                conn.settimeout(5.0)
                while True:
                    try:
                        msg = protocol.recv(conn)
                    except (ConnectionError, socket.timeout, OSError):
                        break
                    protocol.send(conn, sim.handle(msg))
                    if msg.header.get("cmd") == "close":
                        break
    finally:
        srv.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5555)
    serve(ap.parse_args().port)


if __name__ == "__main__":
    main()

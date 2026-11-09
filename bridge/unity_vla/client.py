"""TCP client for Unity's SimServer (lockstep request/response)."""

from __future__ import annotations

import logging
import socket
import time

import numpy as np

from . import protocol
from .protocol import Message

log = logging.getLogger(__name__)


class UnityError(RuntimeError):
    """Unity answered with ok=false."""


class UnityClient:
    def __init__(self, host: str = "127.0.0.1", port: int = 5555, timeout_s: float = 30.0):
        self.host, self.port, self.timeout_s = host, port, timeout_s
        self.sock: socket.socket | None = None
        self.info: dict = {}

    # -- connection -------------------------------------------------------------------
    def connect(self, retries: int = 60, wait_s: float = 1.0) -> dict:
        last = None
        for _ in range(retries):
            try:
                s = socket.create_connection((self.host, self.port), timeout=self.timeout_s)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.sock = s
                self.info = self.request("hello", protocol=protocol.PROTOCOL_VERSION).header
                if self.info.get("protocol") != protocol.PROTOCOL_VERSION:
                    raise UnityError(f"protocol mismatch: unity={self.info.get('protocol')}")
                log.info("connected to Unity %s", self.info.get("unity_version"))
                return self.info
            except (ConnectionRefusedError, socket.timeout, OSError) as e:
                last = e
                time.sleep(wait_s)
        raise ConnectionError(f"could not reach Unity at {self.host}:{self.port}: {last}")

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.request("close")
            except Exception:
                pass
            self.sock.close()
            self.sock = None

    def __enter__(self):
        if self.sock is None:
            self.connect()
        return self

    def __exit__(self, *exc):
        self.close()

    # -- requests ---------------------------------------------------------------------
    def request(self, cmd: str, blobs: dict[str, np.ndarray] | None = None, **fields) -> Message:
        if self.sock is None:
            raise ConnectionError("not connected")
        protocol.send(self.sock, Message({"cmd": cmd, **fields}, blobs or {}))
        reply = protocol.recv(self.sock)
        if not reply.header.get("ok", False):
            raise UnityError(f"{cmd}: {reply.header.get('error', 'unknown error')}")
        return reply

    def reset(self, seed: int, scene: dict, control_dt: float, physics_dt: float, home_qpos=None) -> dict:
        fields = dict(seed=int(seed), scene=scene, control_dt=control_dt, physics_dt=physics_dt)
        if home_qpos is not None:
            fields["home_qpos"] = [float(x) for x in home_qpos]
        self.request("reset", **fields)
        return self.observe()

    def step(self, action: np.ndarray) -> float:
        a = np.asarray(action, dtype=np.float64).reshape(-1)
        if a.shape != (8,):
            raise ValueError(f"action must have 8 values, got {a.shape}")
        return float(self.request("step", action=[float(x) for x in a]).header["sim_time"])

    def observe(self) -> dict:
        r = self.request("observe")
        h = r.header
        return {
            "joint_position": np.asarray(h["joint_position"], np.float32),
            "gripper_position": np.asarray([h["gripper_position"]], np.float32),
            "exterior_image": r.blobs["exterior"],
            "wrist_image": r.blobs["wrist"],
            "success": bool(h.get("success", False)),
            "sim_time": float(h.get("sim_time", 0.0)),
            "task": h.get("task", {}),
        }

    def set_joints(self, qpos, gripper: float = 0.0) -> None:
        """Teleport the arm (no dynamics).  Used for FK checks and replays."""
        self.request("set_joints", qpos=[float(x) for x in qpos], gripper=float(gripper))

    def link_poses(self) -> dict[str, dict]:
        """World pose of every robot link in the robot-root frame (Unity coordinates)."""
        return self.request("link_poses").header["links"]

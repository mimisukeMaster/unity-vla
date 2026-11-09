"""GPU-free policies for testing the Unity side (Phase 3)."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .base import Policy


class HoldPolicy(Policy):
    """Keeps the current joint positions (tests drive stiffness / gravity sag)."""

    name = "hold"

    def __init__(self, chunk: int = 10):
        self.chunk = chunk

    def infer(self, obs):
        a = np.concatenate([obs["joint_position"], obs["gripper_position"]]).astype(np.float32)
        return np.tile(a, (self.chunk, 1))


class SinePolicy(Policy):
    """Sinusoid around a center pose on every joint; gripper opens/closes.
    Deterministic in sim time so two runs with the same seed must match exactly."""

    name = "sine"

    def __init__(self, center, amplitude=0.25, period_s=4.0, control_dt=1 / 15, chunk=10):
        self.center = np.asarray(center, np.float32)
        self.amp = float(amplitude)
        self.period = float(period_s)
        self.dt = float(control_dt)
        self.chunk = chunk

    def reset(self, prompt):
        super().reset(prompt)
        self.k = 0

    def infer(self, obs):
        out = []
        for i in range(self.chunk):
            t = (self.k + i + 1) * self.dt
            phase = 2 * np.pi * t / self.period
            q = self.center + self.amp * np.sin(phase + np.arange(7) * 0.7)
            g = 0.5 + 0.5 * np.sin(phase)
            out.append(np.concatenate([q, [g]]))
        self.k += self.chunk
        return np.asarray(out, np.float32)


class ReplayPolicy(Policy):
    """Replays the actions recorded in an episode .npz (key 'actions', shape (T, 8))."""

    name = "replay"

    def __init__(self, path: str, chunk=10):
        self.actions = np.load(Path(path))["actions"].astype(np.float32)
        self.chunk = chunk

    def reset(self, prompt):
        super().reset(prompt)
        self.k = 0

    def infer(self, obs):
        if self.k >= len(self.actions):
            return np.tile(self.actions[-1], (self.chunk, 1))
        part = self.actions[self.k : self.k + self.chunk]
        self.k += len(part)
        return part


def make_dummy(cfg: dict) -> Policy:
    mode = cfg.get("mode", "sine")
    chunk = int(cfg.get("open_loop_horizon", 10))
    if mode == "hold":
        return HoldPolicy(chunk)
    if mode == "sine":
        return SinePolicy(
            center=cfg["center"],
            amplitude=cfg.get("amplitude", 0.25),
            period_s=cfg.get("period_s", 4.0),
            control_dt=cfg.get("control_dt", 1 / 15),
            chunk=chunk,
        )
    if mode == "replay":
        return ReplayPolicy(cfg["path"], chunk)
    raise ValueError(f"unknown dummy mode {mode}")

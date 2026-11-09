"""Per-episode outputs: side-by-side mp4, npz of observations/actions, result json."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np


class EpisodeRecorder:
    def __init__(self, out_dir: Path, video: bool = True, fps: int = 15):
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.video, self.fps = video, fps
        self.frames: list[np.ndarray] = []
        self.joints, self.grippers, self.actions, self.times = [], [], [], []
        self.infer_latency: list[float] = []

    def add_obs(self, obs: dict):
        self.joints.append(obs["joint_position"].copy())
        self.grippers.append(obs["gripper_position"].copy())
        self.times.append(obs["sim_time"])
        if self.video:
            self.frames.append(side_by_side(obs["exterior_image"], obs["wrist_image"]))

    def add_action(self, a):
        self.actions.append(np.asarray(a, np.float32))

    def finish(self, result: dict) -> dict:
        np.savez_compressed(
            self.out_dir / "episode.npz",
            joint_position=np.asarray(self.joints, np.float32),
            gripper_position=np.asarray(self.grippers, np.float32),
            actions=np.asarray(self.actions, np.float32).reshape(-1, 8),
            sim_time=np.asarray(self.times, np.float64),
            infer_latency_s=np.asarray(self.infer_latency, np.float64),
        )
        if self.video and self.frames:
            import imageio.v2 as imageio

            imageio.mimwrite(self.out_dir / "episode.mp4", self.frames, fps=self.fps, macro_block_size=1)
        (self.out_dir / "result.json").write_text(json.dumps(result, indent=2, ensure_ascii=False))
        return result


def side_by_side(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    h = max(a.shape[0], b.shape[0])

    def pad(x):
        if x.shape[0] == h:
            return x
        out = np.zeros((h, x.shape[1], 3), np.uint8)
        out[: x.shape[0]] = x
        return out

    return np.concatenate([pad(a), pad(b)], axis=1)

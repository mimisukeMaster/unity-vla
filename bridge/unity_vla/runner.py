"""Episode loop: reset -> (observe -> infer -> step x H) until success or max_steps."""

from __future__ import annotations

import datetime as dt
import json
import logging
import platform
import time
from pathlib import Path

import numpy as np

from .client import UnityClient
from .policies import Policy
from .recorder import EpisodeRecorder

log = logging.getLogger(__name__)


def postprocess_action(a: np.ndarray, cfg: dict) -> np.ndarray:
    a = np.asarray(a, np.float32).copy()
    g = cfg.get("gripper", {})
    if g.get("binarize", True):
        a[7] = 1.0 if a[7] > g.get("threshold", 0.5) else 0.0
    a[7] = float(np.clip(a[7], 0.0, 1.0))
    return a


def run_episode(
    unity: UnityClient,
    policy: Policy,
    cfg: dict,
    seed: int,
    prompt: str,
    scene: dict,
    out_dir: Path | None,
) -> dict:
    sim = cfg.get("sim", {})
    control_dt = 1.0 / float(sim.get("control_hz", 15))
    physics_dt = float(sim.get("physics_dt", 0.002))
    horizon = int(cfg["policy"].get("open_loop_horizon", 10))
    max_steps = int(cfg.get("task", {}).get("max_steps", 450))

    rec = EpisodeRecorder(out_dir, video=cfg.get("record", {}).get("video", True)) if out_dir else None
    obs = unity.reset(seed, scene, control_dt, physics_dt, cfg.get("task", {}).get("home_qpos"))
    policy.reset(prompt)

    t_start = time.perf_counter()
    steps, error = 0, None
    success = False
    try:
        while steps < max_steps:
            if rec:
                rec.add_obs(obs)
            chunk = policy.infer(obs)
            if rec and getattr(policy, "last_latency_s", None) is not None:
                rec.infer_latency.append(policy.last_latency_s)
            for a in chunk[:horizon]:
                a = postprocess_action(a, cfg)
                unity.step(a)
                if rec:
                    rec.add_action(a)
                steps += 1
                if steps >= max_steps:
                    break
            obs = unity.observe()
            # Unity reports success only after the condition held for task.hold_s of sim time
            if obs["success"]:
                success = True
                break
    except Exception as e:  # recorded as an error episode, the caller moves on
        log.exception("episode failed")
        error = f"{type(e).__name__}: {e}"

    result = {
        "seed": seed,
        "prompt": prompt,
        "scene": scene,
        "success": success,
        "error": error,
        "steps": steps,
        "sim_time_s": round(steps * control_dt, 3),
        "wall_time_s": round(time.perf_counter() - t_start, 2),
        "policy": policy.name,
        "mean_infer_latency_s": float(np.mean(rec.infer_latency)) if rec and rec.infer_latency else None,
    }
    if rec:
        if obs is not None:
            rec.add_obs(obs)
        rec.finish(result)
    return result


def make_run_dir(root: str | Path, tag: str) -> Path:
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    d = Path(root) / f"{stamp}_{tag}"
    d.mkdir(parents=True, exist_ok=True)
    return d


def write_run_info(run_dir: Path, cfg: dict, unity_info: dict, policy: Policy):
    info = {
        "config": cfg,
        "unity": unity_info,
        "policy_server_metadata": getattr(policy, "server_metadata", None),
        "python": platform.python_version(),
        "platform": platform.platform(),
    }
    (run_dir / "run_info.json").write_text(json.dumps(info, indent=2, ensure_ascii=False, default=str))

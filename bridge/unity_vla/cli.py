"""unity-vla command line.

  unity-vla ping                         Unity と接続できるか確認
  unity-vla check-fk                     Unity の順運動学を参照値と比較 (M1)
  unity-vla run   -c configs/xxx.yaml    1 エピソード (または seeds 分) 実行
  unity-vla eval  -c configs/eval_*.yaml seeds x prompts で成功率を集計 (M4)
  unity-vla policy-check -c ...          openpi サーバー単体の推論確認 (M0)
  unity-vla compare A.npz B.npz          2 エピソードの関節軌跡が一致するか (M2)
"""

from __future__ import annotations

import argparse
import copy
import json
import logging
import sys
import time
from pathlib import Path

import numpy as np
import yaml

from .client import UnityClient
from .policies import make_policy
from .runner import make_run_dir, run_episode, write_run_info

log = logging.getLogger("unity_vla")
REPO = Path(__file__).resolve().parents[2]


def load_config(path: str | None) -> dict:
    base = yaml.safe_load((REPO / "bridge" / "configs" / "default.yaml").read_text(encoding="utf-8"))
    if path:
        over = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        base = deep_merge(base, over)
    return base


def deep_merge(a: dict, b: dict) -> dict:
    out = copy.deepcopy(a)
    for k, v in b.items():
        out[k] = deep_merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


def connect_unity(cfg) -> UnityClient:
    u = cfg["unity"]
    c = UnityClient(u["host"], int(u["port"]), float(u.get("timeout_s", 30)))
    c.connect()
    return c


def policy_cfg(cfg) -> dict:
    p = dict(cfg["policy"])
    p.setdefault("control_dt", 1.0 / float(cfg["sim"]["control_hz"]))
    p.setdefault("center", cfg["task"]["home_qpos"])
    return p


# ----------------------------------------------------------------------------- commands
def cmd_ping(cfg, args):
    with connect_unity(cfg) as u:
        print(json.dumps(u.info, indent=2, ensure_ascii=False))


def cmd_check_fk(cfg, args):
    ref = json.loads((REPO / "tools" / "fk_reference.json").read_text())
    worst_pos, worst_name = 0.0, ""
    with connect_unity(cfg) as u:
        for case in ref["cases"]:
            u.set_joints(case["q"], 0.0)
            poses = u.link_poses()
            for name, exp in case["links"].items():
                if name not in poses:
                    continue
                err = float(np.linalg.norm(np.asarray(poses[name]["pos"]) - np.asarray(exp["pos"])))
                if err > worst_pos:
                    worst_pos, worst_name = err, name
    ok = worst_pos < 0.005
    print(f"cases: {len(ref['cases'])}  worst link error: {worst_pos*1000:.2f} mm ({worst_name})")
    print("M1 FK check:", "PASS" if ok else "FAIL (> 5 mm)")
    return 0 if ok else 1


def _episodes(cfg):
    task = cfg["task"]
    prompts = task.get("prompts") or [{"prompt": task["prompt"], "scene": task.get("scene", {})}]
    seeds = cfg.get("episodes", {}).get("seeds", [0])
    for p in prompts:
        scene = deep_merge(task.get("scene", {}), p.get("scene", {}))
        for s in seeds:
            yield p["prompt"], scene, int(s)


def cmd_run(cfg, args, eval_mode=False):
    tag = args.tag or (cfg["policy"].get("variant") or cfg["policy"]["type"])
    run_dir = make_run_dir(REPO / cfg["record"]["out_dir"], ("eval_" if eval_mode else "") + tag)
    policy = make_policy(policy_cfg(cfg))
    results = []
    with connect_unity(cfg) as u:
        write_run_info(run_dir, cfg, u.info, policy)
        for i, (prompt, scene, seed) in enumerate(_episodes(cfg)):
            ep_dir = run_dir / f"ep{i:03d}_seed{seed}"
            r = run_episode(u, policy, cfg, seed, prompt, scene, ep_dir)
            results.append(r)
            print(f"[{i:03d}] seed={seed} success={r['success']} steps={r['steps']} "
                  f"err={r['error']} prompt='{prompt}'")
    policy.close()
    summary = summarize(results)
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    print(json.dumps(summary["overall"], indent=2))
    print(f"outputs: {run_dir}")
    return 0


def summarize(results):
    by_prompt = {}
    for r in results:
        by_prompt.setdefault(r["prompt"], []).append(r)

    def agg(rs):
        n = len(rs)
        ok = sum(r["success"] for r in rs)
        errs = sum(r["error"] is not None for r in rs)
        return {"episodes": n, "successes": ok, "success_rate": round(ok / n, 3) if n else 0.0,
                "errors": errs, "mean_steps": round(float(np.mean([r["steps"] for r in rs])), 1) if n else 0}

    return {"overall": agg(results), "by_prompt": {k: agg(v) for k, v in by_prompt.items()}, "episodes": results}


def cmd_policy_check(cfg, args):
    """M0: dummy observation -> openpi server -> action chunk shape + latency."""
    p = policy_cfg(cfg)
    if p["type"] != "openpi":
        print("policy.type must be openpi for this check", file=sys.stderr)
        return 2
    policy = make_policy(p)
    policy.reset("pick up the red cube")
    rng = np.random.default_rng(0)
    obs = {
        "exterior_image": rng.integers(0, 255, (180, 320, 3), dtype=np.uint8),
        "wrist_image": rng.integers(0, 255, (180, 320, 3), dtype=np.uint8),
        "joint_position": np.asarray(cfg["task"]["home_qpos"], np.float32),
        "gripper_position": np.zeros(1, np.float32),
    }
    lat = []
    for _ in range(args.n):
        t = time.perf_counter()
        a = policy.infer(obs)
        lat.append(time.perf_counter() - t)
    print(f"server metadata: {policy.server_metadata}")
    print(f"action chunk shape: {a.shape}")
    print(f"first action: {np.round(a[0], 3)}")
    print(f"latency: first {lat[0]:.2f}s, median {np.median(lat):.3f}s over {len(lat)} calls")
    ok = a.shape[1] == 8 and a.shape[0] >= int(p.get("open_loop_horizon", 10))
    print("M0:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def cmd_compare(cfg, args):
    """M2 determinism: compare joint trajectories of two episode.npz files."""
    a, b = (np.load(p)["joint_position"] for p in args.paths)
    n = min(len(a), len(b))
    err = float(np.max(np.abs(a[:n] - b[:n]))) if n else float("inf")
    ok = len(a) == len(b) and err <= 1e-4
    print(f"steps {len(a)} vs {len(b)}, max joint diff {err:.2e} rad ->", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main(argv=None):
    ap = argparse.ArgumentParser(prog="unity-vla")
    ap.add_argument("command", choices=["ping", "check-fk", "run", "eval", "policy-check", "compare"])
    ap.add_argument("paths", nargs="*", help="compare: two episode.npz files")
    ap.add_argument("-c", "--config", help="YAML merged over configs/default.yaml")
    ap.add_argument("--tag", help="run directory suffix")
    ap.add_argument("-n", type=int, default=5, help="policy-check: number of calls")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    cfg = load_config(args.config)
    fn = {
        "ping": cmd_ping,
        "check-fk": cmd_check_fk,
        "run": cmd_run,
        "eval": lambda c, a: cmd_run(c, a, eval_mode=True),
        "policy-check": cmd_policy_check,
        "compare": cmd_compare,
    }[args.command]
    sys.exit(fn(cfg, args) or 0)


if __name__ == "__main__":
    main()

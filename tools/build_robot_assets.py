"""Convert mujoco_menagerie Franka Panda + Robotiq 2F-85 into Unity-ready assets.

Outputs (all geometry already in Unity's coordinate convention):
  unity/Assets/UnityVLA/Robot/Meshes/*.bytes      binary meshes (see write_mesh)
  unity/Assets/UnityVLA/Robot/robot_manifest.json links, joints, visuals, gripper tables
  unity/Assets/UnityVLA/Tests/EditMode/fk_reference.json  FK ground truth for tests

Coordinate conversion
  MuJoCo/ROS: right-handed, Z up, X forward.  Unity: left-handed, Y up, Z forward.
  v_unity = C @ v_ros with C = [[0,-1,0],[0,0,1],[1,0,0]] (det = -1, a reflection).
  A rotation R becomes C R C^T.  A revolute joint about axis a by +theta (right-handed)
  becomes Unity Quaternion.AngleAxis(-theta, C a)  -> manifest stores axis = C a, sign = -1.

Usage:
  uv run tools/build_robot_assets.py --menagerie ../ext/mujoco_menagerie
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh

C = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])

# DROID reset pose (rad).  Verified against DROID in Phase 0 (docs/droid_contract.md).
DROID_HOME = [0.0, -math.pi / 5, 0.0, -4 * math.pi / 5, 0.0, 3 * math.pi / 5, 0.0]

GRIPPER_MAX_DRIVER = 0.8  # rad, 2F-85 driver joint range in menagerie
GRIPPER_SAMPLES = 21


# ----------------------------------------------------------------------------- math
def quat_wxyz_to_mat(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def mat_to_quat_xyzw(m):
    """Rotation matrix -> quaternion (x, y, z, w), Unity order."""
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    if q[3] < 0:
        q = -q
    return q / np.linalg.norm(q)


def rod(axis, theta):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(theta) * k + (1 - math.cos(theta)) * k @ k


def to_unity_vec(v):
    return C @ np.asarray(v, float)


def to_unity_rot(r):
    return C @ r @ C.T


def tf(pos=(0, 0, 0), rot=None):
    t = np.eye(4)
    t[:3, 3] = pos
    if rot is not None:
        t[:3, :3] = rot
    return t


def r3(v, nd=7):
    return [round(float(x), nd) for x in v]


# ----------------------------------------------------------------------------- MJCF
def parse_vec(s, default):
    return np.array([float(x) for x in s.split()]) if s else np.array(default, float)


def body_local(b):
    pos = parse_vec(b.get("pos"), [0, 0, 0])
    quat = parse_vec(b.get("quat"), [1, 0, 0, 0])
    return pos, quat_wxyz_to_mat(quat)


# ----------------------------------------------------------------------------- meshes
def write_mesh(mesh: trimesh.Trimesh, path: Path, scale=1.0):
    """Binary format 'UVM1':
    char[4] 'UVM1', uint32 nVerts, uint32 nIndices,
    float32[3n] positions (Unity coords), float32[3n] normals, int32[nIndices] triangles.
    Triangle winding is left as-is; the Unity builder checks orientation against a
    Unity primitive and flips if needed (reflection C keeps apparent winding correct)."""
    v = np.asarray(mesh.vertices, float) * scale
    vu = (C @ v.T).T.astype(np.float32)
    n = np.asarray(mesh.vertex_normals, float)
    nu = (C @ n.T).T.astype(np.float32)
    idx = np.asarray(mesh.faces, np.int32).reshape(-1)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as f:
        f.write(b"UVM1")
        f.write(struct.pack("<II", len(vu), len(idx)))
        f.write(vu.tobytes())
        f.write(nu.tobytes())
        f.write(idx.tobytes())
    return len(vu), len(idx) // 3


def load_mesh(path: Path) -> trimesh.Trimesh:
    m = trimesh.load(path, force="mesh", process=True)
    if not isinstance(m, trimesh.Trimesh):
        raise RuntimeError(f"could not load {path}")
    return m


# ----------------------------------------------------------------------------- panda
def build_panda(mdir: Path, out_mesh: Path, report: list):
    xml = ET.parse(mdir / "franka_emika_panda" / "panda_nohand.xml").getroot()
    assets = mdir / "franka_emika_panda" / "assets"
    materials = {
        m.get("name"): [float(x) for x in m.get("rgba").split()]
        for m in xml.iter("material")
        if m.get("rgba") and m.get("name")
    }
    mesh_files = {}
    for m in xml.find("asset").iter("mesh"):
        f = m.get("file")
        mesh_files[m.get("name") or Path(f).stem] = f

    default_range = (-2.8973, 2.8973)
    links = []
    ros_chain = []  # for FK reference: (name, parent, T_local_ros, axis_ros or None, range)

    def walk(body, parent_name):
        name = body.get("name")
        if name == "attachment":
            pos, rot = body_local(body)
            ros_chain.append((name, parent_name, tf(pos, rot), None, None))
            links.append(
                {
                    "name": "attachment",
                    "parent": parent_name,
                    "pos": r3(to_unity_vec(pos)),
                    "rot": r3(mat_to_quat_xyzw(to_unity_rot(rot))),
                    "joint": None,
                    "visuals": [],
                    "colliders": [],
                }
            )
            return
        pos, rot = body_local(body)
        joint_el = body.find("joint")
        joint = None
        axis_ros = None
        rng = None
        if joint_el is not None:
            axis_ros = parse_vec(joint_el.get("axis"), [0, 0, 1])
            rng = tuple(parse_vec(joint_el.get("range"), default_range))
            joint = {
                "name": joint_el.get("name"),
                "type": "revolute",
                "axis": r3(to_unity_vec(axis_ros)),
                "sign": -1,
                "lower": rng[0],
                "upper": rng[1],
            }
        inertial = body.find("inertial")
        visuals, colliders = [], []
        for g in body.findall("geom"):
            mesh_name = g.get("mesh")
            fname = mesh_files[mesh_name]
            stem = "panda_" + Path(fname).stem
            mesh = load_mesh(assets / fname)
            nv, nt = write_mesh(mesh, out_mesh / f"{stem}.bytes")
            report.append((stem, nv, nt))
            if g.get("class") == "panda/visual":
                visuals.append(
                    {"mesh": stem, "color": materials.get(g.get("material"), [1, 1, 1, 1])}
                )
            else:
                colliders.append({"mesh": stem})
        links.append(
            {
                "name": name,
                "parent": parent_name,
                "pos": r3(to_unity_vec(pos)),
                "rot": r3(mat_to_quat_xyzw(to_unity_rot(rot))),
                "joint": joint,
                "mass": float(inertial.get("mass")) if inertial is not None else 0.1,
                "com": r3(to_unity_vec(parse_vec(inertial.get("pos"), [0, 0, 0])))
                if inertial is not None
                else [0, 0, 0],
                "visuals": visuals,
                "colliders": colliders,
            }
        )
        ros_chain.append((name, parent_name, tf(pos, rot), axis_ros, rng))
        for child in body.findall("body"):
            walk(child, name)

    walk(xml.find("worldbody").find("body"), None)
    return links, ros_chain


# ----------------------------------------------------------------------------- gripper
def build_gripper(mdir: Path, out_mesh: Path, report: list):
    """Gripper model for Unity.

    The real 2F-85 is a closed-loop linkage, which ArticulationBody cannot represent.
    Visual parts are animated kinematically from tables indexed by the gripper command
    g in [0, 1] (0 = open, 1 = closed, DROID convention).  Physics uses two box pads per
    finger, each driven by two prismatic joints (lateral + vertical) that follow the
    real pad trajectory."""
    gdir = mdir / "robotiq_2f85"
    xml = ET.parse(gdir / "2f85.xml").getroot()
    materials = {
        m.get("name"): [float(x) for x in m.get("rgba").split()]
        for m in xml.iter("material")
        if m.get("rgba") and m.get("name")
    }

    mount = xml.find("worldbody").find("body")  # base_mount
    base = mount.find("body")
    mount_pos, mount_rot = body_local(mount)
    base_pos, base_rot = body_local(base)
    # gripper "base" frame relative to the panda attachment frame (ROS)
    T_att_base = tf(mount_pos, mount_rot) @ tf(base_pos, base_rot)

    def mesh_entry(name, material):
        stem = "2f85_" + name
        p = out_mesh / f"{stem}.bytes"
        if not p.exists():
            mesh = load_mesh(gdir / "assets" / f"{name}.stl")
            nv, nt = write_mesh(mesh, p, scale=0.001)
            report.append((stem, nv, nt))
        return {"mesh": stem, "color": materials.get(material, [0.3, 0.3, 0.3, 1])}

    # static visuals
    T_base_mount = np.linalg.inv(tf(base_pos, base_rot))  # mount expressed in base frame
    static_visuals = [
        dict(mesh_entry("base_mount", "black"), **pose_fields(T_base_mount)),
        dict(mesh_entry("base", "black"), **pose_fields(np.eye(4))),
    ]
    collider_base = {"mesh": "2f85_base"}

    # joint angles as function of g (parallel-jaw approximation, see docs/robot_model.md)
    def joint_angles(g):
        q = GRIPPER_MAX_DRIVER * g
        return q

    def Rx(a):
        return rod([1, 0, 0], a)

    def find(parent, name):
        for b in parent.iter("body"):
            if b.get("name") == name:
                return b
        raise KeyError(name)

    sides = []
    for side in ("right", "left"):
        drv = find(base, f"{side}_driver")
        cpl = find(drv, f"{side}_coupler")
        spr = find(base, f"{side}_spring_link")
        fol = find(spr, f"{side}_follower")
        pad = find(fol, f"{side}_pad")
        T_drv0 = tf(*body_local(drv))
        T_cpl0 = tf(*body_local(cpl))
        T_spr0 = tf(*body_local(spr))
        T_fol0 = tf(*body_local(fol))
        T_pad0 = tf(*body_local(pad))

        # coupler aims at the follower origin; offset at g=0
        def coupler_rel(q):
            c = (T_drv0 @ tf(rot=Rx(q)) @ T_cpl0)[:3, 3]
            f = (T_spr0 @ tf(rot=Rx(q)) @ T_fol0)[:3, 3]
            c0 = (T_drv0 @ T_cpl0)[:3, 3]
            f0 = (T_spr0 @ T_fol0)[:3, 3]
            r, r0 = f - c, f0 - c0
            # express both in driver frame (rotation about x only in that plane)
            Rd = T_drv0[:3, :3]
            rl = Rd.T @ r
            rl0 = Rd.T @ r0
            phi = math.atan2(rl[2], rl[1]) - math.atan2(rl0[2], rl0[1])
            return phi - q

        tables = {k: [] for k in ("driver", "coupler", "spring", "follower")}
        pad_pos = []
        for i in range(GRIPPER_SAMPLES):
            g = i / (GRIPPER_SAMPLES - 1)
            q = joint_angles(g)
            T_d = T_drv0 @ tf(rot=Rx(q))
            T_c = T_cpl0 @ tf(rot=Rx(coupler_rel(q)))
            T_s = T_spr0 @ tf(rot=Rx(q))
            T_f = T_fol0 @ tf(rot=Rx(-q))
            for k, T in (("driver", T_d), ("coupler", T_c), ("spring", T_s), ("follower", T_f)):
                tables[k].append(pose_fields(T))
            T_pad_base = T_s @ T_f @ T_pad0
            pad_pos.append(T_pad_base)

        # pad physical trajectory in base frame (Unity), as displacement from g=0
        p0 = pad_pos[0][:3, 3]
        lat_axis_ros = np.array([0.0, 1.0, 0.0])  # base Y: opening direction
        up_axis_ros = np.array([0.0, 0.0, 1.0])  # base Z: along fingers
        lat = [float((T[:3, 3] - p0) @ lat_axis_ros) for T in pad_pos]
        up = [float((T[:3, 3] - p0) @ up_axis_ros) for T in pad_pos]
        # pad orientation is constant (parallel jaw); verify
        dev = max(
            np.degrees(np.arccos(np.clip((np.trace(T[:3, :3].T @ pad_pos[0][:3, :3]) - 1) / 2, -1, 1)))
            for T in pad_pos
        )
        assert dev < 1e-6, f"pad orientation drifts by {dev} deg"

        # collision boxes in pad frame (pad_box1/2 defaults)
        boxes = []
        for cls in ("pad_box1", "pad_box2"):
            d = xml.find(f".//default[@class='{cls}']/geom")
            bpos = parse_vec(d.get("pos"), [0, 0, 0])
            bsize = parse_vec(d.get("size"), [0, 0, 0]) * 2  # half-size -> full size
            boxes.append(
                {
                    "pos": r3(to_unity_vec(bpos)),
                    "size": r3(np.abs(C @ bsize)),
                    "friction": float(d.get("friction")),
                }
            )

        sides.append(
            {
                "side": side,
                "driver": {**mesh_entry("driver", "gray"), "table": tables["driver"]},
                "coupler": {**mesh_entry("coupler", "black"), "table": tables["coupler"]},
                "spring": {**mesh_entry("spring_link", "black"), "table": tables["spring"]},
                "follower": {**mesh_entry("follower", "black"), "table": tables["follower"]},
                "pad_visuals": [
                    {**mesh_entry("pad", "gray"), **pose_fields(T_pad0)},
                    {**mesh_entry("silicone_pad", "black"), **pose_fields(T_pad0)},
                ],
                "pad_rest": pose_fields(pad_pos[0]),
                "pad_lateral_axis": r3(to_unity_vec(lat_axis_ros)),
                "pad_vertical_axis": r3(to_unity_vec(up_axis_ros)),
                "pad_lateral": [round(x, 7) for x in lat],
                "pad_vertical": [round(x, 7) for x in up],
                "pad_boxes": boxes,
            }
        )

    pinch_ros = parse_vec(base.find("site").get("pos"), [0, 0, 0.145])
    return {
        "attach_to": "attachment",
        "base_pose": pose_fields(T_att_base),
        "mass": 0.9,
        "static_visuals": static_visuals,
        "collider": collider_base,
        "pinch": r3(to_unity_vec(pinch_ros)),
        "samples": GRIPPER_SAMPLES,
        "sides": sides,
        "stroke_m": round(abs(sides[0]["pad_lateral"][-1]) * 2, 5),
    }


def pose_fields(T):
    """4x4 ROS transform -> Unity local pos/rot fields."""
    return {
        "pos": r3(to_unity_vec(T[:3, 3])),
        "rot": r3(mat_to_quat_xyzw(to_unity_rot(T[:3, :3]))),
    }


# ----------------------------------------------------------------------------- FK reference
def fk_ros(chain, q):
    """World transforms (ROS) of each body for joint vector q (7)."""
    out = {}
    world = {None: np.eye(4)}
    qi = 0
    for name, parent, T_local, axis, _ in chain:
        T = world[parent] @ T_local
        if axis is not None:
            T = T @ tf(rot=rod(axis, q[qi]))
            qi += 1
        world[name] = T
        out[name] = T
    return out


# Unity-convention emulation: Hamilton quaternions, Quaternion.AngleAxis standard.
def q_mul(a, b):  # xyzw
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ]
    )


def q_rot(q, v):
    qv = np.array([v[0], v[1], v[2], 0.0])
    qc = np.array([-q[0], -q[1], -q[2], q[3]])
    return q_mul(q_mul(q, qv), qc)[:3]


def angle_axis(deg, axis):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    h = math.radians(deg) / 2
    return np.array([*(a * math.sin(h)), math.cos(h)])


def fk_unity_emulated(links, q):
    """Exactly what the C# KinematicPose does: localRot = rot * AngleAxis(sign*deg, axis)."""
    world = {None: (np.zeros(3), np.array([0, 0, 0, 1.0]))}
    qi = 0
    out = {}
    for L in links:
        pp, pr = world[L["parent"]]
        lr = np.array(L["rot"])
        if L["joint"] is not None:
            j = L["joint"]
            lr = q_mul(lr, angle_axis(j["sign"] * math.degrees(q[qi]), j["axis"]))
            qi += 1
        wp = pp + q_rot(pr, L["pos"])
        wr = q_mul(pr, lr)
        world[L["name"]] = (wp, wr)
        out[L["name"]] = (wp, wr)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--menagerie", type=Path, required=True)
    ap.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1])
    args = ap.parse_args()

    robot_dir = args.repo / "unity" / "Assets" / "UnityVLA" / "Resources" / "UnityVLA"
    mesh_dir = robot_dir / "Meshes"
    report = []
    links, chain = build_panda(args.menagerie, mesh_dir, report)
    gripper = build_gripper(args.menagerie, mesh_dir, report)

    manifest = {
        "format": "unity-vla-robot/1",
        "source": "google-deepmind/mujoco_menagerie (franka_emika_panda, robotiq_2f85)",
        "frame": "unity",
        "joint_names": [L["joint"]["name"] for L in links if L["joint"]],
        "home_qpos": DROID_HOME,
        "links": links,
        "gripper": gripper,
    }
    (robot_dir / "robot_manifest.json").write_text(json.dumps(manifest, indent=1))

    # FK reference + self-check of the Unity convention
    rng = np.random.default_rng(0)
    lims = [(L["joint"]["lower"], L["joint"]["upper"]) for L in links if L["joint"]]
    cases = []
    worst = 0.0
    for k in range(50):
        q = DROID_HOME if k == 0 else [rng.uniform(lo, hi) for lo, hi in lims]
        ros = fk_ros(chain, q)
        emu = fk_unity_emulated(links, q)
        entry = {"q": [float(x) for x in q], "links": {}}
        for name, T in ros.items():
            p_u = to_unity_vec(T[:3, 3])
            r_u = mat_to_quat_xyzw(to_unity_rot(T[:3, :3]))
            ep, er = emu[name]
            worst = max(worst, float(np.linalg.norm(ep - p_u)))
            dot = abs(float(np.dot(er / np.linalg.norm(er), r_u)))
            assert dot > 1 - 1e-10, f"rotation mismatch at {name}: {dot}"
            entry["links"][name] = {"pos": r3(p_u, 6), "rot": r3(r_u, 6)}
        cases.append(entry)
    assert worst < 1e-6, f"position mismatch {worst}"
    ref_path = args.repo / "unity" / "Assets" / "UnityVLA" / "Tests" / "EditMode" / "fk_reference.json"
    ref_path.write_text(json.dumps({"cases": cases}, indent=1))

    # python-side reference for bridge tests
    (args.repo / "tools" / "fk_reference.json").write_text(json.dumps({"cases": cases}))

    total_v = sum(r[1] for r in report)
    total_t = sum(r[2] for r in report)
    print(f"meshes: {len(report)}  verts: {total_v}  tris: {total_t}")
    print(f"gripper stroke: {gripper['stroke_m']*1000:.1f} mm")
    home = fk_ros(chain, DROID_HOME)["attachment"][:3, 3]
    print(f"flange at DROID home (ROS frame): {np.round(home, 4)}")
    print(f"Unity-convention self-check passed (max pos err {worst:.2e} m)")


if __name__ == "__main__":
    main()

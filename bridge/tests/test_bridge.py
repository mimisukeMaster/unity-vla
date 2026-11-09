import json
import socket
import threading

import numpy as np
import pytest

from unity_vla import protocol
from unity_vla.cli import load_config, main as cli_main, summarize
from unity_vla.client import UnityClient
from unity_vla.mock_unity import serve
from unity_vla.policies import make_policy
from unity_vla.policies.openpi_remote import build_droid_request
from unity_vla.runner import postprocess_action, run_episode


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


@pytest.fixture()
def mock_unity():
    port = free_port()
    stop, ready = threading.Event(), threading.Event()
    th = threading.Thread(target=serve, args=(port, stop, ready), daemon=True)
    th.start()
    ready.wait(5)
    yield port
    stop.set()
    th.join(2)


def test_protocol_roundtrip():
    img = np.arange(180 * 320 * 3, dtype=np.uint8).reshape(180, 320, 3)
    msg = protocol.Message({"cmd": "x", "v": [1, 2]}, {"img": img, "f": np.ones(3, np.float32)})
    raw = protocol.encode(msg)
    back = protocol.decode_body(raw[4:])
    assert back.header == {"cmd": "x", "v": [1, 2]}
    np.testing.assert_array_equal(back.blobs["img"], img)
    assert back.blobs["f"].dtype == np.float32


def test_droid_request_shapes():
    obs = {
        "exterior_image": np.zeros((180, 320, 3), np.uint8),
        "wrist_image": np.zeros((180, 320, 3), np.uint8),
        "joint_position": np.zeros(7, np.float32),
        "gripper_position": np.zeros(1, np.float32),
    }
    req = build_droid_request(obs, "pick up the red cube")
    assert req["observation/exterior_image_1_left"].shape == (224, 224, 3)
    assert req["observation/wrist_image_left"].dtype == np.uint8
    assert req["observation/joint_position"].shape == (7,)
    assert req["observation/gripper_position"].shape == (1,)
    assert set(req) == {
        "observation/exterior_image_1_left", "observation/wrist_image_left",
        "observation/joint_position", "observation/gripper_position", "prompt",
    }


def test_gripper_binarize():
    cfg = {"gripper": {"binarize": True, "threshold": 0.5}}
    assert postprocess_action(np.r_[np.zeros(7), 0.7], cfg)[7] == 1.0
    assert postprocess_action(np.r_[np.zeros(7), 0.2], cfg)[7] == 0.0
    assert postprocess_action(np.r_[np.zeros(7), 1.7], {"gripper": {"binarize": False}})[7] == 1.0


def _run(port, mode, tmp_path, name):
    cfg = load_config(None)
    cfg["unity"]["port"] = port
    cfg["policy"].update({"type": "dummy", "mode": mode, "control_dt": 1 / 15,
                          "center": cfg["task"]["home_qpos"]})
    cfg["task"]["max_steps"] = 60
    pol = make_policy(cfg["policy"])
    with UnityClient("127.0.0.1", port) as u:
        r = run_episode(u, pol, cfg, 0, "test", cfg["task"]["scene"], tmp_path / name)
    return r, np.load(tmp_path / name / "episode.npz")


def test_episode_hold(mock_unity, tmp_path):
    r, npz = _run(mock_unity, "hold", tmp_path, "hold")
    assert r["error"] is None and r["steps"] == 60
    assert npz["actions"].shape == (60, 8)
    assert (tmp_path / "hold" / "episode.mp4").exists()
    q = npz["joint_position"]
    assert np.allclose(q[0], q[-1], atol=1e-6)


def test_episode_sine_deterministic(mock_unity, tmp_path):
    _, a = _run(mock_unity, "sine", tmp_path, "a")
    _, b = _run(mock_unity, "sine", tmp_path, "b")
    np.testing.assert_array_equal(a["joint_position"], b["joint_position"])
    assert np.ptp(a["joint_position"][:, 0]) > 0.1


def test_check_fk_against_mock(mock_unity, monkeypatch, capsys, tmp_path):
    cfgp = tmp_path / "c.yaml"
    cfgp.write_text(f"unity: {{port: {mock_unity}}}\n")
    with pytest.raises(SystemExit) as e:
        cli_main(["check-fk", "-c", str(cfgp)])
    assert e.value.code == 0
    assert "PASS" in capsys.readouterr().out


def test_summary():
    rs = [{"prompt": "a", "success": True, "error": None, "steps": 10},
          {"prompt": "a", "success": False, "error": "x", "steps": 20}]
    s = summarize(rs)
    assert s["overall"]["success_rate"] == 0.5 and s["overall"]["errors"] == 1

"""openpi policy server client, DROID wire format.

Request layout is identical to Isaac Lab Arena's Pi0DroidAdapter (openpi commit c23745b):
both images resized with openpi_client.image_tools.resize_with_pad to 224x224,
joint_position (7,) rad, gripper_position (1,) in [0, 1] (0 = open).
The jointpos checkpoints return absolute joint positions + gripper position, (H, 8).
"""

from __future__ import annotations

import logging
import time

import numpy as np

from .base import Policy

log = logging.getLogger(__name__)


def build_droid_request(obs: dict, prompt: str, image_size=(224, 224)) -> dict:
    from openpi_client import image_tools

    h, w = image_size
    return {
        "observation/exterior_image_1_left": image_tools.resize_with_pad(obs["exterior_image"], h, w),
        "observation/wrist_image_left": image_tools.resize_with_pad(obs["wrist_image"], h, w),
        "observation/joint_position": np.asarray(obs["joint_position"], np.float32),
        "observation/gripper_position": np.asarray(obs["gripper_position"], np.float32).reshape(1),
        "prompt": prompt,
    }


class OpenPiRemotePolicy(Policy):
    name = "openpi"

    def __init__(self, host="localhost", port=8000, api_key=None, image_size=(224, 224)):
        from openpi_client import websocket_client_policy

        self.image_size = image_size
        log.info("connecting to openpi server %s:%s", host, port)
        self.client = websocket_client_policy.WebsocketClientPolicy(host=host, port=port, api_key=api_key)
        self.server_metadata = self.client.get_server_metadata()
        self.last_latency_s = None

    def infer(self, obs):
        req = build_droid_request(obs, self.prompt, self.image_size)
        t0 = time.perf_counter()
        out = self.client.infer(req)
        self.last_latency_s = time.perf_counter() - t0
        actions = np.asarray(out["actions"], np.float32)
        if actions.ndim != 2 or actions.shape[1] < 8:
            raise ValueError(f"unexpected action shape {actions.shape}")
        return actions[:, :8]

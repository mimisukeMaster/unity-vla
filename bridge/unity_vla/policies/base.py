from __future__ import annotations

import abc

import numpy as np


class Policy(abc.ABC):
    """obs dict (see UnityClient.observe) + prompt -> action chunk (H, 8)."""

    name = "policy"

    def reset(self, prompt: str) -> None:
        self.prompt = prompt

    @abc.abstractmethod
    def infer(self, obs: dict) -> np.ndarray: ...

    def close(self) -> None:
        pass

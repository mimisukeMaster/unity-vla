"""Policies: each returns an action chunk of shape (H, 8) = 7 absolute joint positions + gripper."""

from __future__ import annotations

from .base import Policy


def make_policy(cfg: dict) -> Policy:
    kind = cfg.get("type", "dummy")
    if kind == "dummy":
        from .dummy import make_dummy

        return make_dummy(cfg)
    if kind == "openpi":
        from .openpi_remote import OpenPiRemotePolicy

        return OpenPiRemotePolicy(
            host=cfg.get("host", "localhost"),
            port=cfg.get("port", 8000),
            api_key=cfg.get("api_key"),
            image_size=tuple(cfg.get("image_size", (224, 224))),
        )
    raise ValueError(f"unknown policy type: {kind}")


__all__ = ["Policy", "make_policy"]

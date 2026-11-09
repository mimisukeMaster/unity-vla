"""openpi policy server on Modal (serverless GPU, free monthly credits).

One-time:
    uv tool install modal          # isolated, nothing installed into your system Python
    modal setup                    # log in

Download the checkpoint into a Modal Volume (CPU only, so it does not burn GPU credit):
    modal run server/modal_app.py::download --variant pi0
    modal run server/modal_app.py::download --variant pi05

Start a server (prints a https://...modal.run URL; use it as wss://... in the bridge):
    modal serve server/modal_app.py      # dev mode, stops when you press Ctrl+C
    # or: modal deploy server/modal_app.py   (stays deployed; scales to zero when idle)

GPU: A10G (24 GB, bf16).  Containers shut down after SCALEDOWN_S seconds without traffic.
"""

from __future__ import annotations

import modal

OPENPI_COMMIT = "c23745b5ad24e98f66967ea795a07b2588ed6c79"  # same as Isaac Lab Arena
GPU = "A10G"
SCALEDOWN_S = 300
CACHE = "/cache"

VARIANTS = {
    # variant: (openpi train config, checkpoint dir)
    "pi0": ("pi0_droid_jointpos_polaris", "gs://openpi-assets-simeval/pi0_droid_jointpos"),
    "pi05": ("pi05_droid_jointpos_polaris", "gs://openpi-assets-simeval/pi05_droid_jointpos"),
}

image = (
    modal.Image.debian_slim(python_version="3.11")
    .apt_install("git", "git-lfs", "build-essential", "clang")
    .pip_install("uv")
    .run_commands(
        "git clone https://github.com/Physical-Intelligence/openpi.git /openpi",
        f"cd /openpi && git checkout {OPENPI_COMMIT} && git submodule update --init --recursive",
        "cd /openpi && GIT_LFS_SKIP_SMUDGE=1 uv sync --frozen",
        "cd /openpi && GIT_LFS_SKIP_SMUDGE=1 uv pip install -e .",
    )
    .env(
        {
            "OPENPI_DATA_HOME": f"{CACHE}/openpi",
            "XLA_PYTHON_CLIENT_MEM_FRACTION": "0.9",
        }
    )
)

volume = modal.Volume.from_name("unity-vla-openpi-cache", create_if_missing=True)
app = modal.App("unity-vla-openpi", image=image)


@app.function(volumes={CACHE: volume}, timeout=3600, cpu=4, memory=16384)
def download(variant: str = "pi0"):
    """Fetch the checkpoint into the volume (run once per variant)."""
    import subprocess

    _, ckpt = VARIANTS[variant]
    code = (
        "from openpi.shared import download; "
        f"p = download.maybe_download('{ckpt}'); print('cached at', p)"
    )
    subprocess.run(["/openpi/.venv/bin/python", "-c", code], check=True, cwd="/openpi")
    volume.commit()


def _serve(variant: str):
    import subprocess

    cfg, ckpt = VARIANTS[variant]
    subprocess.Popen(
        [
            "/openpi/.venv/bin/python", "scripts/serve_policy.py", "--port=8000",
            "policy:checkpoint", f"--policy.config={cfg}", f"--policy.dir={ckpt}",
        ],
        cwd="/openpi",
    )


@app.function(gpu=GPU, volumes={CACHE: volume}, timeout=4 * 3600, scaledown_window=SCALEDOWN_S, max_containers=1)
@modal.web_server(port=8000, startup_timeout=1200)
def serve_pi0():
    _serve("pi0")


@app.function(gpu=GPU, volumes={CACHE: volume}, timeout=4 * 3600, scaledown_window=SCALEDOWN_S, max_containers=1)
@modal.web_server(port=8000, startup_timeout=1200)
def serve_pi05():
    _serve("pi05")

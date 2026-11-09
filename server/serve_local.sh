#!/usr/bin/env bash
# openpi policy server in Docker on a machine with an NVIDIA GPU (>8 GB) and
# nvidia-container-toolkit.  Not usable on AMD/integrated GPUs.
#   ./server/serve_local.sh pi0      (or pi05)   -> ws://<host>:8000
set -euo pipefail
VARIANT="${1:-pi0}"
PORT="${PORT:-8000}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMMIT="$(tr -d '[:space:]' < "$HERE/OPENPI_COMMIT")"
case "$VARIANT" in
  pi0)  CFG=pi0_droid_jointpos_polaris;  DIR=gs://openpi-assets-simeval/pi0_droid_jointpos ;;
  pi05) CFG=pi05_droid_jointpos_polaris; DIR=gs://openpi-assets-simeval/pi05_droid_jointpos ;;
  *) echo "variant must be pi0 or pi05" >&2; exit 1 ;;
esac
IMAGE="unity-vla-openpi:${COMMIT:0:8}"
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
  GIT_LFS_SKIP_SMUDGE=1 git clone --filter=blob:none https://github.com/Physical-Intelligence/openpi "$TMP/openpi"
  (cd "$TMP/openpi" && git checkout "$COMMIT" && git submodule update --init --recursive)
  { cat "$TMP/openpi/scripts/docker/serve_policy.Dockerfile"; echo "COPY . /app"; } > "$TMP/Dockerfile"
  docker build -f "$TMP/Dockerfile" -t "$IMAGE" "$TMP/openpi"
fi
docker run --rm -it --gpus all -p "$PORT:8000" \
  -v "${HOME}/.cache/openpi:/openpi_assets" -e OPENPI_DATA_HOME=/openpi_assets \
  "$IMAGE" uv run scripts/serve_policy.py --port=8000 policy:checkpoint \
  --policy.config="$CFG" --policy.dir="$DIR"

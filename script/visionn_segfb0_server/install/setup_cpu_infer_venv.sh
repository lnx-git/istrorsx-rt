#!/usr/bin/env bash
# Setup CPU-only inference venv (ONNX Runtime) for testing the road-segmentation
# model on a machine without a GPU (e.g. a cloud VM), no retraining/TensorRT needed.
#
# Usage:
#   ./install/setup_cpu_infer_venv.sh
#
# Re-run safely on the same machine, or copy this whole repo to a fresh VM and
# run it there to reproduce the same environment.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
VENV_DIR="$PROJECT_ROOT/.venv_rosvm"

PY_MINOR="$(python3 -c 'import sys; print(sys.version_info.minor)')"
VENV_PKG="python3.${PY_MINOR}-venv"

echo "[1/3] Ensuring system package ${VENV_PKG} is installed..."
if ! dpkg -s "$VENV_PKG" >/dev/null 2>&1; then
    sudo apt-get update
    sudo apt-get install -y "$VENV_PKG"
else
    echo "  already installed."
fi

echo "[2/3] Creating venv at ${VENV_DIR} (if missing)..."
if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv "$VENV_DIR"
else
    echo "  already exists."
fi

echo "[3/3] Installing Python packages from requirements-cpu-infer.txt..."
"$VENV_DIR/bin/pip" install --upgrade pip
"$VENV_DIR/bin/pip" install -r "$SCRIPT_DIR/requirements-cpu-infer.txt"

echo
echo "Done. Activate with:"
echo "  source ${VENV_DIR}/bin/activate"
echo "Then run:"
echo "  python infer_image_rosvm.py"

#!/usr/bin/env bash
# One-shot environment setup for a fresh RunPod (or any Ubuntu 24.04 box).
#
# /workspace persists across pod restarts, but system (apt) and pip packages
# usually do NOT. Run this first thing after reopening the pod:
#
#     bash scripts/setup_env.sh
#
# Then verify:  python scripts/stand_test.py   (should print RESULT: PASS)
set -euo pipefail

echo "[setup] system GL libs for headless MuJoCo rendering (OSMesa)..."
if command -v sudo >/dev/null 2>&1; then SUDO=sudo; else SUDO=; fi
$SUDO apt-get update -qq
$SUDO apt-get install -y -qq libosmesa6 libgl1-mesa-dri libegl1 libegl-mesa0

echo "[setup] python deps..."
pip install --quiet -r "$(dirname "$0")/../requirements.txt"

echo "[setup] verifying imports + model load..."
python - <<'PY'
import mujoco, osqp, numpy, scipy
from pathlib import Path
scene = Path("assets/unitree_go2/go2_scene.xml")
m = mujoco.MjModel.from_xml_path(str(scene))
print(f"  mujoco {mujoco.__version__}  osqp {osqp.__version__}  numpy {numpy.__version__}")
print(f"  model OK: nq={m.nq} nv={m.nv} nu={m.nu} sensors={m.nsensor}")
PY

echo "[setup] done. Next: python scripts/stand_test.py"

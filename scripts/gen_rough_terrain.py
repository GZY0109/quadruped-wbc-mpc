"""One-off generator for the Phase 3(B) rough-terrain heightfield asset.

Produces assets/unitree_go2/rough_hfield.png: an 8-bit grayscale heightfield
(smoothed random noise, fixed seed) referenced by
assets/unitree_go2/go2_scene_rough.xml's <hfield> asset. Vendored into the
repo (like the Go2 MJCF itself) so the terrain is deterministic and
reproducible without re-running this script -- re-run only if the terrain
parameters below change.

Run: python scripts/gen_rough_terrain.py
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy.ndimage import gaussian_filter
import imageio.v2 as imageio

SEED = 0
RES = 64          # heightfield grid resolution (nrow=ncol=RES, must match the XML)
SMOOTH_SIGMA = 4.0  # gaussian smoothing (grid cells) -- higher = gentler bumps


def main():
    rng = np.random.default_rng(SEED)
    raw = rng.uniform(0.0, 1.0, size=(RES, RES))
    smooth = gaussian_filter(raw, sigma=SMOOTH_SIGMA, mode="wrap")
    smooth -= smooth.min()
    smooth /= smooth.max()  # normalize to [0, 1] -- XML's hfield size elevation_z scales this to meters

    img = (smooth * 255).astype(np.uint8)
    # lives under assets/unitree_go2/assets/ (not assets/unitree_go2/) because
    # go2.xml's <compiler meshdir="assets"/> is the search path MuJoCo uses
    # for hfield file= references too, resolved relative to go2.xml's own dir.
    out = Path(__file__).resolve().parents[1] / "assets" / "unitree_go2" / "assets" / "rough_hfield.png"
    imageio.imwrite(out, img)
    print(f"wrote {out} ({RES}x{RES}, seed={SEED}, sigma={SMOOTH_SIGMA})")


if __name__ == "__main__":
    main()

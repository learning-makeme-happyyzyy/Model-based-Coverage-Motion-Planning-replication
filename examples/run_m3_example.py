"""Minimal workpiece example for M3 visibility calculation.

Run from the repo root:

    uv run python -m examples.run_m3_example

Consumes M1 (S_bracket_1000.npz) and M2 (V_bracket_{200,300,400}.npz),
computes the visibility matrices A for all three candidate counts and
saves A_*.npz (+ .json) plus the 4-panel debug visualization for n=400.
"""

import time
from pathlib import Path

from src.m1_discretize import DiscretizedSurface
from src.m2_sample import CandidateViewpointSet
from src.m3_visibility import compute_visibility

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "data" / "outputs"
SURFACE = OUT / "S_bracket_1000.npz"


def main() -> None:
    surface = DiscretizedSurface.load(SURFACE)
    print("=== M3 Visibility Matrix - minimal workpiece example ===")
    print(f"Workpiece: L-bracket extrusion ({SURFACE.name}, |S| = {len(surface)} patches)")
    print()

    for n in [200, 300, 400]:
        viewpoints = CandidateViewpointSet.load(OUT / f"V_bracket_{n}.npz")
        t0 = time.time()
        A = compute_visibility(
            surface, viewpoints,
            source_surface=str(SURFACE), source_viewpoints=f"V_bracket_{n}.npz",
        )
        print(f"--- n = {n} (computed in {time.time() - t0:.1f} s) ---")
        print(A.format_report())
        stem = OUT / f"A_bracket_{n}"
        npz_path, json_path = A.save(stem)
        if n == 400:
            viz_path = A.visualize(stem.with_suffix(".png"), surface=surface,
                                   viewpoints=viewpoints)
        else:
            viz_path = None
        print(f"Artifacts: {npz_path.name} (+ {json_path.name})"
              + (f"  viz: {viz_path.name}" if viz_path else ""))
        print()


if __name__ == "__main__":
    main()

"""Minimal workpiece example for M4 travel cost computation.

Run from the repo root:

    uv run python -m examples.run_m4_example

Consumes M2 (V_bracket_{200,300,400}.npz), computes the travel-cost
matrices E and the home legs for all three candidate counts and saves
E_*.npz (+ .json) plus the 3-panel visualization for n=400. The M3
visibility matrix is loaded only to cross-check the row correspondence
(visibility is NOT part of the cost formula).
"""

import time
from pathlib import Path

import numpy as np

from src.m1_discretize import DiscretizedSurface
from src.m2_sample import CandidateViewpointSet
from src.m4_travel_cost import compute_travel_costs

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "data" / "outputs"


def main() -> None:
    surface = DiscretizedSurface.load(OUT / "S_bracket_1000.npz")
    print("=== M4 Travel Cost - minimal workpiece example ===")
    print(f"Workpiece: L-bracket extrusion; home pose = object center + (0.9, 0, 0.6) m")
    print()

    for n in [200, 300, 400]:
        viewpoints = CandidateViewpointSet.load(OUT / f"V_bracket_{n}.npz")
        t0 = time.time()
        costs = compute_travel_costs(
            viewpoints, source_viewpoints=f"V_bracket_{n}.npz"
        )
        print(f"--- n = {n} (computed in {time.time() - t0:.2f} s) ---")
        print(costs.format_report())
        stem = OUT / f"E_bracket_{n}"
        npz_path, json_path = costs.save(stem)
        if n == 400:
            viz_path = costs.visualize(stem.with_suffix(".png"), viewpoints=viewpoints)
        else:
            viz_path = None
        # cross-check with M3: A has exactly one row per candidate viewpoint
        A = np.load(OUT / f"A_bracket_{n}.npz")["A"]
        print(f"Cross-check: A.shape[0] = {A.shape[0]} == n = {n} "
              f"(row correspondence, visibility not used in the cost)")
        print(f"Artifacts: {npz_path.name} (+ {json_path.name})"
              + (f"  viz: {viz_path.name}" if viz_path else ""))
        print()


if __name__ == "__main__":
    main()

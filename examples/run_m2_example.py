"""Minimal workpiece example for M2 candidate viewpoint generation.

Run from the repo root:

    uv run python -m examples.run_m2_example

Consumes the M1 output of run_m1_example.py (data/outputs/S_bracket_1000.npz),
generates the n=400 master candidate set (seed 0) and saves n=200/300/400
files. Per the paper ("the same set of sampled candidate viewpoints are
used for the three different methods"), the smaller sets are prefixes of
the master set.

Also demonstrates the d_max tuning loop of paper05_replication.md §6.4:
the guide's initial d_max = 0.30 m is smaller than the minimum
viewpoint-to-surface distance of its own sampling shell (FOD_min +
margin = 0.45 m), so every neighborhood is empty; the diagnostic
recommends the smallest d_max with median >= 10 neighbors and the
candidate set is regenerated with it (same seed -> same positions,
Eq. 10 directions recomputed).
"""

from pathlib import Path

from src.m1_discretize import DiscretizedSurface
from src.m2_sample import (
    MIN_NEIGHBORS_TARGET,
    generate_candidates,
    recommend_d_max,
)

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "data" / "outputs"
SURFACE = OUT / "S_bracket_1000.npz"
SEED = 0
N_MASTER = 400
D_MAX_INITIAL = 0.30  # guide §6 initial value


def main() -> None:
    surface = DiscretizedSurface.load(SURFACE)

    # 1) run with the guide's initial d_max and show the §6.4 diagnostic
    print("=== M2 Candidate Viewpoint Generation - minimal workpiece example ===")
    print(f"Workpiece: L-bracket extrusion ({SURFACE.name}, |S| = {len(surface)} patches)")
    print()
    initial = generate_candidates(surface, n=N_MASTER, seed=SEED, d_max=D_MAX_INITIAL)
    print("--- run 1: guide initial d_max = 0.30 m ---")
    print(f"fallback (nearest-patch limit): {initial.meta['fallback_count']}/{N_MASTER}")
    print(f"median neighbors within d_max: {initial.meta['n_neighbors_median']} "
          f"(guide §6.4 target: >= {MIN_NEIGHBORS_TARGET})")

    # 2) tune d_max per guide §6.4 (diagnostic 1: median neighbor count >= 10)
    rec, _ = recommend_d_max(initial.positions, surface.centers)
    print(f"-> guide §6.4 tuning: smallest d_max with median neighbors >= "
          f"{MIN_NEIGHBORS_TARGET} is {rec:.3f} m")
    print(f"   (the guide's 0.30 m is inconsistent with its own sampling shell: "
          f"the minimum viewpoint-to-surface distance is FOD_min + margin = 0.45 m)")
    print()

    # 3) regenerate with the tuned d_max (same seed -> identical positions)
    tuned = generate_candidates(surface, n=N_MASTER, seed=SEED, d_max=rec)
    print("--- run 2: tuned d_max (guide §6.4) ---")
    print(tuned.format_report())

    # 4) save the master set and the shared n=200/300 prefixes
    stem = OUT / "V_bracket"
    for n in [400, 300, 200]:
        vps = tuned if n == N_MASTER else tuned.subset(n)
        npz_path, json_path = vps.save(f"{stem}_{n}")
        print(f"  saved: {npz_path.name} (+ {json_path.name})")
    viz_path = tuned.visualize(OUT / "V_bracket_400.png", surface=surface)
    print(f"  visualization: {viz_path}")


if __name__ == "__main__":
    main()

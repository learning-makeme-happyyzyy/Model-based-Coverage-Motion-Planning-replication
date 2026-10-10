"""Minimal workpiece example for M1 surface discretization.

Run from the repo root:

    uv run python -m examples.run_m1_example

Builds a watertight L-bracket (0.20 x 0.12 x 0.12 m, hand-calculable
surface area 0.1024 m^2), prepares it (bbox centered at the origin, the
project-wide convention from paper05_replication.md §4), discretizes it
into m = 1000 patches and prints the M1 validation report.
"""

from pathlib import Path

import numpy as np
import trimesh

from src.m1_discretize import discretize, prepare_mesh

ROOT = Path(__file__).resolve().parents[1]
MESHES = ROOT / "data" / "meshes"
OUT = ROOT / "data" / "outputs"
M = 1000


def build_bracket() -> trimesh.Trimesh:
    """L-profile extrusion; total surface area = 0.1024 m^2 (hand-calc).

    Caps:  2 x polygon area (0.2*0.04 + 0.06*0.08 = 0.0128)        = 0.0256
    Walls: perimeter (0.2+0.04+0.14+0.08+0.06+0.12 = 0.64) x 0.12  = 0.0768
    """
    pts = np.array(
        [[0, 0], [0.2, 0], [0.2, 0.04], [0.06, 0.04], [0.06, 0.12], [0, 0.12]],
        dtype=float,
    )
    tris = np.array([[0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 5]])
    return trimesh.creation.extrude_triangulation(pts, tris, height=0.12)


def main() -> None:
    MESHES.mkdir(parents=True, exist_ok=True)
    OUT.mkdir(parents=True, exist_ok=True)

    bracket = build_bracket()
    mesh_path = MESHES / "bracket.obj"
    bracket.export(mesh_path)
    bracket = prepare_mesh(bracket, center_origin=True)  # guide §4 convention

    surface = discretize(bracket, m=M, units="m")  # built at 0.2 m scale

    stem = OUT / f"S_bracket_{M}"
    npz_path, json_path = surface.save(stem)
    viz_path = surface.visualize(stem.with_suffix(".png"))

    print("=== M1 Surface Discretization - minimal workpiece example ===")
    print(f"Workpiece: L-bracket extrusion, {len(bracket.faces)} source faces "
          f"-> {len(surface)} patches")
    print()
    print(surface.format_report())
    print()
    print("Artifacts:")
    print(f"  mesh:         {mesh_path}")
    print(f"  patches:      {npz_path} (+ {json_path.name})")
    print(f"  visualization: {viz_path}")


if __name__ == "__main__":
    main()

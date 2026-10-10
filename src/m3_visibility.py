"""M3: visibility matrix A in {0,1}^{N x M} (paper_05 replication).

Follows paper05_replication.md §7.

Paper facts (verbatim from the PDF):
  * "A triangular surface patch is considered visible from a given
    viewpoint if all the following conditions are met for all of its
    vertices":
      1. "The vertices of the surface patch must be in the Field-Of-View
         (FOV) of viewpoint."  FOV: "30 deg and 21 deg in the horizontal
         and vertical directions, respectively" (Table I + text).
      2. "The vertices of the surface patch must be in the Field-Of-Depth
         (FOD) of viewpoint."  FOD: (0.40, 1.00) m (Table I + datasheet).
      3. "The viewing angle must be within the specified range."
         Table I: Viewing Angle 75 deg.
      4. "There is no occlusion in between the viewpoint and surface
         patch."
  * A is the n x m binary visibility matrix; A_ij = 1 iff patch j is
    visible from viewpoint i.
  * Aggregation (paper rule): a patch is visible iff ALL of its vertices
    satisfy the criteria (plus the patch-level viewing angle).

Documented interpretations / assumptions (paper ambiguities and
reproduction/engineering choices; see the final report and README):
  * Viewing angle (paper ambiguity): the paper never defines which angle
    the 75 deg applies to. The guide's §7.5 worked examples pin it down
    (head-on view = 0 deg and passes; camera at (0.6, 0, 0.1) over a +z
    patch ~= 80 deg and fails), so we use the angle between
    (viewpoint -> patch center) and the outward patch normal:

        theta = arccos(clip(dot(n_j, p_c - c_j) / ||p_c - c_j||, -1, 1))

    visible iff theta <= 75 deg (inclusive). NOTE: the guide's §7.1 code
    snippet (los = c_j - p_c; arccos(los . n_j)) has a sign slip -- with
    outward normals a head-on view would give 180 deg and fail; its own
    §7.5 examples confirm the sign used here. Back-facing patches
    (theta > 90 deg) fail automatically, as intended.
  * FOV is evaluated per vertex in the camera frame from M2
    (viewing_frames(): rows [x_c, y_c, z_c], z_c = optical axis):
    depth > 0, |x|/depth <= tan(15 deg), |y|/depth <= tan(10.5 deg),
    boundaries INCLUSIVE (guide §7.1 uses <=; the paper states no
    boundary convention). Under the M2 roll convention
    (x = normalize(cross(v_hat, z_world))), x is the horizontal
    direction: 30 deg horizontal on x, 21 deg vertical on y. The paper
    leaves roll free ("rotating a given viewpoint about its optical axis
    does not significantly alter ..."), so the rectangular-FOV
    orientation is an assumption.
  * FOD per vertex: 0.40 <= ||q - p_c|| <= 1.00 m, inclusive.
  * Occlusion: a ray is cast from the viewpoint to each patch vertex
    against the discretized workpiece mesh (built from the patch
    vertices themselves, so target rays terminate exactly on the
    surface). A hit occludes iff its distance along the ray is
    < ||q - p_c|| - eps with eps = 1e-3 m (guide §7.1, engineering
    choice, in mesh units). Hits at the target distance -- the target's
    own triangles, shared-vertex neighbors, coplanar grazing -- are
    self-hits and are ignored.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import time
from pathlib import Path
from typing import Optional, Union

import numpy as np
import trimesh

from src.m1_discretize import DiscretizedSurface, load_config
from src.m2_sample import CandidateViewpointSet

__all__ = [
    "VisibilityMatrix",
    "build_ray_mesh",
    "fov_check",
    "fod_check",
    "viewing_angle_check",
    "occlusion_check",
    "compute_visibility",
    "coverage_ratio",
    "main",
]

# guide §7.1: self-hit tolerance for ray casting (mesh units)
DEFAULT_RAY_EPS = 1e-3


# ---------------------------------------------------------------------------
# geometry helpers (pure functions, hand-testable)
# ---------------------------------------------------------------------------


def fov_check(
    rel: np.ndarray,
    frame: np.ndarray,
    fov_deg: tuple[float, float] = (30.0, 21.0),
) -> np.ndarray:
    """FOV criterion per point (paper: 30 deg horizontal, 21 deg vertical).

    `rel` (k,3): points minus camera position; `frame` (3,3): camera
    frame rows [x_c, y_c, z_c] (z_c = optical axis). A point is in the
    FOV iff it is in front (depth > 0) and its angular offset from the
    optical axis is within the half angles on both axes. Boundaries are
    inclusive (guide §7.1; the paper states no boundary convention).
    """
    x = rel @ frame[0]
    y = rel @ frame[1]
    z = rel @ frame[2]
    ok = (z > 0.0) & (np.abs(x) <= z * np.tan(np.radians(fov_deg[0] / 2.0)))
    ok &= np.abs(y) <= z * np.tan(np.radians(fov_deg[1] / 2.0))
    return ok


def fod_check(rel: np.ndarray, fod: tuple[float, float] = (0.40, 1.00)) -> np.ndarray:
    """FOD criterion per point (paper Table I): 0.40 <= ||q - p_c|| <= 1.00 m."""
    d = np.linalg.norm(rel, axis=1)
    return (d >= fod[0]) & (d <= fod[1])


def viewing_angle_check(
    positions: np.ndarray,
    centers: np.ndarray,
    normals: np.ndarray,
    max_deg: float = 75.0,
) -> np.ndarray:
    """Viewing-angle criterion (n,3)/(m,3)/(m,3) -> (n,m) bool.

    theta = angle between (viewpoint -> patch center) and the outward
    patch normal; visible iff theta <= max_deg (inclusive). Back-facing
    patches (theta > 90 deg) fail automatically. See the module
    docstring for the paper-ambiguity note.
    """
    v = positions[:, None, :] - centers[None, :, :]  # camera -> center
    d = np.linalg.norm(v, axis=2)
    cos = (v * normals[None, :, :]).sum(axis=-1) / d
    cos = np.clip(cos, -1.0, 1.0)
    return cos >= np.cos(np.radians(max_deg))


def build_ray_mesh(surface: DiscretizedSurface) -> trimesh.Trimesh:
    """The discretized workpiece mesh for occlusion rays, built from the
    patch vertices themselves (duplicate coordinates welded). Using the
    patch vertices guarantees that a ray aimed at a target vertex
    terminates exactly on the surface -- no chord/sagitta offset."""
    verts = surface.vertices.reshape(-1, 3)
    faces = np.arange(len(verts), dtype=np.int64).reshape(-1, 3)
    mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
    mesh.merge_vertices()
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.remove_unreferenced_vertices()
    return mesh


def occlusion_check(
    origins: np.ndarray,
    directions: np.ndarray,
    target_distances: np.ndarray,
    ray_mesh: trimesh.Trimesh,
    eps: float = DEFAULT_RAY_EPS,
) -> np.ndarray:
    """Occlusion per ray (k,3). True = occluded.

    A hit occludes iff its distance from the origin is smaller than
    (target distance - eps). The target's own triangles are hit exactly
    at the target distance (the target vertex is a vertex of the ray
    mesh), so they are self-hits and ignored; anything strictly closer
    by more than eps blocks the view.
    """
    inter = trimesh.ray.ray_triangle.RayMeshIntersector(ray_mesh)
    locations, index_ray, _ = inter.intersects_location(
        origins, directions, multiple_hits=True
    )
    occluded = np.zeros(len(origins), dtype=bool)
    if len(locations):
        hit_dist = np.linalg.norm(locations - origins[index_ray], axis=1)
        min_dist = np.full(len(origins), np.inf)
        np.minimum.at(min_dist, index_ray, hit_dist)
        occluded = min_dist < (np.asarray(target_distances) - eps)
    return occluded


def coverage_ratio(A: np.ndarray, selected_rows: np.ndarray) -> tuple[float, np.ndarray]:
    """Guide §7.3: fraction of patches seen by any of the selected
    viewpoints (patches are the counting unit, not area)."""
    selected = np.asarray(selected_rows)
    if len(selected) == 0:
        return 0.0, np.zeros(A.shape[1], dtype=bool)
    covered = A[selected].any(axis=0)
    return float(covered.mean()), covered


# ---------------------------------------------------------------------------
# matrix container
# ---------------------------------------------------------------------------


class VisibilityMatrix:
    """A in {0,1}^{N x M} plus provenance, statistics and diagnostics."""

    def __init__(self, A: np.ndarray, meta: dict):
        if A.ndim != 2 or A.dtype != bool:
            raise ValueError(f"A must be a 2-D bool array, got {A.shape} {A.dtype}")
        self.A = np.asarray(A, dtype=bool)
        self.meta = dict(meta)

    def __len__(self) -> int:
        return self.A.shape[0]

    @property
    def shape(self) -> tuple[int, int]:
        return self.A.shape

    def row_sum(self) -> np.ndarray:
        return self.A.sum(axis=1)

    def col_sum(self) -> np.ndarray:
        return self.A.sum(axis=0)

    def stats(self) -> dict:
        rs = self.row_sum()
        cs = self.col_sum()
        union_cov, _ = coverage_ratio(self.A, np.arange(len(self.A)))
        best_i = int(np.argmax(rs))
        best_cov, _ = coverage_ratio(self.A, np.array([best_i]))
        return {
            "shape": list(self.A.shape),
            "density": float(self.A.mean()),
            "row_sum": {"min": int(rs.min()), "median": float(np.median(rs)),
                        "max": int(rs.max()), "mean": float(rs.mean()),
                        "zero_rows": int((rs == 0).sum())},
            "col_sum": {"min": int(cs.min()), "median": float(np.median(cs)),
                        "max": int(cs.max()),
                        "zero_cols": int((cs == 0).sum()),
                        "uncovered_patches": int((cs == 0).sum())},
            "union_coverage_all_viewpoints": union_cov,
            "best_single_viewpoint": {"id": best_i, "coverage": best_cov,
                                      "patches": int(rs[best_i])},
        }

    def format_report(self) -> str:
        s = self.stats()
        m = self.meta
        lines = [
            f"Visibility matrix A: {s['shape'][0]} viewpoints x {s['shape'][1]} patches "
            f"(bool, density {100 * s['density']:.1f}%)",
            f"Source: {m.get('source_surface')} x {m.get('source_viewpoints')}",
            "",
            "Criteria (paper: all four, evaluated on ALL patch vertices):",
            f"  FOV 30 deg horizontal x 21 deg vertical (Table I; camera frame z = optical axis)",
            f"  FOD [0.40, 1.00] m per vertex (Table I)",
            f"  viewing angle <= 75 deg: angle((viewpoint -> patch center), outward normal)",
            f"  occlusion: ray per vertex, eps = {m.get('parameters', {}).get('ray_eps')} m",
            "",
            "Row statistics (patches per viewpoint):",
            f"  min {s['row_sum']['min']} / median {s['row_sum']['median']:.0f} / "
            f"max {s['row_sum']['max']} / mean {s['row_sum']['mean']:.1f}",
            f"  zero-visibility viewpoints: {s['row_sum']['zero_rows']}",
            "",
            "Column statistics (viewpoints per patch):",
            f"  min {s['col_sum']['min']} / median {s['col_sum']['median']:.0f} / "
            f"max {s['col_sum']['max']}",
            f"  zero-visibility patches: {s['col_sum']['zero_cols']}",
            "",
            "Coverage statistics:",
            f"  union coverage (all {s['shape'][0]} viewpoints): "
            f"{100 * s['union_coverage_all_viewpoints']:.1f}%",
            f"  best single viewpoint #{s['best_single_viewpoint']['id']}: "
            f"{100 * s['best_single_viewpoint']['coverage']:.1f}% "
            f"({s['best_single_viewpoint']['patches']} patches)",
            f"  elapsed: {m.get('elapsed_s'):.1f} s",
        ]
        return "\n".join(lines)

    def save(self, stem: Union[str, Path]) -> tuple[Path, Path]:
        stem = Path(stem)
        np.savez(stem.with_suffix(".npz"), A=self.A)
        meta = dict(self.meta)
        meta["stats"] = self.stats()
        meta_path = stem.with_suffix(".json")
        meta_path.write_text(json.dumps(meta, indent=2))
        return stem.with_suffix(".npz"), meta_path

    @classmethod
    def load(cls, stem: Union[str, Path]) -> "VisibilityMatrix":
        stem = Path(stem)
        data = np.load(stem.with_suffix(".npz"))
        meta = json.loads(stem.with_suffix(".json").read_text())
        return cls(data["A"], meta)

    def visualize(
        self,
        path: Union[str, Path],
        surface: Optional[DiscretizedSurface] = None,
        viewpoints: Optional[CandidateViewpointSet] = None,
        viewpoint_id: Optional[int] = None,
        patch_id: Optional[int] = None,
        show: bool = False,
    ) -> Path:
        """Debug view: heatmap, patch-coverage map, one viewpoint's
        visible patches, and the viewpoints that see one patch."""
        import matplotlib

        if not show:
            matplotlib.use("Agg", force=True)
        import matplotlib.pyplot as plt
        from matplotlib.colors import LinearSegmentedColormap
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection

        n, m = self.shape
        rs, cs = self.row_sum(), self.col_sum()
        if viewpoint_id is None:
            viewpoint_id = int(np.argmax(rs))
        if patch_id is None:
            patch_id = int(np.flatnonzero(cs == int(np.median(cs)))[0])
        blue = LinearSegmentedColormap.from_list("blue", ["#cde2fb", "#184f95"])

        fig = plt.figure(figsize=(11.5, 9.0))
        fig.suptitle(
            f"Visibility matrix A ({n} x {m}, density {100 * self.A.mean():.1f}%)",
            fontsize=10,
        )

        ax = fig.add_subplot(2, 2, 1)
        im = ax.imshow(self.A, aspect="auto", interpolation="none", cmap=blue)
        ax.set_xlabel("patch index j")
        ax.set_ylabel("viewpoint index i")
        ax.set_title(f"visibility heatmap (white = visible)\n"
                     f"{self.meta.get('source_viewpoints', '')}", fontsize=8)
        fig.colorbar(im, ax=ax, shrink=0.7)
        ax.tick_params(labelsize=7)

        ax2 = fig.add_subplot(2, 2, 2, projection="3d")
        if surface is not None:
            norm = plt.Normalize(cs.min(), cs.max())
            colors = blue(norm(cs))
            pc = Poly3DCollection(list(surface.vertices), facecolors=colors,
                                  edgecolors="none", alpha=0.9)
            ax2.add_collection3d(pc)
            ax2.set_box_aspect(surface.bounds[1] - surface.bounds[0])
        ax2.set_title(f"patches colored by col_sum (viewpoints per patch)\n"
                      f"zero-visibility patches: {(cs == 0).sum()}", fontsize=8)
        ax2.tick_params(labelsize=6)
        ax2.set_xlabel("x"); ax2.set_ylabel("y"); ax2.set_zlabel("z")

        ax3 = fig.add_subplot(2, 2, 3, projection="3d")
        if surface is not None:
            vis = self.A[viewpoint_id]
            colors3 = np.where(
                vis[:, None],
                np.asarray(blue(0.85))[None, :],
                np.array([[0.85, 0.86, 0.90, 1.0]]),
            )
            pc3 = Poly3DCollection(list(surface.vertices), facecolors=colors3,
                                   edgecolors="none", alpha=0.9)
            ax3.add_collection3d(pc3)
            ax3.set_box_aspect(surface.bounds[1] - surface.bounds[0])
            if viewpoints is not None:
                p = viewpoints.positions[viewpoint_id]
                v = viewpoints.viewing_directions[viewpoint_id]
                ax3.scatter(*p, color="#c92a2a", s=40)
                diag = np.linalg.norm(surface.bounds[1] - surface.bounds[0])
                ax3.quiver(*p, *v, length=0.15 * diag, color="#c92a2a")
        ax3.set_title(f"viewpoint #{viewpoint_id}: visible patches "
                      f"(red = camera)\n{int(self.A[viewpoint_id].sum())}/{m} patches",
                      fontsize=8)
        ax3.tick_params(labelsize=6)
        ax3.set_xlabel("x"); ax3.set_ylabel("y"); ax3.set_zlabel("z")

        ax4 = fig.add_subplot(2, 2, 4, projection="3d")
        if surface is not None and viewpoints is not None:
            see = self.A[:, patch_id]
            pc4 = Poly3DCollection(
                list(surface.vertices), facecolors="0.9", edgecolors="none", alpha=0.5
            )
            ax4.add_collection3d(pc4)
            # the patch itself, in red
            pc4b = Poly3DCollection(
                [surface.vertices[patch_id]], facecolors="#c92a2a", edgecolors="none", alpha=1.0
            )
            ax4.add_collection3d(pc4b)
            ax4.scatter(viewpoints.positions[see, 0], viewpoints.positions[see, 1],
                        viewpoints.positions[see, 2], color="#184f95", s=12)
            ax4.scatter(viewpoints.positions[~see, 0], viewpoints.positions[~see, 1],
                        viewpoints.positions[~see, 2], color="0.75", s=6)
            ax4.set_box_aspect(surface.bounds[1] - surface.bounds[0])
        ax4.set_title(f"patch #{patch_id}: seen by {int(self.A[:, patch_id].sum())}/{n} "
                      f"viewpoints\n(blue = sees it, gray = does not, red = patch)",
                      fontsize=8)
        ax4.tick_params(labelsize=6)
        ax4.set_xlabel("x"); ax4.set_ylabel("y"); ax4.set_zlabel("z")

        fig.tight_layout(rect=(0, 0, 1, 0.95))
        path = Path(path)
        fig.savefig(path, dpi=150, bbox_inches="tight")
        plt.close(fig)
        return path


# ---------------------------------------------------------------------------
# top-level M3 entry point
# ---------------------------------------------------------------------------


def compute_visibility(
    surface: DiscretizedSurface,
    viewpoints: CandidateViewpointSet,
    fod: tuple[float, float] = (0.40, 1.00),
    fov_deg: tuple[float, float] = (30.0, 21.0),
    max_view_angle_deg: float = 75.0,
    eps: float = DEFAULT_RAY_EPS,
    ray_mesh: Optional[trimesh.Trimesh] = None,
    source_surface: str = "in-memory",
    source_viewpoints: str = "in-memory",
) -> VisibilityMatrix:
    """M3 black box: (S, V, mesh) -> visibility matrix A in {0,1}^{N x M}.

    Per viewpoint: FOV/FOD are vectorized over the mesh vertices; rays
    are cast only for vertices that pass both (occlusion check); the
    patch-level result ANDs its three vertices' results with the
    viewing-angle criterion.
    """
    t0 = time.time()
    n, m = len(viewpoints), len(surface)
    if ray_mesh is None:
        ray_mesh = build_ray_mesh(surface)
    verts = np.asarray(ray_mesh.vertices, dtype=np.float64)
    faces = np.asarray(ray_mesh.faces)
    frames = viewpoints.viewing_frames()
    A = np.zeros((n, m), dtype=bool)

    for i in range(n):
        p = viewpoints.positions[i]
        rel = verts - p
        ok = fov_check(rel, frames[i], fov_deg) & fod_check(rel, fod)
        vert_ok = np.zeros(len(verts), dtype=bool)
        k = np.flatnonzero(ok)
        if len(k):
            occ = occlusion_check(
                np.broadcast_to(p, (len(k), 3)),
                verts[k] - p,
                np.linalg.norm(verts[k] - p, axis=1),
                ray_mesh,
                eps,
            )
            vert_ok[k] = ~occ
        patch_ok = vert_ok[faces].all(axis=1)
        angle_ok = viewing_angle_check(
            p[None, :], surface.centers, surface.normals, max_view_angle_deg
        )[0]
        A[i] = patch_ok & angle_ok

    meta = {
        "module": "m3_visibility",
        "reproduction_guide": "paper05_replication.md §7",
        "paper_criteria": "patch visible iff ALL vertices satisfy FOV & FOD & "
        "occlusion, and the patch viewing angle <= 75 deg (paper text)",
        "source_surface": source_surface,
        "source_viewpoints": source_viewpoints,
        "parameters": {
            "fod": list(fod),
            "fov_deg": list(fov_deg),
            "max_view_angle_deg": max_view_angle_deg,
            "ray_eps": eps,
            "ray_mesh": f"discretized workpiece mesh from patch vertices "
            f"({len(verts)} vertices, {len(faces)} faces)",
        },
        "provenance": {
            "paper": "FOV 30x21 deg (horizontal x vertical), FOD (0.40, 1.00) m, "
            "viewing angle 75 deg, vertex-based visibility, no-occlusion "
            "criterion",
            "ambiguity": "the paper does not define which angle the 75 deg "
            "viewing angle refers to; angle((viewpoint -> patch center), "
            "outward normal) follows the guide's §7.5 worked examples "
            "(the guide's §7.1 snippet has a sign slip -- see module docstring)",
            "engineering": "FOV/FOD/viewing-angle boundaries inclusive; "
            "ray self-hit epsilon 1e-3 m (guide §7.1); camera roll "
            "convention from M2",
        },
        "elapsed_s": round(time.time() - t0, 2),
        "created": _dt.datetime.now().isoformat(timespec="seconds"),
    }
    return VisibilityMatrix(A, meta)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        prog="m3_visibility",
        description="M3 visibility: (S, V, mesh) -> A in {0,1}^{N x M}",
    )
    ap.add_argument("--surface", required=True, help="M1 output S_*.npz")
    ap.add_argument("--viewpoints", required=True, help="M2 output V_*.npz")
    ap.add_argument("--config", type=Path, default=None)
    ap.add_argument("--eps", type=float, default=None, metavar="M")
    ap.add_argument("--out", type=Path, default=Path("data/outputs"))
    ap.add_argument("--no-viz", action="store_true")
    args = ap.parse_args(argv)

    cfg = {}
    if args.config is not None:
        cfg = load_config(args.config)
    sensor = cfg.get("sensor", {})
    fod = (float(sensor.get("fod_min", 0.40)), float(sensor.get("fod_max", 1.00)))
    fov_deg = (float(sensor.get("fov_h_deg", 30.0)), float(sensor.get("fov_v_deg", 21.0)))
    max_angle = float(sensor.get("max_viewing_angle_deg", 75.0))
    eps = args.eps if args.eps is not None else float(
        cfg.get("visibility", {}).get("ray_eps", DEFAULT_RAY_EPS)
    )

    surface = DiscretizedSurface.load(args.surface)
    viewpoints = CandidateViewpointSet.load(args.viewpoints)
    A = compute_visibility(
        surface, viewpoints, fod=fod, fov_deg=fov_deg,
        max_view_angle_deg=max_angle, eps=eps,
        source_surface=str(args.surface), source_viewpoints=str(args.viewpoints),
    )

    args.out.mkdir(parents=True, exist_ok=True)
    name = Path(args.viewpoints).stem.replace("V_", "")
    stem = args.out / f"A_{name}"
    npz_path, json_path = A.save(stem)
    if not args.no_viz:
        A.visualize(stem.with_suffix(".png"), surface=surface, viewpoints=viewpoints)

    print(A.format_report())
    print(f"\nArtifacts: {npz_path}  {json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

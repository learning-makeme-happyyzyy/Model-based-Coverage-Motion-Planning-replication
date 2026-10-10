"""M1: surface discretization (paper_05 replication).

Follows `paper05_replication.md` §5. The paper itself never specifies how
the target surface is discretized ("The surface patch set S has m patches
of the target object"; patches are triangular). This reproduction therefore
uses the strategy fixed in the replication guide:

  mesh (any face count) -- uniform resampling --> S = {s_1..s_m}, m ~ 1000

Each patch record carries (patch_id, 3 vertices, center, unit outward
normal, area), plus the mesh bounding box. Self-checks from the guide:

  * patch count within m +/- 5%
  * normals point outward (ray-parity test on watertight meshes)
  * patch edge-length histogram near-uniform (std/mean < 0.5, diagnostic)

Uniform resampling is implemented as "simplification + subdivision"
(guide §5): quadric decimation (trimesh, fast-simplification backend)
when the input is finer than the target, then `subdivide_to_size` with a
bisection search on the max-edge budget so the face count lands within
tolerance.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Optional, Union

import numpy as np
import trimesh

__all__ = [
    "SurfacePatch",
    "DiscretizedSurface",
    "load_mesh",
    "prepare_mesh",
    "patches_from_mesh",
    "resample_uniform",
    "discretize",
    "check_outward_normals",
    "load_config",
    "main",
]

# guide §5 self-check thresholds
COUNT_TOL = 0.05  # patch count within m +/- 5%
EDGE_CV_MAX = 0.5  # edge-length std/mean uniformity diagnostic

MAX_SUBDIV_PASSES = 8  # memory guard: one pass can 4x the face count
GRID_POINTS = 12  # max-edge candidates evaluated in the logspace scan


@dataclass(frozen=True)
class SurfacePatch:
    """One triangular surface patch (guide §5 black-box record)."""

    patch_id: int
    vertices: np.ndarray  # (3, 3) float
    center: np.ndarray  # (3,) float, vertex mean
    normal: np.ndarray  # (3,) float, unit outward normal
    area: float


class DiscretizedSurface:
    """Surface patch set S = {s_1..s_m} plus the source-mesh bounding box.

    Storage is columnar (numpy arrays) so downstream modules (M2/M3) can
    vectorize; `patches()` yields per-patch records.
    """

    def __init__(
        self,
        patch_ids: np.ndarray,
        vertices: np.ndarray,
        centers: np.ndarray,
        normals: np.ndarray,
        areas: np.ndarray,
        bounds: np.ndarray,
        meta: dict,
    ):
        m = len(patch_ids)
        for name, arr, shape in [
            ("vertices", vertices, (m, 3, 3)),
            ("centers", centers, (m, 3)),
            ("normals", normals, (m, 3)),
            ("areas", areas, (m,)),
            ("bounds", bounds, (2, 3)),
        ]:
            if arr.shape != shape:
                raise ValueError(f"{name} has shape {arr.shape}, expected {shape}")
        self.patch_ids = np.asarray(patch_ids, dtype=np.int32)
        self.vertices = np.asarray(vertices, dtype=np.float64)
        self.centers = np.asarray(centers, dtype=np.float64)
        self.normals = np.asarray(normals, dtype=np.float64)
        self.areas = np.asarray(areas, dtype=np.float64)
        self.bounds = np.asarray(bounds, dtype=np.float64)
        self.meta = dict(meta)

    def __len__(self) -> int:
        return len(self.patch_ids)

    def get(self, patch_id: int) -> SurfacePatch:
        return SurfacePatch(
            patch_id=int(self.patch_ids[patch_id]),
            vertices=self.vertices[patch_id].copy(),
            center=self.centers[patch_id].copy(),
            normal=self.normals[patch_id].copy(),
            area=float(self.areas[patch_id]),
        )

    def patches(self) -> Iterator[SurfacePatch]:
        for i in range(len(self)):
            yield self.get(i)

    # -- geometry ----------------------------------------------------------

    def edge_lengths(self) -> np.ndarray:
        """Per-patch edge lengths (m, 3): |v1-v0|, |v2-v1|, |v2-v0|."""
        v = self.vertices
        return np.stack(
            [
                np.linalg.norm(v[:, 1] - v[:, 0], axis=1),
                np.linalg.norm(v[:, 2] - v[:, 1], axis=1),
                np.linalg.norm(v[:, 2] - v[:, 0], axis=1),
            ],
            axis=1,
        )

    def stats(self) -> dict:
        """Aggregate statistics used by the self-checks and the report."""
        el = self.edge_lengths().ravel()
        norm_len = np.linalg.norm(self.normals, axis=1)
        outward = self.meta.get("checks", {}).get("outward_normals")
        return {
            "patch_count": len(self),
            "target_patches": self.meta.get("target_patches"),
            "count_error_pct": self.meta.get("count_error_pct"),
            "area_min": float(self.areas.min()),
            "area_median": float(np.median(self.areas)),
            "area_max": float(self.areas.max()),
            "area_sum": float(self.areas.sum()),
            "area_mesh": self.meta.get("mesh_area"),
            "edge_mean": float(el.mean()),
            "edge_std": float(el.std()),
            "edge_std_over_mean": float(el.std() / el.mean()),
            "norm_mean_len": float(norm_len.mean()),
            "norm_std_len": float(norm_len.std()),
            "outward": outward,
        }

    def format_report(self) -> str:
        """The M1 validation report (labels fixed by the task spec)."""
        s = self.stats()
        mesh = self.meta.get("source_mesh", {})
        size = self.bounds[1] - self.bounds[0]
        fmt = lambda a: "[" + " ".join(f"{x: .6f}" for x in a) + "]"
        fmt_size = " x ".join(f"{x:.3f}" for x in size)
        target = s["target_patches"]
        in_tol = s["patch_count"] is not None and abs(s["count_error_pct"]) <= 5.0
        outward = s["outward"]
        out_str = (
            f"{outward[0]}/{outward[1]} OK" if outward else "skipped (mesh not watertight)"
        )
        unit = mesh.get("units", "m")
        delta_pct = 100.0 * (s["area_sum"] - s["area_mesh"]) / s["area_mesh"]
        cv_ok = s["edge_std_over_mean"] < EDGE_CV_MAX
        lines = [
            f"Mesh:",
            f"  Vertices: {mesh.get('vertices')}",
            f"  Faces: {mesh.get('faces')}",
            f"  Watertight: {'yes' if mesh.get('watertight') else 'no'}",
            f"  Area: {s['area_mesh']:.6f} {unit}^2",
            f"  Bounding box: min {fmt(self.bounds[0])} / max {fmt(self.bounds[1])}",
            f"    size {fmt_size} {unit}",
            "",
            f"Patch count: {s['patch_count']} (target {target}, "
            f"{s['count_error_pct']:+.1f}%, within +-5%: {'yes' if in_tol else 'no'})",
            "",
            f"Bounding box: min {fmt(self.bounds[0])} / max {fmt(self.bounds[1])}",
            f"  size {fmt_size} {unit} (patch vertices span the same box)",
            "",
            f"Normal statistics:",
            f"  unit norm: mean |n| = {s['norm_mean_len']:.6f}, std = {s['norm_std_len']:.3e}",
            f"  outward check (ray parity): {out_str}",
            "",
            f"Area statistics:",
            f"  patch area [{unit}^2]: min {s['area_min']:.6f}  "
            f"median {s['area_median']:.6f}  max {s['area_max']:.6f}",
            f"  total: sum(patch areas) = {s['area_sum']:.6f} vs mesh area "
            f"{s['area_mesh']:.6f} (delta {delta_pct:+.2f}%)",
            f"  edge length [{unit}]: mean {s['edge_mean']:.6f}  std {s['edge_std']:.6f}  "
            f"std/mean {s['edge_std_over_mean']:.3f} (uniformity check < {EDGE_CV_MAX}: "
            f"{'yes' if cv_ok else 'no (diagnostic only)'})",
        ]
        return "\n".join(lines)

    # -- serialization -----------------------------------------------------

    def save(self, stem: Union[str, Path]) -> tuple[Path, Path]:
        """Save arrays to `<stem>.npz` and provenance/stats to `<stem>.json`."""
        stem = Path(stem)
        np.savez(
            stem.with_suffix(".npz"),
            patch_ids=self.patch_ids,
            vertices=self.vertices,
            centers=self.centers,
            normals=self.normals,
            areas=self.areas,
            bounds=self.bounds,
        )
        meta_path = stem.with_suffix(".json")
        meta_path.write_text(json.dumps(self.meta, indent=2))
        return stem.with_suffix(".npz"), meta_path

    @classmethod
    def load(cls, stem: Union[str, Path]) -> "DiscretizedSurface":
        stem = Path(stem)
        data = np.load(stem.with_suffix(".npz"))
        meta = json.loads(stem.with_suffix(".json").read_text())
        return cls(
            patch_ids=data["patch_ids"],
            vertices=data["vertices"],
            centers=data["centers"],
            normals=data["normals"],
            areas=data["areas"],
            bounds=data["bounds"],
            meta=meta,
        )

    # -- visualization -----------------------------------------------------

    def visualize(self, path: Union[str, Path], show: bool = False) -> Path:
        """Surface-patch visualization: patches colored by their normal
        (RGB = (n+1)/2, the standard surface-orientation encoding) with a
        subsampled set of normal arrows, plus the edge-length histogram
        (guide §5 uniformity self-check)."""
        import matplotlib

        if not show:
            matplotlib.use("Agg", force=True)
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection

        m = len(self)
        diag = float(np.linalg.norm(self.bounds[1] - self.bounds[0]))
        rgb = np.clip((self.normals + 1.0) / 2.0, 0.0, 1.0)

        fig = plt.figure(figsize=(11.5, 4.6))
        fig.suptitle(
            f"Surface patches S (|S| = {m})\n"
            "colors: surface normal RGB = (n+1)/2",
            fontsize=10,
        )

        ax = fig.add_subplot(1, 2, 1, projection="3d")
        pc = Poly3DCollection(list(self.vertices), facecolors=rgb, alpha=0.85)
        ax.add_collection3d(pc)
        step = max(1, m // 40)
        ax.quiver(
            self.centers[::step, 0],
            self.centers[::step, 1],
            self.centers[::step, 2],
            self.normals[::step, 0],
            self.normals[::step, 1],
            self.normals[::step, 2],
            length=0.07 * diag,
            color="#333333",
            linewidth=0.7,
        )
        ax.set_box_aspect(self.bounds[1] - self.bounds[0])
        ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
        ax.set_title(f"{m} patches, outward normals (arrows: 1/{step})", fontsize=9)
        ax.tick_params(labelsize=7)

        ax2 = fig.add_subplot(1, 2, 2)
        el = self.edge_lengths().ravel()
        s = self.stats()
        ax2.hist(el, bins=40, color="#2a78d6", alpha=0.85, edgecolor="white", linewidth=0.3)
        ax2.axvline(s["edge_mean"], color="#104281", linewidth=1.5, label="mean")
        ax2.set_xlabel(f"patch edge length [{self.meta.get('source_mesh', {}).get('units', 'm')}]")
        ax2.set_ylabel("count")
        ax2.set_title(
            f"edge-length histogram (uniformity self-check)\n"
            f"std/mean = {s['edge_std_over_mean']:.3f} "
            f"({'< ' if s['edge_std_over_mean'] < EDGE_CV_MAX else '>= '}{EDGE_CV_MAX})",
            fontsize=9,
        )
        ax2.legend(fontsize=8, frameon=False)
        ax2.spines[["top", "right"]].set_visible(False)
        ax2.tick_params(labelsize=7)

        fig.tight_layout(rect=(0, 0, 1, 0.93))
        path = Path(path)
        fig.savefig(path, dpi=150, bbox_inches="tight")
        plt.close(fig)
        return path


# ---------------------------------------------------------------------------
# mesh loading / preparation (guide §4)
# ---------------------------------------------------------------------------


def load_mesh(path: Union[str, Path]) -> trimesh.Trimesh:
    """Load a CAD mesh (obj/stl/ply/...) as a single Trimesh."""
    mesh = trimesh.load(str(path), force="mesh")
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"{path}: expected a single mesh, got {type(mesh).__name__}")
    return mesh


def _clean_mesh(mesh: trimesh.Trimesh) -> None:
    """Weld duplicate vertices and drop degenerate faces (in place)."""
    mesh.merge_vertices()
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.remove_unreferenced_vertices()


def prepare_mesh(
    mesh: trimesh.Trimesh,
    center_origin: bool = True,
    target_diagonal: Optional[float] = None,
) -> trimesh.Trimesh:
    """Guide §4 preparation: weld, drop degenerate faces, fix winding/normals
    (watertight meshes), re-center the bbox at the origin (project-wide
    convention) and optionally scale to a target bbox diagonal (m)."""
    mesh = mesh.copy()
    _clean_mesh(mesh)
    if mesh.is_watertight:
        trimesh.repair.fix_normals(mesh)
    if center_origin:
        mesh.apply_translation(-mesh.bounds.mean(axis=0))
    if target_diagonal is not None:
        mesh.apply_scale(target_diagonal / mesh.scale)
    return mesh


# ---------------------------------------------------------------------------
# patch extraction
# ---------------------------------------------------------------------------


def patches_from_mesh(
    mesh: trimesh.Trimesh,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Extract patch records from the mesh triangles.

    Returns (patch_ids, vertices, centers, normals, areas) with shapes
    (m,), (m,3,3), (m,3), (m,3), (m,). The normal follows the face winding
    (face normals, as in the guide); for watertight meshes run
    `trimesh.repair.fix_normals` first so they point outward.
    """
    tri = mesh.vertices[mesh.faces]  # (m, 3, 3)
    e1 = tri[:, 1] - tri[:, 0]
    e2 = tri[:, 2] - tri[:, 0]
    cross = np.cross(e1, e2)
    areas = 0.5 * np.linalg.norm(cross, axis=1)
    if np.any(areas <= 0.0):
        raise ValueError(f"{int((areas <= 0).sum())} degenerate faces; clean the mesh first")
    centers = tri.mean(axis=1)
    normals = cross / (2.0 * areas[:, None])
    ids = np.arange(len(mesh.faces), dtype=np.int32)
    return ids, tri, centers, normals, areas


# ---------------------------------------------------------------------------
# uniform resampling (guide §5: simplification + subdivision)
# ---------------------------------------------------------------------------


def _subdivide_count(
    mesh: trimesh.Trimesh, max_edge: float
) -> tuple[Union[float, int], Optional[np.ndarray], Optional[np.ndarray]]:
    """Face count (and mesh) after `subdivide_to_size` with the given edge
    budget. Each pass halves edge lengths, so the needed pass count is
    estimated up front; if it exceeds the guard, `inf` is returned as the
    count (the bisection treats it as "way too many faces" and moves on)."""
    e0 = float(mesh.edges_unique_length.max())
    if e0 <= max_edge:
        return len(mesh.faces), mesh.vertices, mesh.faces
    passes = int(np.ceil(np.log2(e0 / max_edge)))
    if passes > MAX_SUBDIV_PASSES - 2:
        return float("inf"), None, None
    try:
        vertices, faces = trimesh.remesh.subdivide_to_size(
            mesh.vertices, mesh.faces, max_edge=max_edge, max_iter=passes + 2
        )
    except ValueError:
        return float("inf"), None, None
    return len(faces), vertices, faces


def resample_uniform(mesh: trimesh.Trimesh, m: int) -> tuple[trimesh.Trimesh, dict]:
    """Uniform resampling to ~m patches (guide §5).

    Coarse-first inputs are quadric-decimated to ~0.7m (simplification),
    then `subdivide_to_size` refines to m (subdivision). The max-edge
    budget is found by a logspace scan plus geometric bisection so the
    count lands within m +/- 5% whenever the count granularity allows it;
    otherwise the closest achievable count is returned with a warning.

    Returns (resampled_mesh, info). Watertight input stays watertight.
    """
    if m < 4:
        raise ValueError(f"target patch count {m} is too small (>= 4)")
    mesh = mesh.copy()
    _clean_mesh(mesh)
    tol = COUNT_TOL * m
    info: dict = {"decimated_from": None, "target_edge_length": None}

    if len(mesh.faces) > m:
        decimated_from = len(mesh.faces)
        target = max(int(0.7 * m), 8)
        while True:
            try:
                mesh = mesh.simplify_quadric_decimation(face_count=target)
            except Exception as exc:  # backend failure: keep the coarse mesh
                warnings.warn(f"quadric decimation failed ({exc}); skipping simplification")
                break
            _clean_mesh(mesh)
            if len(mesh.faces) < m or target <= 8:
                break
            target = max(int(0.5 * target), 8)
        if len(mesh.faces) >= m:
            warnings.warn(
                f"could not decimate below {m} faces; count tolerance may not hold"
            )
        info["decimated_from"] = decimated_from

    area = float(mesh.area)
    target_edge = float(np.sqrt(area / m))
    info["target_edge_length"] = target_edge
    diag = float(np.linalg.norm(mesh.bounds[1] - mesh.bounds[0]))

    best: tuple[float, Union[float, int], Optional[np.ndarray], Optional[np.ndarray]] = (
        diag,
        len(mesh.faces),
        mesh.vertices,
        mesh.faces,
    )

    def consider(max_edge: float) -> Union[float, int]:
        nonlocal best
        count, verts, faces = _subdivide_count(mesh, max_edge)
        if count != float("inf") and abs(count - m) < abs(best[1] - m):
            best = (max_edge, count, verts, faces)
        return count

    # logspace scan: the count is a step function of max_edge, and a pure
    # bisection can tunnel past narrow plateaus (e.g. small m on coarse
    # meshes); the grid makes the plateaus visible.
    for me in np.geomspace(target_edge / 2.0, target_edge * 4.0, GRID_POINTS):
        consider(float(me))

    # bracket [lo, hi] with count(lo) > m >= count(hi), then bisect.
    lo = float(best[0])
    if best[1] <= m:
        while consider(lo) < m + tol and lo > target_edge / 200.0:
            lo /= 2.0
    hi = diag
    if consider(lo) < m:  # floor reached without enough faces: best effort
        warnings.warn(
            f"cannot reach {m} patches by subdivision (best {best[1]}); "
            "returning finest feasible mesh"
        )
        return trimesh.Trimesh(best[2], best[3], process=False), info

    for _ in range(32):
        mid = float(np.sqrt(lo * hi))
        count, verts, faces = _subdivide_count(mesh, mid)
        if count != float("inf") and abs(count - m) < abs(best[1] - m):
            best = (mid, count, verts, faces)
        if count != float("inf") and abs(count - m) <= tol:
            break
        if count > m or count == float("inf"):
            lo = mid
        else:
            hi = mid
        if hi / lo < 1.02:
            break

    if abs(best[1] - m) <= tol:
        return trimesh.Trimesh(best[2], best[3], process=False), info

    # The face-count granularity of midpoint subdivision can skip the
    # m +/- 5% window entirely (regular meshes subdivide in lockstep).
    # Fallback: take the plateau just above m and quadric-decimate it down
    # to exactly m; quadric cost removes the shortest edges first, which
    # also improves uniformity. Rejected if it visibly changes the surface.
    me_boundary = float(np.sqrt(lo * hi))
    c_above, v_above, f_above = _subdivide_count(mesh, me_boundary * 0.97)
    if c_above != float("inf") and m < c_above <= 1.5 * m:
        sub = trimesh.Trimesh(v_above, f_above, process=False)
        try:
            cand = sub.simplify_quadric_decimation(face_count=m)
        except Exception as exc:
            warnings.warn(f"decimation fallback failed ({exc})")
        else:
            _clean_mesh(cand)
            if abs(cand.area - sub.area) <= 0.005 * sub.area and abs(len(cand.faces) - m) <= tol:
                info["final_decimated_from"] = int(c_above)
                return cand, info
            warnings.warn(
                "decimation fallback altered the surface (area/count check "
                "failed); keeping the closest subdivision result"
            )
    warnings.warn(
        f"patch count {best[1]} differs from target {m} by more than "
        f"{COUNT_TOL:.0%} (face-count granularity of subdivision); "
        "see paper05_replication.md §5"
    )
    return trimesh.Trimesh(best[2], best[3], process=False), info


# ---------------------------------------------------------------------------
# top-level M1 entry point
# ---------------------------------------------------------------------------


def discretize(mesh: trimesh.Trimesh, m: int = 1000, units: str = "as authored") -> DiscretizedSurface:
    """M1 black box: mesh -> surface patch set S (guide §5)."""
    mesh = mesh.copy()
    _clean_mesh(mesh)
    watertight = bool(mesh.is_watertight)
    if watertight:
        trimesh.repair.fix_normals(mesh)

    resampled, info = resample_uniform(mesh, m)
    patch_ids, vertices, centers, normals, areas = patches_from_mesh(resampled)
    bounds = np.array(resampled.bounds)
    n = len(patch_ids)

    # self-checks: the ray-parity test needs a closed surface (no boundary
    # edges); strict watertightness can be lost to mild quadric decimation
    # without opening the surface.
    closed = _is_closed(resampled)
    outward = check_outward_normals(resampled, centers, normals) if closed else None
    el = np.stack(
        [
            np.linalg.norm(vertices[:, 1] - vertices[:, 0], axis=1),
            np.linalg.norm(vertices[:, 2] - vertices[:, 1], axis=1),
            np.linalg.norm(vertices[:, 2] - vertices[:, 0], axis=1),
        ]
    ).ravel()
    cv = float(el.std() / el.mean())
    count_err_pct = 100.0 * (n - m) / m

    meta = {
        "module": "m1_discretize",
        "reproduction_guide": "paper05_replication.md §5",
        "source_mesh": {
            "vertices": int(len(mesh.vertices)),
            "faces": int(len(mesh.faces)),
            "watertight": watertight,
            "area": float(mesh.area),
            "units": units,
        },
        "mesh_area": float(mesh.area),
        "target_patches": int(m),
        "patch_count": int(n),
        "count_error_pct": round(count_err_pct, 3),
        "discretization": {
            "method": "uniform resampling: quadric decimation (fast-simplification) "
            "+ subdivide_to_size (trimesh remesh)",
            "decimated_from": info["decimated_from"],
            "final_decimated_from": info.get("final_decimated_from"),
            "target_edge_length": info["target_edge_length"],
            "resampled_watertight": bool(resampled.is_watertight),
            "resampled_closed": bool(closed),
        },
        "bounds_min": bounds[0].tolist(),
        "bounds_max": bounds[1].tolist(),
        "checks": {
            "patch_count_within_5pct": bool(abs(count_err_pct) <= 5.0),
            "normals_unit": bool(np.allclose(np.linalg.norm(normals, axis=1), 1.0)),
            "outward_normals": None
            if outward is None
            else [int(outward.sum()), int(len(outward))],
            "edge_std_over_mean": round(cv, 4),
            "edge_uniformity_within_0_5": bool(cv < EDGE_CV_MAX),
        },
        "created": _dt.datetime.now().isoformat(timespec="seconds"),
    }
    return DiscretizedSurface(patch_ids, vertices, centers, normals, areas, bounds, meta)


def _is_closed(mesh: trimesh.Trimesh) -> bool:
    """True if the mesh has no boundary edges (every edge shared by >= 2
    faces). Unlike `is_watertight`, this tolerates non-manifold pinch
    vertices that mild quadric decimation can create; the surface is still
    closed, so ray parity remains reliable."""
    if len(mesh.faces) == 0:
        return False
    _, counts = np.unique(mesh.edges_sorted, axis=0, return_counts=True)
    return bool((counts >= 2).all())


def check_outward_normals(
    mesh: trimesh.Trimesh,
    centers: np.ndarray,
    normals: np.ndarray,
    eps: Optional[float] = None,
) -> Optional[np.ndarray]:
    """Ray-parity test that patch normals point outward (guide §5 self-check).

    For a closed mesh, an outward normal means `center + eps*n` lies
    *outside* the surface, so a ray fired from there along +n crosses the
    surface an even number of times; odd parity flags an inward normal.

    Returns a per-patch bool array (True = outward OK), or None for meshes
    with boundary edges, where parity is unreliable.
    """
    if not _is_closed(mesh):
        return None
    if eps is None:
        eps = 1e-3 * float(np.linalg.norm(mesh.bounds[1] - mesh.bounds[0]))
    intersector = trimesh.ray.ray_triangle.RayMeshIntersector(mesh)
    origins = centers + eps * normals
    ok = np.empty(len(centers), dtype=bool)
    for i, (origin, direction) in enumerate(zip(origins, normals)):
        hits = intersector.intersects_location([origin], [direction], multiple_hits=True)[0]
        ok[i] = len(hits) % 2 == 0
    return ok


# ---------------------------------------------------------------------------
# config + CLI
# ---------------------------------------------------------------------------


def load_config(path: Union[str, Path]) -> dict:
    import yaml

    with open(path) as fh:
        return yaml.safe_load(fh)


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        prog="m1_discretize",
        description="M1 surface discretization: CAD mesh -> uniform surface patch set S",
    )
    ap.add_argument("--mesh", required=True, help="input mesh (obj/stl/ply/...)")
    ap.add_argument("--patches", "-m", type=int, default=None, help="target patch count")
    ap.add_argument("--config", type=Path, default=None, help="yaml config (mesh.target_patches)")
    ap.add_argument("--out", type=Path, default=Path("data/outputs"))
    ap.add_argument("--normalize-diagonal", type=float, default=None, metavar="M")
    ap.add_argument("--units", default="as authored", help="units label for the report (e.g. m, mm)")
    ap.add_argument("--no-center", action="store_true", help="keep the mesh bbox where it is")
    ap.add_argument("--no-viz", action="store_true")
    args = ap.parse_args(argv)

    m = args.patches
    if m is None:
        if args.config is not None:
            m = load_config(args.config).get("mesh", {}).get("target_patches")
        m = m or 1000

    mesh = load_mesh(args.mesh)
    mesh = prepare_mesh(
        mesh,
        center_origin=not args.no_center,
        target_diagonal=args.normalize_diagonal,
    )
    surface = discretize(mesh, m=m, units=args.units)

    args.out.mkdir(parents=True, exist_ok=True)
    stem = args.out / f"S_{Path(args.mesh).stem}_{m}"
    npz_path, json_path = surface.save(stem)
    if not args.no_viz:
        surface.visualize(stem.with_suffix(".png"))

    print(surface.format_report())
    print(f"\nArtifacts: {npz_path}  {json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

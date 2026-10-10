"""M2: candidate viewpoint generation (paper_05 replication).

Follows `paper05_replication.md` §6. M2 only generates *possible*
observation poses; "which patch can be seen from where" is M3.

Paper facts (verbatim from the PDF):

  * positions: "randomized sampling ... performed in Euclidean space
    around the target object, within the maximum viewing range of the
    sensor. Redundant viewpoints are generated in this step."
  * viewing direction: potential field, Eq. (10)

        v = ( sum_i K d_i / ||d_i||^3 ) / || sum_i K d_i / ||d_i||^3 ||

        d_i = p_vp - p_patch_i,  summed over
        { p_patch_i : (p_vp - p_patch_i)^T (p_vp - p_patch_i) < d_max }
        K = constant, d_max = maximum inclusion distance (no value given)

  * roll about the optical axis is free: "rotating a given viewpoint
    about its optical axis does not significantly alter the sensor's
    ability to capture the geometric data of a given scene".
  * n in {200, 300, 400} (Tables II/III); "the same set of sampled
    candidate viewpoints are used for the three different methods".

Eq. (10) sign resolution (documented ambiguity)
-----------------------------------------------
With the paper's literal d_i = p_vp - p_patch_i, the sum points from the
patches toward the camera. The physically meaningful quantity is the
camera *optical axis* (camera -> scene): the field is meant to orient
each viewpoint so it *captures* nearby patches, and both the paper's
roll-about-optical-axis argument and the M3 FOV check act on the optical
axis. We therefore store

    v_hat = normalize( sum_{d_i < d_max} (p_patch_i - p_vp) / ||p_patch_i - p_vp||^3 )

i.e. the negative of the paper's literal sum. The reproduction guide
(§6.2) adopts the same resolution; K cancels under normalization.
This reading is flagged as an interpretation, not a paper statement.

Empty-neighborhood limit (engineering choice): when no patch lies within
d_max, Eq. (10) has no terms. We use the d_max -> 0+ limit of the field
-- the direction of the nearest patch -- and record the fallback count
in the metadata. The guide's §6.4 diagnostic is the intended mechanism
for tuning d_max (its initial 0.30 m is inconsistent with its own
sampling shell for any object: the minimum viewpoint-to-surface
distance is FOD_min + margin = 0.45 m > 0.30 m, see example output).

Reproduction settings (from the guide, not the paper):
  * position shell (guide §6.1): direction uniform on the sphere,
    distance from the bbox center d ~ U(halfdiag + FOD_min + margin,
    halfdiag + FOD_max), reject below the table plane
    z = bbox_min_z - table_clearance (guide §4)
  * reachability shell (guide §6.3, replaces the paper's IK+FCL,
    simplification #2): r_min <= ||p - base|| <= r_max, z >= table_z + 0.1
  * d_max = 0.30 m initial value (guide §6, to be tuned via §6.4),
    seed = 0, base = (0.9, 0, 0) m
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Optional, Union

import numpy as np

from src.m1_discretize import DiscretizedSurface, load_config

__all__ = [
    "CandidateViewpoint",
    "CandidateViewpointSet",
    "sample_positions",
    "reachability_filter",
    "potential_direction",
    "potential_directions",
    "recommend_d_max",
    "generate_candidates",
    "validate",
    "main",
]

# guide §6.4 diagnostic 1 target: median neighbor count >= 10
MIN_NEIGHBORS_TARGET = 10
# rejection-sampling safety cap (acceptance rate depends on the shell)
MAX_ATTEMPTS_PER_POINT = 200


@dataclass(frozen=True)
class CandidateViewpoint:
    """One candidate viewpoint: position + viewing direction (optical axis)."""

    viewpoint_id: int
    position: np.ndarray  # (3,)
    viewing_direction: np.ndarray  # (3,) unit, camera -> scene

    def viewing_frame(self) -> np.ndarray:
        """Camera frame rows [x_c, y_c, z_c] with z_c = viewing direction.

        The roll about the optical axis is not fixed by the paper (see
        module docstring); this uses the convention of
        paper05_replication.md §7.1 so M3's FOV check can consume it
        directly. Convention, not a paper statement.
        """
        return viewing_frames(
            self.viewing_direction[None, :], self.position[None, :]
        )[0]


def viewing_frames(directions: np.ndarray, positions: Optional[np.ndarray] = None) -> np.ndarray:
    """(n, 3, 3) camera frames for unit viewing directions, rows [x, y, z].

    z = v_hat; x = normalize(cross(v_hat, [0, 0, 1])), falling back to
    cross(v_hat, [0, 1, 0]) when v_hat is parallel to z; y = cross(z, x).
    Guide §7.1 convention (roll arbitrary -- not specified by the paper).
    """
    directions = np.asarray(directions, dtype=np.float64)
    n = len(directions)
    z = directions / np.linalg.norm(directions, axis=1, keepdims=True)
    ref = np.broadcast_to([0.0, 0.0, 1.0], (n, 3)).copy()
    x = np.cross(z, ref)
    degenerate = np.linalg.norm(x, axis=1) < 1e-12
    if degenerate.any():
        ref[degenerate] = [0.0, 1.0, 0.0]
        x[degenerate] = np.cross(z[degenerate], [0.0, 1.0, 0.0])
    x /= np.linalg.norm(x, axis=1, keepdims=True)
    y = np.cross(z, x)
    return np.stack([x, y, z], axis=1)


# ---------------------------------------------------------------------------
# position sampling (guide §6.1) and reachability filter (guide §6.3)
# ---------------------------------------------------------------------------


def sample_positions(
    center: np.ndarray,
    bbox_halfdiag: float,
    n: int,
    rng: np.random.Generator,
    fod: tuple[float, float] = (0.40, 1.00),
    distance_margin: float = 0.05,
    table_z: Optional[float] = None,
) -> np.ndarray:
    """Guide §6.1: sample n viewpoint positions on a spherical shell.

    Directions uniform on the sphere (the table rejection leaves the
    "predominantly upper hemisphere" of the guide), distance from the
    bbox center d ~ U(bbox_halfdiag + FOD_min + margin,
    bbox_halfdiag + FOD_max), positions below the table plane rejected.
    """
    low = bbox_halfdiag + fod[0] + distance_margin
    high = bbox_halfdiag + fod[1]
    positions = np.empty((n, 3))
    i = 0
    for _ in range(n * MAX_ATTEMPTS_PER_POINT):
        d = float(rng.uniform(low, high))
        u = rng.normal(size=3)
        u /= np.linalg.norm(u)
        p = center + d * u
        if table_z is not None and p[2] < table_z:
            continue
        positions[i] = p
        i += 1
        if i == n:
            return positions
    raise RuntimeError(
        f"could not sample {n} positions after {n * MAX_ATTEMPTS_PER_POINT} "
        "attempts; check the shell/table/reachability parameters"
    )


def reachability_filter(
    positions: np.ndarray,
    base: tuple[float, float, float] = (0.9, 0.0, 0.0),
    r_min: float = 0.55,
    r_max: float = 1.30,
    z_min: Optional[float] = None,
) -> np.ndarray:
    """Guide §6.3: reachability shell replacing the paper's IK+FCL filter.

    A simplified envelope of a 6-DOF arm (no collision checking, guide
    simplification #2): r_min <= ||p - base|| <= r_max and z >= z_min
    (z_min = table_z + 0.1 in the pipeline).
    """
    positions = np.asarray(positions, dtype=np.float64)
    ok = np.ones(len(positions), dtype=bool)
    dist = np.linalg.norm(positions - np.asarray(base, dtype=np.float64), axis=1)
    ok &= (dist >= r_min) & (dist <= r_max)
    if z_min is not None:
        ok &= positions[:, 2] >= z_min
    return ok


# ---------------------------------------------------------------------------
# viewing direction: potential field, Eq. (10)
# ---------------------------------------------------------------------------


def potential_direction(
    p: np.ndarray, patch_centers: np.ndarray, d_max: float
) -> tuple[np.ndarray, int, bool]:
    """Eq. (10) for one viewpoint; returns (v_hat, n_neighbors, fallback).

    v_hat is the camera optical axis (camera -> scene), i.e. the negative
    of the paper's literal sum -- see the module docstring for the sign
    resolution. Patches with squared distance < d_max^2 contribute with
    weight 1/d^2 (K cancels under normalization). When the neighborhood
    is empty the d_max -> 0+ limit is used: the direction of the nearest
    patch (fallback=True).
    """
    rel = np.asarray(patch_centers, dtype=np.float64) - np.asarray(p, dtype=np.float64)
    dist2 = (rel**2).sum(axis=1)
    ok = (dist2 > 0.0) & (dist2 < d_max**2)
    if ok.any():
        w = (rel[ok] / dist2[ok, None] ** 1.5).sum(axis=0)
        return w / np.linalg.norm(w), int(ok.sum()), False
    d2 = np.where(dist2 > 0.0, dist2, np.inf)
    j = int(np.argmin(d2))
    if not np.isfinite(d2[j]):
        raise ValueError("no patch at finite distance; viewing direction undefined")
    v = rel[j]
    return v / np.linalg.norm(v), 0, True


def potential_directions(
    positions: np.ndarray, patch_centers: np.ndarray, d_max: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Vectorized Eq. (10); returns (directions (n,3), n_neighbors (n,),
    fallback (n,) bool). Same conventions as `potential_direction`."""
    positions = np.asarray(positions, dtype=np.float64)
    patch_centers = np.asarray(patch_centers, dtype=np.float64)
    rel = patch_centers[None, :, :] - positions[:, None, :]  # (n, m, 3)
    dist2 = (rel**2).sum(axis=-1)
    mask = (dist2 > 0.0) & (dist2 < d_max**2)
    n_nb = mask.sum(axis=1)
    w = np.where(mask[..., None], rel / dist2[..., None] ** 1.5, 0.0).sum(axis=1)
    norm = np.linalg.norm(w, axis=1)
    fallback = norm == 0.0
    if fallback.any():
        # d_max -> 0+ limit: nearest patch dominates
        d2 = np.where(dist2 > 0.0, dist2, np.inf)
        j = np.argmin(d2, axis=1)
        w[fallback] = rel[np.arange(len(positions)), j][fallback]
        norm = np.linalg.norm(w, axis=1)
    return w / norm[:, None], n_nb, fallback


def recommend_d_max(
    positions: np.ndarray,
    patch_centers: np.ndarray,
    min_neighbors: int = MIN_NEIGHBORS_TARGET,
) -> tuple[float, np.ndarray]:
    """Guide §6.4 diagnostic 1: the smallest d_max such that the *median*
    neighbor count over viewpoints reaches `min_neighbors`, found by
    bisection on the actual median (Eq. 10 uses a strict `< d_max`, which
    is respected here). Returns (d_max, pairwise distance matrix)."""
    dist = np.linalg.norm(
        np.asarray(patch_centers)[None, :, :] - np.asarray(positions)[:, None, :], axis=-1
    )

    def median_nb(d: float) -> float:
        return float(np.median((dist < d).sum(axis=1)))

    hi = float(dist.max())
    if median_nb(hi) < min_neighbors:
        # not enough patches in the whole surface to reach the target
        return hi, dist
    lo = 0.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if median_nb(mid) >= min_neighbors:
            hi = mid
        else:
            lo = mid
        if hi - lo < 1e-6:
            break
    return hi, dist


# ---------------------------------------------------------------------------
# viewpoint container
# ---------------------------------------------------------------------------


class CandidateViewpointSet:
    """Candidate viewpoint set V = {v_1..v_n} (columnar storage + metadata)."""

    def __init__(
        self,
        viewpoint_ids: np.ndarray,
        positions: np.ndarray,
        viewing_directions: np.ndarray,
        meta: dict,
    ):
        n = len(viewpoint_ids)
        for name, arr, shape in [
            ("positions", positions, (n, 3)),
            ("viewing_directions", viewing_directions, (n, 3)),
        ]:
            if arr.shape != shape:
                raise ValueError(f"{name} has shape {arr.shape}, expected {shape}")
        self.viewpoint_ids = np.asarray(viewpoint_ids, dtype=np.int32)
        self.positions = np.asarray(positions, dtype=np.float64)
        self.viewing_directions = np.asarray(viewing_directions, dtype=np.float64)
        self.meta = dict(meta)

    def __len__(self) -> int:
        return len(self.viewpoint_ids)

    def get(self, i: int) -> CandidateViewpoint:
        return CandidateViewpoint(
            viewpoint_id=int(self.viewpoint_ids[i]),
            position=self.positions[i].copy(),
            viewing_direction=self.viewing_directions[i].copy(),
        )

    def viewpoints(self) -> Iterator[CandidateViewpoint]:
        for i in range(len(self)):
            yield self.get(i)

    def viewing_frames(self) -> np.ndarray:
        """(n, 3, 3) camera frames [x, y, z] for M3 (guide §7.1 convention)."""
        return viewing_frames(self.viewing_directions)

    def subset(self, n: int) -> "CandidateViewpointSet":
        """First n viewpoints. The paper states that "the same set of
        sampled candidate viewpoints are used for the three different
        methods", so n=200/300 are prefixes of the n=400 master set
        (deterministic stream: identical to generating with n directly)."""
        if n > len(self):
            raise ValueError(f"subset {n} > master set size {len(self)}")
        meta = dict(self.meta)
        meta["n"] = int(n)
        meta["master_n"] = len(self)
        return CandidateViewpointSet(
            self.viewpoint_ids[:n], self.positions[:n], self.viewing_directions[:n], meta
        )

    def save(self, stem: Union[str, Path]) -> tuple[Path, Path]:
        stem = Path(stem)
        np.savez(
            stem.with_suffix(".npz"),
            viewpoint_ids=self.viewpoint_ids,
            positions=self.positions,
            viewing_directions=self.viewing_directions,
        )
        meta_path = stem.with_suffix(".json")
        meta_path.write_text(json.dumps(self.meta, indent=2))
        return stem.with_suffix(".npz"), meta_path

    @classmethod
    def load(cls, stem: Union[str, Path]) -> "CandidateViewpointSet":
        stem = Path(stem)
        data = np.load(stem.with_suffix(".npz"))
        meta = json.loads(stem.with_suffix(".json").read_text())
        return cls(
            viewpoint_ids=data["viewpoint_ids"],
            positions=data["positions"],
            viewing_directions=data["viewing_directions"],
            meta=meta,
        )

    def format_report(self) -> str:
        m = self.meta
        c = m.get("checks", {})
        samp = m.get("sampling", {})
        fmt = lambda a: "[" + " ".join(f"{x: .3f}" for x in a) + "]"
        lines = [
            f"Candidate viewpoints: {m.get('n')} "
            f"(paper Tables II/III: n in {{200, 300, 400}}, same set shared "
            f"across methods)",
            f"Seed: {m.get('seed')}",
            f"d_max: {m.get('d_max'):.3f} m  "
            f"(guide §6 initial 0.30 m, tuned per §6.4; "
            f"neighbor target median >= {MIN_NEIGHBORS_TARGET})",
            f"Source surface: {m.get('source_surface')}",
            "",
            "Positions:",
            f"  shell |p - center| in [{samp.get('d_low'):.3f}, {samp.get('d_high'):.3f}] m "
            f"(halfdiag + FOD + margin, guide §6.1)",
            f"  reachability (guide §6.3, replaces IK/FCL): "
            f"{c.get('reachability_ok')}/{m.get('n')} within "
            f"[{samp.get('r_min'):.2f}, {samp.get('r_max'):.2f}] m of base {fmt(samp.get('base', []))}",
            f"  below table (z < {samp.get('table_z'):.3f} m): {c.get('below_table')}",
            "",
            "Directions (Eq. 10 potential field, optical axis camera->scene):",
            f"  unit norm: max |1 - |v_hat|| = {c.get('direction_norm_max_err'):.1e}",
            f"  neighbors within d_max: median {m.get('n_neighbors_median')} "
            f"(target >= {MIN_NEIGHBORS_TARGET}), "
            f"fallback nearest-patch limit: {m.get('fallback_count')}/{m.get('n')}",
            "",
            "Geometry checks:",
            f"  min distance to workpiece bbox: {c.get('bbox_dist_min'):.3f} m "
            f"(all viewpoints outside)",
            f"  distance to nearest patch: min {c.get('surf_dist_min'):.3f} / "
            f"median {c.get('surf_dist_median'):.3f} / max {c.get('surf_dist_max'):.3f} m",
            f"    within FOD [{samp.get('fod_min')}, {samp.get('fod_max')}] m: "
            f"{100 * c.get('fod_in_frac'):.1f}%   "
            f"below: {100 * c.get('fod_below_frac'):.1f}%   "
            f"above: {100 * c.get('fod_above_frac'):.1f}%",
            f"  v_hat . (center - p) > 0: {100 * c.get('center_dot_frac'):.1f}% "
            f"(guide §6.4 diagnostic 2)",
            f"  v_hat . (nearest patch - p) > 0: {100 * c.get('surface_dot_frac'):.1f}%",
        ]
        return "\n".join(lines)

    def visualize(
        self,
        path: Union[str, Path],
        surface: Optional[DiscretizedSurface] = None,
        show: bool = False,
    ) -> Path:
        """Viewpoints + viewing-direction arrows (optionally over the M1
        surface) for human inspection, plus the viewpoint-to-surface
        distance histogram with the FOD window marked."""
        import matplotlib

        if not show:
            matplotlib.use("Agg", force=True)
        import matplotlib.pyplot as plt
        from matplotlib.colors import LinearSegmentedColormap
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection

        n = len(self)
        checks = self.meta.get("checks", {})
        samp = self.meta.get("sampling", {})
        # blue sequential ramp (light -> dark = first -> last viewpoint)
        cmap = LinearSegmentedColormap.from_list("vp", ["#cde2fb", "#184f95"])
        colors = cmap(np.linspace(0.0, 1.0, n))

        fig = plt.figure(figsize=(11.5, 4.6))
        fig.suptitle(
            f"Candidate viewpoints V (|V| = {n}, seed {self.meta.get('seed')}, "
            f"d_max {self.meta.get('d_max'):.2f} m)\n"
            "viewing direction = potential-field optical axis (Eq. 10, sign per "
            "paper05_replication.md §6.2)",
            fontsize=10,
        )

        ax = fig.add_subplot(1, 2, 1, projection="3d")
        if surface is not None:
            pc = Poly3DCollection(
                list(surface.vertices), facecolors="#d9dee6", alpha=0.6, edgecolors="none"
            )
            ax.add_collection3d(pc)
        ax.scatter(
            self.positions[:, 0],
            self.positions[:, 1],
            self.positions[:, 2],
            c=colors,
            s=12,
            depthshade=False,
        )
        diag = float(
            np.linalg.norm(
                np.asarray(self.meta["bounds_max"]) - np.asarray(self.meta["bounds_min"])
            )
        ) if "bounds_max" in self.meta else 1.0
        ax.quiver(
            self.positions[:, 0],
            self.positions[:, 1],
            self.positions[:, 2],
            self.viewing_directions[:, 0],
            self.viewing_directions[:, 1],
            self.viewing_directions[:, 2],
            length=0.15 * diag,
            color="#333333",
            linewidth=0.6,
        )
        ax.set_box_aspect(np.ptp(self.positions, axis=0))
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
        ax.set_title(
            f"{n} viewpoints (light->dark = id order) + viewing arrows\n"
            f"median neighbors within d_max: {checks.get('n_neighbors_median')}",
            fontsize=9,
        )
        ax.tick_params(labelsize=7)

        ax2 = fig.add_subplot(1, 2, 2)
        surf = checks.get("surf_distances")
        if surf is not None:
            ax2.hist(
                np.asarray(surf), bins=40, color="#2a78d6", alpha=0.85,
                edgecolor="white", linewidth=0.3,
            )
            for bound, label in [
                (samp.get("fod_min"), "FOD min"),
                (samp.get("fod_max"), "FOD max"),
            ]:
                if bound is not None:
                    ax2.axvline(bound, color="#8a2be2" if label == "FOD min" else "#b8860b",
                                linewidth=1.2, linestyle="--", label=label)
            ax2.set_xlabel("viewpoint-to-nearest-patch distance (m)")
            ax2.set_ylabel("count")
            ax2.set_title(
                f"distance to nearest surface patch\n"
                f"median {checks.get('surf_dist_median'):.3f} m, "
                f"{100 * checks.get('fod_in_frac'):.0f}% inside FOD window",
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
# top-level M2 entry point + validation
# ---------------------------------------------------------------------------


def validate(
    viewpoints: CandidateViewpointSet,
    surface: DiscretizedSurface,
    fod: tuple[float, float] = (0.40, 1.00),
    base: tuple[float, float, float] = (0.9, 0.0, 0.0),
    r_min: float = 0.55,
    r_max: float = 1.30,
    table_clearance: float = 0.05,
) -> dict:
    """M2 self-checks (task spec step 5). Pure geometry -- no visibility
    evaluation (M3 territory)."""
    n = len(viewpoints)
    p = viewpoints.positions
    v = viewpoints.viewing_directions
    bounds = surface.bounds
    center = bounds.mean(axis=0)
    table_z = float(bounds[0, 2] - table_clearance)
    base = np.asarray(base, dtype=np.float64)

    # distance to the workpiece bbox (exact; 0 = inside)
    bbox_dist = np.linalg.norm(p - np.clip(p, bounds[0], bounds[1]), axis=1)
    # surface-distance proxy: nearest patch center (M3 does the real thing)
    surf = np.linalg.norm(surface.centers[None, :, :] - p[:, None, :], axis=-1).min(axis=1)
    j = np.argmin(np.linalg.norm(surface.centers[None, :, :] - p[:, None, :], axis=-1), axis=1)
    to_nn = surface.centers[j] - p

    reach_ok = reachability_filter(p, base=tuple(base), r_min=r_min, r_max=r_max,
                                   z_min=table_z + 0.1)
    return {
        "count_ok": n == int(viewpoints.meta.get("n", n)),
        "positions_finite": bool(np.isfinite(p).all()),
        "directions_finite": bool(np.isfinite(v).all()),
        "direction_norm_max_err": float(np.abs(np.linalg.norm(v, axis=1) - 1.0).max()),
        "bbox_dist_min": float(bbox_dist.min()),
        "surf_dist_min": float(surf.min()),
        "surf_dist_median": float(np.median(surf)),
        "surf_dist_max": float(surf.max()),
        "surf_distances": surf.tolist(),
        "fod_in_frac": float(((surf >= fod[0]) & (surf <= fod[1])).mean()),
        "fod_below_frac": float((surf < fod[0]).mean()),
        "fod_above_frac": float((surf > fod[1]).mean()),
        "reachability_ok": int(reach_ok.sum()),
        "below_table": int((p[:, 2] < table_z).sum()),
        "center_dot_frac": float(((v * (center - p)).sum(axis=1) > 0.0).mean()),
        "surface_dot_frac": float(((v * to_nn).sum(axis=1) > 0.0).mean()),
    }


def generate_candidates(
    surface: DiscretizedSurface,
    n: int = 400,
    seed: int = 0,
    d_max: float = 0.30,
    fod: tuple[float, float] = (0.40, 1.00),
    distance_margin: float = 0.05,
    table_clearance: float = 0.05,
    reachability: Optional[dict] = None,
    source_surface: str = "in-memory",
) -> CandidateViewpointSet:
    """M2 black box: M1 surface patches -> candidate viewpoint set V.

    Position shell + table rejection (guide §6.1) and the reachability
    shell (guide §6.3) are applied by rejection sampling until n
    candidates are accepted; viewing directions come from Eq. (10).
    """
    reach = dict(reachability or {})
    base = tuple(reach.get("base", (0.9, 0.0, 0.0)))
    r_min = float(reach.get("r_min", 0.55))
    r_max = float(reach.get("r_max", 1.30))

    bounds = surface.bounds
    center = bounds.mean(axis=0)
    halfdiag = float(np.linalg.norm(bounds[1] - bounds[0]) / 2.0)
    table_z = float(bounds[0, 2] - table_clearance)
    low = halfdiag + fod[0] + distance_margin
    high = halfdiag + fod[1]

    rng = np.random.default_rng(seed)
    positions = np.empty((n, 3))
    i = 0
    for _ in range(n * MAX_ATTEMPTS_PER_POINT):
        d = float(rng.uniform(low, high))
        u = rng.normal(size=3)
        u /= np.linalg.norm(u)
        p = center + d * u
        if p[2] < table_z:
            continue
        if not reachability_filter(p[None, :], base=base, r_min=r_min, r_max=r_max,
                                   z_min=table_z + 0.1)[0]:
            continue
        positions[i] = p
        i += 1
        if i == n:
            break
    else:
        raise RuntimeError(
            f"could not generate {n} reachable candidates after "
            f"{n * MAX_ATTEMPTS_PER_POINT} attempts"
        )

    directions, n_nb, fallback = potential_directions(positions, surface.centers, d_max)
    ids = np.arange(n, dtype=np.int32)

    meta = {
        "module": "m2_sample",
        "reproduction_guide": "paper05_replication.md §6",
        "paper_eq": "Eq. (10): d_i = p_vp - p_patch_i; stored direction is the "
        "camera optical axis (camera -> scene), i.e. the negative of the "
        "paper's literal sum (sign resolution, see module docstring)",
        "source_surface": source_surface,
        "n": int(n),
        "seed": int(seed),
        "d_max": float(d_max),
        "sampling": {
            "method": "spherical shell around bbox center + table rejection "
            "(guide §6.1); reachability shell replaces the paper's IK+FCL "
            "(guide §6.3, simplification #2)",
            "d_low": low,
            "d_high": high,
            "halfdiag": halfdiag,
            "fod_min": fod[0],
            "fod_max": fod[1],
            "distance_margin": distance_margin,
            "table_clearance": table_clearance,
            "table_z": table_z,
            "base": list(base),
            "r_min": r_min,
            "r_max": r_max,
        },
        "bounds_min": bounds[0].tolist(),
        "bounds_max": bounds[1].tolist(),
        "n_neighbors_median": int(np.median(n_nb)),
        "fallback_count": int(fallback.sum()),
        "created": _dt.datetime.now().isoformat(timespec="seconds"),
    }
    set_ = CandidateViewpointSet(ids, positions, directions, meta)
    meta["checks"] = validate(set_, surface, fod=fod, base=base, r_min=r_min,
                              r_max=r_max, table_clearance=table_clearance)
    return CandidateViewpointSet(ids, positions, directions, meta)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        prog="m2_sample",
        description="M2 candidate viewpoint generation: M1 patches -> V (Eq. 10 potential field)",
    )
    ap.add_argument("--surface", required=True, help="M1 output S_*.npz")
    ap.add_argument("--n", type=int, default=None, help="candidate count (paper: 200/300/400)")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--d-max", type=float, default=None, metavar="M")
    ap.add_argument("--tune-dmax", action="store_true",
                    help="auto-set d_max via the guide §6.4 diagnostic (median >= 10 neighbors)")
    ap.add_argument("--config", type=Path, default=None)
    ap.add_argument("--out", type=Path, default=Path("data/outputs"))
    ap.add_argument("--no-viz", action="store_true")
    args = ap.parse_args(argv)

    cfg = {}
    if args.config is not None:
        cfg = load_config(args.config).get("sampling", {})
    n = args.n or 400
    seed = args.seed if args.seed is not None else int(cfg.get("seed", 0))
    d_max = args.d_max if args.d_max is not None else float(cfg.get("d_max", 0.30))
    reach = dict(cfg.get("reachability", {}))
    pos = dict(cfg.get("position", {}))

    surface = DiscretizedSurface.load(args.surface)
    if args.tune_dmax:
        # guide §6.4 diagnostic 1: median neighbor count >= 10
        probe = generate_candidates(surface, n=n, seed=seed, d_max=d_max)
        tuned, _ = recommend_d_max(probe.positions, surface.centers)
        print(f"d_max tuning: guide initial {d_max:.3f} m -> tuned {tuned:.3f} m "
              f"(median neighbors >= {MIN_NEIGHBORS_TARGET}; same seed, same positions)")
        d_max = tuned
    viewpoints = generate_candidates(
        surface,
        n=n,
        seed=seed,
        d_max=d_max,
        distance_margin=float(pos.get("distance_margin", 0.05)),
        table_clearance=float(pos.get("table_clearance", 0.05)),
        reachability=reach or None,
        source_surface=str(args.surface),
    )

    args.out.mkdir(parents=True, exist_ok=True)
    stem = args.out / f"V_{Path(args.surface).stem.replace('S_', '')}_{n}"
    npz_path, json_path = viewpoints.save(stem)
    if not args.no_viz:
        viewpoints.visualize(stem.with_suffix(".png"), surface=surface)

    print(viewpoints.format_report())
    print(f"\nArtifacts: {npz_path}  {json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

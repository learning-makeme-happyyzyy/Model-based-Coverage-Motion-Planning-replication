"""M4: travel cost computation (paper_05 replication).

Follows paper05_replication.md §8.

Paper fact (verbatim from the PDF): the traveling cost is the collision-free
robot path *time* between the robot poses at two viewpoints -- computed with
MoveIt/OMPL (RRT-Connect) + FCL, parametrized by cubic splines with actuator
speed/acceleration data -- stored in the edges of the undirected graph G(V, E).

Reproduction model (guide §8, simplification #1 -- the paper gives no
coefficients, so these are reproduction-chosen and reported as such):

    e(v_i, v_j) = alpha * ||p_i - p_j|| + beta * arccos(v_hat_i . v_hat_j)

  * alpha = 1.2 s/m, beta = 0.4 s/rad (guide §8). Rationale: industrial arm
    end-effector effective speed ~0.8-1.5 m/s plus an orientation-adjustment
    overhead. Both terms are times (s/m * m = s; s/rad * rad = s), so they
    add. The guide's §8 calibration trick (retune alpha once M5 runs so the
    GA-SCTSP cycle time lands in 15-30 s) is for the M5 stage.
  * theta_ij = arccos(clip(v_hat_i . v_hat_j, -1, 1)) is the angle between
    the two OPTICAL AXES (the M2 viewing directions). Roll about the optical
    axis is NOT modeled: the paper leaves roll free for sensing, and the
    guide's formula only contains v_hat -- a full camera-rotation angle is
    not what the guide defines. Recorded as a limitation.
  * Symmetry: e(v_i, v_j) = e(v_j, v_i) (the paper's graph is undirected);
    C_ii = 0. Numerically C is exactly symmetric and zero on the diagonal.
  * The Euclidean distance is a proxy for the real collision-free path
    length -- absolute cycle times are NOT comparable with the paper's
    (guide §0, deviation #1); relative patterns are.

Home pose (guide §8; the paper only shows it in Fig. 2 without numbers):
    p_h = object center + (0.9, 0, 0.6) m, viewing direction pointing at
    the object center: v_hat_h = normalize(center - p_h). Roll unspecified
    and unused (minimal assumption; the proxy only needs the optical axis).
    home->viewpoint and viewpoint->home use the same proxy formula; the two
    arrays are stored separately so the direction semantics survive even if
    a future model is asymmetric.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import time
from pathlib import Path
from typing import Optional, Union

import numpy as np

from src.m1_discretize import load_config
from src.m2_sample import CandidateViewpointSet

__all__ = [
    "TravelCosts",
    "angle_from_dots",
    "travel_cost_matrix",
    "travel_cost_vector",
    "home_pose",
    "compute_travel_costs",
    "main",
]

DIRECTION_NORM_TOL = 1e-6  # unit-direction validation tolerance


# ---------------------------------------------------------------------------
# pure math helpers (hand-testable)
# ---------------------------------------------------------------------------


def angle_from_dots(dots: np.ndarray) -> np.ndarray:
    """arccos with clipping: angle between unit directions, in [0, pi] rad.

    The clip absorbs floating-point overflow of the dot product (e.g.
    |dot| = 1 + 1e-16 from rounding) instead of producing NaN.
    """
    dots = np.asarray(dots, dtype=np.float64)
    return np.arccos(np.clip(dots, -1.0, 1.0))


def _validate_poses(
    positions: np.ndarray, directions: np.ndarray, n_expected: Optional[int] = None
) -> tuple[np.ndarray, np.ndarray]:
    """Shape/finite/unit checks; raises ValueError on invalid input instead
    of silently producing wrong costs."""
    positions = np.asarray(positions, dtype=np.float64)
    directions = np.asarray(directions, dtype=np.float64)
    if positions.ndim != 2 or positions.shape[1] != 3:
        raise ValueError(f"positions must be (n, 3), got {positions.shape}")
    if directions.shape != positions.shape:
        raise ValueError(
            f"directions shape {directions.shape} != positions shape {positions.shape}"
        )
    if n_expected is not None and len(positions) != n_expected:
        raise ValueError(f"expected {n_expected} poses, got {len(positions)}")
    if not np.isfinite(positions).all():
        raise ValueError("positions contain NaN/Inf")
    if not np.isfinite(directions).all():
        raise ValueError("directions contain NaN/Inf")
    norms = np.linalg.norm(directions, axis=1)
    if not np.allclose(norms, 1.0, atol=DIRECTION_NORM_TOL):
        raise ValueError(
            f"directions must be unit vectors (max |1 - ||v||| = "
            f"{np.abs(norms - 1.0).max():.2e} > {DIRECTION_NORM_TOL})"
        )
    return positions, directions


def travel_cost_matrix(
    positions: np.ndarray,
    directions: np.ndarray,
    alpha: float = 1.2,
    beta: float = 0.4,
) -> np.ndarray:
    """C in R^{n x n} (seconds): C[i, j] = alpha*d_ij + beta*theta_ij.

    d_ij = ||p_i - p_j|| (m), theta_ij = arccos(clip(v_i . v_j)) (rad).
    Exactly symmetric; the diagonal is set to exactly 0 by definition
    (e(v_i, v_i) = 0 -- without this, unit-normalization rounding of the
    input directions would leave ~1e-8 s residues from arccos(1 - 1e-16)).
    """
    positions, directions = _validate_poses(positions, directions)
    d = np.linalg.norm(positions[:, None, :] - positions[None, :, :], axis=2)
    theta = angle_from_dots(directions @ directions.T)
    C = alpha * d + beta * theta
    np.fill_diagonal(C, 0.0)
    return C


def travel_cost_vector(
    position: np.ndarray,
    direction: np.ndarray,
    positions: np.ndarray,
    directions: np.ndarray,
    alpha: float = 1.2,
    beta: float = 0.4,
) -> np.ndarray:
    """Cost from one pose (e.g. home) to every viewpoint, (n,) seconds."""
    position = np.asarray(position, dtype=np.float64)
    direction = np.asarray(direction, dtype=np.float64)
    positions, directions = _validate_poses(positions, directions)
    if position.shape != (3,) or direction.shape != (3,):
        raise ValueError(f"pose must be (3,), got {position.shape}/{direction.shape}")
    if not np.isfinite(position).all() or not np.isfinite(direction).all():
        raise ValueError("pose contains NaN/Inf")
    if not np.isclose(np.linalg.norm(direction), 1.0, atol=DIRECTION_NORM_TOL):
        raise ValueError(f"pose direction must be unit, got ||v|| = {np.linalg.norm(direction)}")
    d = np.linalg.norm(positions - position, axis=1)
    theta = angle_from_dots(directions @ direction)
    return alpha * d + beta * theta


def home_pose(
    object_center: np.ndarray, offset: tuple[float, float, float] = (0.9, 0.0, 0.6)
) -> tuple[np.ndarray, np.ndarray]:
    """Home pose (guide §8): position = center + offset; viewing direction
    points at the object center (roll unspecified and unused)."""
    center = np.asarray(object_center, dtype=np.float64)
    offset = np.asarray(offset, dtype=np.float64)
    if center.shape != (3,) or offset.shape != (3,):
        raise ValueError(f"center/offset must be (3,), got {center.shape}/{offset.shape}")
    if np.linalg.norm(offset) == 0.0:
        raise ValueError("home offset must be non-zero")
    position = center + offset
    direction = center - position
    direction = direction / np.linalg.norm(direction)
    return position, direction


# ---------------------------------------------------------------------------
# container
# ---------------------------------------------------------------------------


class TravelCosts:
    """E (n x n) + home legs, with provenance, statistics and serialization.

    Indexing contract (for M5): C[i, j] is the cost viewpoints[i] ->
    viewpoints[j] (input order); home_to_viewpoint[i] is home -> v_i and
    viewpoint_to_home[i] is v_i -> home.
    """

    def __init__(
        self,
        viewpoint_ids: np.ndarray,
        matrix: np.ndarray,
        home_to_viewpoint: np.ndarray,
        viewpoint_to_home: np.ndarray,
        home_position: np.ndarray,
        home_direction: np.ndarray,
        meta: dict,
    ):
        n = len(viewpoint_ids)
        for name, arr, shape in [
            ("matrix", matrix, (n, n)),
            ("home_to_viewpoint", home_to_viewpoint, (n,)),
            ("viewpoint_to_home", viewpoint_to_home, (n,)),
        ]:
            if arr.shape != shape:
                raise ValueError(f"{name} has shape {arr.shape}, expected {shape}")
        self.viewpoint_ids = np.asarray(viewpoint_ids, dtype=np.int32)
        self.matrix = np.asarray(matrix, dtype=np.float64)
        self.home_to_viewpoint = np.asarray(home_to_viewpoint, dtype=np.float64)
        self.viewpoint_to_home = np.asarray(viewpoint_to_home, dtype=np.float64)
        self.home_position = np.asarray(home_position, dtype=np.float64)
        self.home_direction = np.asarray(home_direction, dtype=np.float64)
        self.meta = dict(meta)

    def __len__(self) -> int:
        return len(self.viewpoint_ids)

    def stats(self) -> dict:
        n = len(self)
        C = self.matrix
        diag = np.diag(C)
        sym = np.abs(C - C.T).max()
        off = C[~np.eye(n, dtype=bool)]
        return {
            "shape": list(C.shape),
            "units": "seconds",
            "diag_max_err": float(np.abs(diag).max()),
            "symmetry_max_err": float(sym),
            "offdiag": {
                "min": float(off.min()),
                "median": float(np.median(off)),
                "mean": float(off.mean()),
                "max": float(off.max()),
            },
            "home_to_viewpoint": {
                "min": float(self.home_to_viewpoint.min()),
                "median": float(np.median(self.home_to_viewpoint)),
                "mean": float(self.home_to_viewpoint.mean()),
                "max": float(self.home_to_viewpoint.max()),
            },
            "viewpoint_to_home": {
                "min": float(self.viewpoint_to_home.min()),
                "median": float(np.median(self.viewpoint_to_home)),
                "mean": float(self.viewpoint_to_home.mean()),
                "max": float(self.viewpoint_to_home.max()),
            },
            "home_leg_max_diff": float(
                np.abs(self.home_to_viewpoint - self.viewpoint_to_home).max()
            ),
        }

    def format_report(self) -> str:
        s = self.stats()
        m = self.meta
        fmt = lambda a: "[" + " ".join(f"{x:.3f}" for x in a) + "]"
        lines = [
            f"Travel-cost matrix E: {s['shape'][0]} x {s['shape'][1]} ({s['units']})",
            f"Model: e(v_i, v_j) = alpha * ||p_i - p_j|| + beta * arccos(v_i . v_j)",
            f"  alpha = {m.get('alpha')} s/m, beta = {m.get('beta')} s/rad "
            f"(guide §8, reproduction-chosen; paper uses MoveIt/RRT + cubic spline)",
            f"Home pose: p_h = {fmt(m.get('home_position', []))}, "
            f"v_hat_h = {fmt(m.get('home_direction', []))} "
            f"(object center + offset {m.get('home_offset')}, looking at the center)",
            f"Source: {m.get('source_viewpoints')}",
            "",
            f"Diagonal check: max |E[i, i]| = {s['diag_max_err']:.2e} s",
            f"Symmetry check: max |E - E^T| = {s['symmetry_max_err']:.2e} s",
            f"Off-diagonal: min {s['offdiag']['min']:.3f} / median {s['offdiag']['median']:.3f} / "
            f"mean {s['offdiag']['mean']:.3f} / max {s['offdiag']['max']:.3f} s "
            f"(guide §8 sanity band 0.2-3 s)",
            f"Home -> viewpoint: min {s['home_to_viewpoint']['min']:.3f} / "
            f"median {s['home_to_viewpoint']['median']:.3f} / "
            f"max {s['home_to_viewpoint']['max']:.3f} s",
            f"Viewpoint -> home: min {s['viewpoint_to_home']['min']:.3f} / "
            f"median {s['viewpoint_to_home']['median']:.3f} / "
            f"max {s['viewpoint_to_home']['max']:.3f} s "
            f"(symmetric model: max leg diff {s['home_leg_max_diff']:.2e} s)",
            f"Elapsed: {m.get('elapsed_s')} s",
        ]
        return "\n".join(lines)

    def save(self, stem: Union[str, Path]) -> tuple[Path, Path]:
        stem = Path(stem)
        np.savez(
            stem.with_suffix(".npz"),
            viewpoint_ids=self.viewpoint_ids,
            matrix=self.matrix,
            home_to_viewpoint=self.home_to_viewpoint,
            viewpoint_to_home=self.viewpoint_to_home,
            home_position=self.home_position,
            home_direction=self.home_direction,
        )
        meta = dict(self.meta)
        meta["stats"] = self.stats()
        meta_path = stem.with_suffix(".json")
        meta_path.write_text(json.dumps(meta, indent=2))
        return stem.with_suffix(".npz"), meta_path

    @classmethod
    def load(cls, stem: Union[str, Path]) -> "TravelCosts":
        stem = Path(stem)
        data = np.load(stem.with_suffix(".npz"))
        meta = json.loads(stem.with_suffix(".json").read_text())
        return cls(
            viewpoint_ids=data["viewpoint_ids"],
            matrix=data["matrix"],
            home_to_viewpoint=data["home_to_viewpoint"],
            viewpoint_to_home=data["viewpoint_to_home"],
            home_position=data["home_position"],
            home_direction=data["home_direction"],
            meta=meta,
        )

    def visualize(
        self,
        path: Union[str, Path],
        viewpoints: Optional[CandidateViewpointSet] = None,
        show: bool = False,
    ) -> Path:
        """Heatmap of E, off-diagonal cost distribution, and a 3D scene
        with viewpoints colored by their home cost + the home pose.
        Viewpoint positions are not duplicated in TravelCosts; pass the
        source candidate set for the 3D panel."""
        import matplotlib

        if not show:
            matplotlib.use("Agg", force=True)
        import matplotlib.pyplot as plt
        from matplotlib.colors import LinearSegmentedColormap

        n = len(self)
        blue = LinearSegmentedColormap.from_list("blue", ["#cde2fb", "#184f95"])

        fig = plt.figure(figsize=(14.5, 4.6))
        fig.suptitle(
            f"Travel costs E ({n} x {n}, seconds) - "
            f"e = {self.meta.get('alpha')} * d + {self.meta.get('beta')} * theta",
            fontsize=10,
        )

        ax = fig.add_subplot(1, 3, 1)
        im = ax.imshow(self.matrix, aspect="auto", cmap=blue)
        ax.set_xlabel("viewpoint index j")
        ax.set_ylabel("viewpoint index i")
        ax.set_title("cost matrix heatmap (E[i, j] seconds)", fontsize=9)
        fig.colorbar(im, ax=ax, shrink=0.8)
        ax.tick_params(labelsize=7)

        ax2 = fig.add_subplot(1, 3, 2)
        off = self.matrix[~np.eye(n, dtype=bool)]
        ax2.hist(off, bins=50, color="#2a78d6", alpha=0.85,
                 edgecolor="white", linewidth=0.3, label="viewpoint-viewpoint")
        ax2.hist(self.viewpoint_to_home, bins=30, color="#8a6d3b", alpha=0.6,
                 edgecolor="white", linewidth=0.3, label="viewpoint-home")
        ax2.set_xlabel("travel cost (s)")
        ax2.set_ylabel("count")
        ax2.set_title("cost distribution (off-diagonal + home legs)", fontsize=9)
        ax2.legend(fontsize=8, frameon=False)
        ax2.spines[["top", "right"]].set_visible(False)
        ax2.tick_params(labelsize=7)

        ax3 = fig.add_subplot(1, 3, 3, projection="3d")
        if viewpoints is not None:
            colors = blue((self.viewpoint_to_home - self.viewpoint_to_home.min())
                          / max(np.ptp(self.viewpoint_to_home), 1e-12))
            ax3.scatter(viewpoints.positions[:, 0], viewpoints.positions[:, 1],
                        viewpoints.positions[:, 2], c=colors, s=10, depthshade=False)
        ax3.scatter(*self.home_position, color="#c92a2a", s=60, marker="*")
        ax3.quiver(*self.home_position, *self.home_direction,
                   length=0.3, color="#c92a2a")
        center = self.home_position + self.home_direction * np.linalg.norm(
            self.meta.get("home_offset", [0.9, 0.0, 0.6]))
        ax3.scatter(*center, color="#333333", s=20, marker="o")
        all_pts = np.array([self.home_position, center])
        if viewpoints is not None:
            all_pts = np.vstack([viewpoints.positions, all_pts])
        ax3.set_box_aspect(np.ptp(all_pts, axis=0))
        ax3.set_title("viewpoints (blue = home cost) + home pose (red star)",
                      fontsize=9)
        ax3.set_xlabel("x (m)"); ax3.set_ylabel("y (m)"); ax3.set_zlabel("z (m)")
        ax3.tick_params(labelsize=6)

        fig.tight_layout(rect=(0, 0, 1, 0.93))
        path = Path(path)
        fig.savefig(path, dpi=150, bbox_inches="tight")
        plt.close(fig)
        return path


# ---------------------------------------------------------------------------
# top-level M4 entry point
# ---------------------------------------------------------------------------


def compute_travel_costs(
    viewpoints: CandidateViewpointSet,
    object_center: Optional[np.ndarray] = None,
    alpha: float = 1.2,
    beta: float = 0.4,
    home_offset: tuple[float, float, float] = (0.9, 0.0, 0.6),
    source_viewpoints: str = "in-memory",
) -> TravelCosts:
    """M4 black box: V -> (E, home legs). Does not touch M2's candidates and
    does not use M3's visibility (pure travel proxy)."""
    t0 = time.time()
    n = len(viewpoints)
    positions, directions = _validate_poses(
        viewpoints.positions, viewpoints.viewing_directions, n_expected=n
    )
    if object_center is None:
        if "bounds_min" not in viewpoints.meta or "bounds_max" not in viewpoints.meta:
            raise ValueError(
                "object center not given and viewpoint meta has no bbox; "
                "pass object_center explicitly"
            )
        object_center = (
            np.asarray(viewpoints.meta["bounds_min"], dtype=np.float64)
            + np.asarray(viewpoints.meta["bounds_max"], dtype=np.float64)
        ) / 2.0
    object_center = np.asarray(object_center, dtype=np.float64)

    matrix = travel_cost_matrix(positions, directions, alpha, beta)
    hp, hd = home_pose(object_center, home_offset)
    h2v = travel_cost_vector(hp, hd, positions, directions, alpha, beta)
    v2h = travel_cost_vector(hp, hd, positions, directions, alpha, beta)

    meta = {
        "module": "m4_travel_cost",
        "reproduction_guide": "paper05_replication.md §8",
        "paper_fact": "traveling cost = collision-free robot path time "
        "(MoveIt/OMPL RRT-Connect + cubic spline), edges of undirected G(V, E)",
        "model": "e(v_i, v_j) = alpha * ||p_i - p_j|| + beta * arccos(v_i . v_j) "
        "(guide §8, simplification #1; theta = optical-axis angle, roll not modeled)",
        "provenance": {
            "paper": "undirected graph G(V,E); e_ij = collision-free path time "
            "between robot poses at viewpoints",
            "reproduction": "alpha = 1.2 s/m, beta = 0.4 s/rad, home pose = "
            "object center + (0.9, 0, 0.6) m looking at the center (guide §8; "
            "the paper gives no coefficients and only shows the home pose in Fig. 2)",
            "engineering": "arccos input clipped to [-1, 1]; unit-direction "
            "validation (atol 1e-6); Euclidean distance != real arm path length",
        },
        "alpha": alpha,
        "beta": beta,
        "units": "seconds (positions in m, angles in rad)",
        "home_offset": list(home_offset),
        "home_position": hp.tolist(),
        "home_direction": hd.tolist(),
        "object_center": object_center.tolist(),
        "source_viewpoints": source_viewpoints,
        "indexing": "E[i, j] = v_i -> v_j in input order; "
        "home_to_viewpoint[i] = home -> v_i; viewpoint_to_home[i] = v_i -> home",
        "elapsed_s": round(time.time() - t0, 4),
        "created": _dt.datetime.now().isoformat(timespec="seconds"),
    }
    return TravelCosts(
        viewpoints.viewpoint_ids, matrix, h2v, v2h, hp, hd, meta
    )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        prog="m4_travel_cost",
        description="M4 travel cost: V -> (E, home legs) via the guide §8 proxy",
    )
    ap.add_argument("--viewpoints", required=True, help="M2 output V_*.npz")
    ap.add_argument("--config", type=Path, default=None)
    ap.add_argument("--object-center", type=float, nargs=3, default=None,
                    metavar=("X", "Y", "Z"))
    ap.add_argument("--out", type=Path, default=Path("data/outputs"))
    ap.add_argument("--no-viz", action="store_true")
    args = ap.parse_args(argv)

    cfg = {}
    if args.config is not None:
        cfg = load_config(args.config)
    tp = cfg.get("travel_proxy", {})
    alpha = float(tp.get("alpha", 1.2))
    beta = float(tp.get("beta", 0.4))
    home_offset = tuple(tp.get("home_pose", {}).get("offset", [0.9, 0.0, 0.6]))

    viewpoints = CandidateViewpointSet.load(args.viewpoints)
    costs = compute_travel_costs(
        viewpoints,
        object_center=None if args.object_center is None else np.asarray(args.object_center),
        alpha=alpha,
        beta=beta,
        home_offset=home_offset,
        source_viewpoints=str(args.viewpoints),
    )

    args.out.mkdir(parents=True, exist_ok=True)
    name = Path(args.viewpoints).stem.replace("V_", "")
    stem = args.out / f"E_{name}"
    npz_path, json_path = costs.save(stem)
    if not args.no_viz:
        costs.visualize(stem.with_suffix(".png"), viewpoints=viewpoints)

    print(costs.format_report())
    print(f"\nArtifacts: {npz_path}  {json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

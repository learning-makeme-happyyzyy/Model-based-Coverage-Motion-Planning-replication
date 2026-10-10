"""Unit tests for M2 candidate viewpoint generation (paper05_replication.md §6).

Covers the task-spec validations: counts (200/300/400), finite values,
unit directions, deterministic seed, the shared candidate set (prefix
property), and geometric relations on simple workpieces.
"""

import numpy as np
import pytest
import trimesh

from src.m1_discretize import DiscretizedSurface, discretize, patches_from_mesh
from src.m2_sample import (
    CandidateViewpointSet,
    generate_candidates,
    potential_direction,
    potential_directions,
    reachability_filter,
    recommend_d_max,
    sample_positions,
    validate,
    viewing_frames,
)

BOX = trimesh.creation.box(extents=[2.0, 1.0, 1.0])
BASE = (0.9, 0.0, 0.0)


@pytest.fixture(scope="module")
def box_surface() -> DiscretizedSurface:
    return discretize(BOX, m=300)  # M1 -> M2 integration


@pytest.fixture(scope="module")
def plane_surface() -> DiscretizedSurface:
    """Patches on the z=0 plane with centers on the symmetric grid
    {+-0.15, +-0.05}^2 (guide §6.4 hand case; exact x/y symmetry in fp)."""
    grid = np.array([-0.15, -0.05, 0.05, 0.15])
    centers = np.array([[x, y, 0.0] for x in grid for y in grid])
    eps = 0.01
    vertices = np.stack(
        [centers, centers + [eps, 0, 0], centers + [0, eps, 0]], axis=1
    )  # (m, 3, 3) tiny triangles around each center
    m = len(centers)
    return DiscretizedSurface(
        np.arange(m, dtype=np.int32),
        vertices,
        centers,
        np.tile([0.0, 0.0, 1.0], (m, 1)),
        np.full(m, 0.5 * eps**2),
        np.array([[grid.min(), grid.min(), -eps], [grid.max(), grid.max(), eps]]),
        {"source_mesh": {"area": float(2 * 0.3**2)}},
    )


# ---------------------------------------------------------------------------
# counts / determinism / shared candidate set
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("n", [200, 300, 400])
def test_candidate_count(box_surface, n):
    vps = generate_candidates(box_surface, n=n, seed=0, d_max=1.0)
    assert len(vps) == n
    assert len(vps.positions) == n
    assert len(vps.viewing_directions) == n
    np.testing.assert_array_equal(vps.viewpoint_ids, np.arange(n))


def test_finite_values(box_surface):
    vps = generate_candidates(box_surface, n=200, seed=0, d_max=1.0)
    assert np.isfinite(vps.positions).all()
    assert np.isfinite(vps.viewing_directions).all()


def test_directions_unit_norm(box_surface):
    vps = generate_candidates(box_surface, n=200, seed=0, d_max=1.0)
    assert np.abs(np.linalg.norm(vps.viewing_directions, axis=1) - 1.0).max() < 1e-12


def test_same_seed_reproducible(box_surface):
    a = generate_candidates(box_surface, n=300, seed=7, d_max=1.0)
    b = generate_candidates(box_surface, n=300, seed=7, d_max=1.0)
    np.testing.assert_array_equal(a.positions, b.positions)
    np.testing.assert_array_equal(a.viewing_directions, b.viewing_directions)


def test_different_seed_differs(box_surface):
    a = generate_candidates(box_surface, n=300, seed=1, d_max=1.0)
    b = generate_candidates(box_surface, n=300, seed=2, d_max=1.0)
    assert not np.allclose(a.positions, b.positions)


def test_shared_candidate_set_prefix(box_surface):
    # paper: "the same set of sampled candidate viewpoints are used for
    # the three different methods" -> n=200/300 are prefixes of n=400
    master = generate_candidates(box_surface, n=400, seed=0, d_max=1.0)
    for n in [200, 300]:
        sub = master.subset(n)
        direct = generate_candidates(box_surface, n=n, seed=0, d_max=1.0)
        np.testing.assert_array_equal(sub.positions, direct.positions)
        np.testing.assert_array_equal(sub.viewing_directions, direct.viewing_directions)
        np.testing.assert_array_equal(sub.positions, master.positions[:n])


# ---------------------------------------------------------------------------
# position geometry (shell / table / reachability / outside the part)
# ---------------------------------------------------------------------------


def test_positions_geometry(box_surface):
    vps = generate_candidates(box_surface, n=400, seed=0, d_max=1.0)
    p = vps.positions
    bounds = box_surface.bounds
    center = bounds.mean(axis=0)
    halfdiag = np.linalg.norm(bounds[1] - bounds[0]) / 2.0
    low, high = halfdiag + 0.40 + 0.05, halfdiag + 1.00
    # spherical shell distance from the bbox center
    d = np.linalg.norm(p - center, axis=1)
    assert (d >= low).all() and (d <= high).all()
    # reachability shell around the robot base
    db = np.linalg.norm(p - np.array(BASE), axis=1)
    assert (db >= 0.55).all() and (db <= 1.30).all()
    # above the table plane + clearance
    table_z = bounds[0, 2] - 0.05
    assert (p[:, 2] >= table_z + 0.1).all()
    # all viewpoints outside the workpiece bbox
    bbox_dist = np.linalg.norm(p - np.clip(p, bounds[0], bounds[1]), axis=1)
    assert (bbox_dist > 0).all()


def test_reachability_filter_hand_cases():
    # distances to base (0.9, 0, 0): 0.6 / 0.5 / 1.4 / 0.6
    pts = np.array([[1.5, 0.0, 0.0], [0.4, 0.0, 0.0], [2.3, 0.0, 0.0], [1.5, 0.0, 0.0]])
    ok = reachability_filter(pts, base=BASE, r_min=0.55, r_max=1.30)
    np.testing.assert_array_equal(ok, [True, False, False, True])
    ok_z = reachability_filter(pts, base=BASE, r_min=0.55, r_max=1.30, z_min=0.5)
    np.testing.assert_array_equal(ok_z, [False, False, False, False])


def test_sample_positions_shell():
    rng = np.random.default_rng(0)
    center = np.array([0.0, 0.0, 0.0])
    p = sample_positions(center, 0.30, 200, rng, table_z=-0.2)
    d = np.linalg.norm(p - center, axis=1)
    assert (d >= 0.30 + 0.40 + 0.05).all() and (d <= 0.30 + 1.00).all()
    assert (p[:, 2] >= -0.2).all()


# ---------------------------------------------------------------------------
# Eq. (10) potential field: hand-calculated cases
# ---------------------------------------------------------------------------


def test_potential_direction_plane_hand_calc(plane_surface):
    # guide §6.4: camera at (0.5, 0, 0.2), symmetric patch grid on z=0.
    # The 1/d^2 weights make the field dominated by the *nearest* patches,
    # so the guide's "v_hat points near the plane centroid" is approximate
    # ("约 normalize(-0.5, 0, -0.2)"). Exact assertions: y-component cancels
    # to 0 by symmetry, and v_hat is close to the centroid direction.
    p = np.array([0.5, 0.0, 0.2])
    v, n_nb, fallback = potential_direction(p, plane_surface.centers, d_max=1.0)
    assert n_nb == len(plane_surface)
    assert not fallback
    assert abs(v[1]) < 1e-12  # exact cancellation under y -> -y symmetry
    centroid_dir = np.array([-0.5, 0.0, -0.2]) / np.linalg.norm([-0.5, 0.0, -0.2])
    assert v @ centroid_dir > 0.95
    # optical axis points camera -> surface
    assert v @ (plane_surface.centers[0] - p) > 0


def test_potential_direction_single_patch():
    # one patch at the origin, camera at (0.5, 0, 0.2)
    centers = np.array([[0.0, 0.0, 0.0]])
    v, n_nb, fallback = potential_direction(np.array([0.5, 0.0, 0.2]), centers, d_max=1.0)
    np.testing.assert_allclose(v, np.array([-0.5, 0.0, -0.2]) / np.linalg.norm([-0.5, 0.0, -0.2]),
                               atol=1e-12)
    assert n_nb == 1 and not fallback


def test_potential_direction_excludes_beyond_d_max():
    # patches 0.5 m away, d_max = 0.3 -> empty neighborhood -> nearest-patch
    # limit (d_max -> 0+ of Eq. 10), flagged as fallback. The nearest patch
    # is at the origin, camera at (0.5, 0, 0): optical axis = (-1, 0, 0).
    centers = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    p = np.array([0.5, 0.0, 0.0])
    v, n_nb, fallback = potential_direction(p, centers, d_max=0.3)
    assert n_nb == 0 and fallback
    np.testing.assert_allclose(v, [-1.0, 0.0, 0.0], atol=1e-12)  # toward nearest patch


def test_potential_directions_vectorized_matches_scalar(box_surface):
    rng = np.random.default_rng(3)
    p = rng.normal(size=(5, 3))
    v_vec, nb_vec, fb_vec = potential_directions(p, box_surface.centers, d_max=1.0)
    for i in range(5):
        v, n_nb, fallback = potential_direction(p[i], box_surface.centers, d_max=1.0)
        np.testing.assert_allclose(v_vec[i], v, atol=1e-12)
        assert nb_vec[i] == n_nb and fb_vec[i] == fallback


# ---------------------------------------------------------------------------
# d_max recommendation (guide §6.4 diagnostic)
# ---------------------------------------------------------------------------


def test_recommend_d_max_achieves_median_target(box_surface):
    vps = generate_candidates(box_surface, n=200, seed=0, d_max=0.30)
    rec, _ = recommend_d_max(vps.positions, box_surface.centers, min_neighbors=10)
    assert np.isfinite(rec) and rec > 0
    _, n_nb, fallback = potential_directions(vps.positions, box_surface.centers, rec)
    assert np.median(n_nb) >= 10
    # viewpoints with >= 10 neighbors cannot be in fallback, so the
    # fallback fraction is bounded by 1 - median share (<= 50%)
    assert fallback.sum() <= 0.5 * len(vps)


# ---------------------------------------------------------------------------
# viewing frame convention (roll free in the paper; guide §7.1 for M3)
# ---------------------------------------------------------------------------


def test_viewing_frames_convention():
    dirs = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
    frames = viewing_frames(dirs)
    for i in range(3):
        x, y, z = frames[i]
        np.testing.assert_allclose(z, dirs[i] / np.linalg.norm(dirs[i]), atol=1e-12)
        assert abs(x @ z) < 1e-12 and abs(y @ z) < 1e-12
        np.testing.assert_allclose(y, np.cross(z, x), atol=1e-12)
        for row in [x, y, z]:
            assert abs(np.linalg.norm(row) - 1.0) < 1e-12


# ---------------------------------------------------------------------------
# validation report + serialization
# ---------------------------------------------------------------------------


def test_validation_checks(box_surface):
    vps = generate_candidates(box_surface, n=200, seed=0, d_max=1.0)
    checks = vps.meta["checks"]
    assert checks["count_ok"]
    assert checks["positions_finite"] and checks["directions_finite"]
    assert checks["direction_norm_max_err"] < 1e-12
    assert checks["bbox_dist_min"] > 0.0  # no viewpoint inside the workpiece
    assert checks["reachability_ok"] == 200
    assert checks["below_table"] == 0
    assert checks["center_dot_frac"] == pytest.approx(1.0)  # guide diag 2
    assert checks["surface_dot_frac"] == pytest.approx(1.0)
    # recompute independently
    same = validate(vps, box_surface)
    assert same["count_ok"] and same["bbox_dist_min"] > 0.0


def test_serialization_roundtrip(tmp_path, box_surface):
    vps = generate_candidates(box_surface, n=100, seed=0, d_max=1.0)
    npz_path, json_path = vps.save(tmp_path / "V_test")
    assert npz_path.exists() and json_path.exists()
    loaded = CandidateViewpointSet.load(tmp_path / "V_test")
    assert len(loaded) == 100
    np.testing.assert_array_equal(loaded.positions, vps.positions)
    np.testing.assert_array_equal(loaded.viewing_directions, vps.viewing_directions)
    np.testing.assert_array_equal(loaded.viewpoint_ids, vps.viewpoint_ids)
    assert loaded.meta["seed"] == 0

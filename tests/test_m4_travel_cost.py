"""Unit tests for M4 travel cost (paper05_replication.md §8).

Hand-calculated toy cases with explicit expected values, plus the
matrix/home/serialization properties and the bracket integration test.
"""

import numpy as np
import pytest

from src.m2_sample import CandidateViewpointSet
from src.m4_travel_cost import (
    TravelCosts,
    angle_from_dots,
    compute_travel_costs,
    home_pose,
    travel_cost_matrix,
    travel_cost_vector,
)

ALPHA, BETA = 1.2, 0.4


def make_viewpoints(positions, directions) -> CandidateViewpointSet:
    positions = np.asarray(positions, dtype=np.float64)
    directions = np.asarray(directions, dtype=np.float64)
    directions = directions / np.linalg.norm(directions, axis=1, keepdims=True)
    n = len(positions)
    return CandidateViewpointSet(
        np.arange(n, dtype=np.int32), positions, directions, {"n": n}
    )


Z = np.array([0.0, 0.0, 1.0])  # +z
X = np.array([1.0, 0.0, 0.0])


# ---------------------------------------------------------------------------
# hand-calculated cost cases (task step 6, items 1-5)
# ---------------------------------------------------------------------------


def test_same_pose_zero_cost():
    C = travel_cost_matrix([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], [Z, Z],
                           alpha=ALPHA, beta=BETA)
    assert C[0, 0] == 0.0
    assert C[1, 1] == 0.0
    # identical poses: position and direction both equal -> 0
    C2 = travel_cost_matrix([[0.5, 0.5, 0.5], [0.5, 0.5, 0.5]], [Z, Z],
                            alpha=ALPHA, beta=BETA)
    assert C2[0, 1] == 0.0


def test_position_only_cost_alpha_d():
    # d = ||(0,0,0) - (1,0,0)|| = 1 m, same direction -> theta = 0
    C = travel_cost_matrix([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], [Z, Z],
                           alpha=ALPHA, beta=BETA)
    assert C[0, 1] == pytest.approx(ALPHA * 1.0, abs=1e-12)  # 1.2 s


def test_direction_only_cost_beta_theta():
    # same position, orthogonal directions -> theta = pi/2
    C = travel_cost_matrix([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], [Z, X],
                           alpha=ALPHA, beta=BETA)
    assert C[0, 1] == pytest.approx(BETA * np.pi / 2, abs=1e-12)  # 0.6283 s


def test_combined_cost_sum_of_terms():
    # d = 2 m, orthogonal directions -> 1.2 * 2 + 0.4 * pi/2 = 3.0283... s
    C = travel_cost_matrix([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]], [Z, X],
                           alpha=ALPHA, beta=BETA)
    assert C[0, 1] == pytest.approx(ALPHA * 2 + BETA * np.pi / 2, abs=1e-12)


def test_angle_hand_cases():
    # same -> 0; orthogonal -> pi/2; opposite -> pi; 60 deg -> pi/3
    v60 = np.array([np.sin(np.radians(60)), 0.0, np.cos(np.radians(60))])
    dots = np.array([1.0, 0.0, -1.0, 0.5])
    angles = angle_from_dots(dots)
    np.testing.assert_allclose(angles, [0.0, np.pi / 2, np.pi, np.pi / 3], atol=1e-12)
    # the 0.5 dot is exactly the 60-degree case: dot(Z, v60) = cos(60) = 0.5
    assert np.dot(Z, v60) == pytest.approx(0.5, abs=1e-12)
    C = travel_cost_matrix([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], [Z, v60],
                           alpha=ALPHA, beta=BETA)
    assert C[0, 1] == pytest.approx(BETA * np.pi / 3, abs=1e-12)


def test_angle_numerical_stability_fp_overflow():
    # slightly out-of-range dots must clip, not NaN
    angles = angle_from_dots(np.array([1.0 + 1e-12, -1.0 - 1e-12, 1e-15]))
    assert np.isfinite(angles).all()
    np.testing.assert_allclose(angles, [0.0, np.pi, np.pi / 2 - 1e-15], atol=1e-12)
    # end-to-end: nearly parallel directions -> tiny but finite angle
    # (1e-4 rad tilt is above machine precision; a 1e-9 tilt would round
    # the dot product to exactly 1.0 and yield angle 0 -- also finite)
    w = np.array([1e-4, 0.0, 1.0])
    w /= np.linalg.norm(w)
    C = travel_cost_matrix([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], [Z, w],
                           alpha=ALPHA, beta=BETA)
    assert np.isfinite(C).all() and 0.0 < C[0, 1] < 1e-3


# ---------------------------------------------------------------------------
# invalid inputs (task step 6, item 7)
# ---------------------------------------------------------------------------


def test_invalid_shapes_raise():
    with pytest.raises(ValueError):
        travel_cost_matrix(np.zeros((2, 2)), np.zeros((2, 3)))
    with pytest.raises(ValueError):
        travel_cost_matrix(np.zeros((2, 3)), np.zeros((3, 3)))
    with pytest.raises(ValueError):
        travel_cost_matrix(np.zeros((2, 3)), np.zeros((2, 2)))


def test_nan_inf_raise():
    with pytest.raises(ValueError, match="NaN/Inf"):
        travel_cost_matrix([[0.0, 0.0, np.nan]], [Z])
    with pytest.raises(ValueError, match="NaN/Inf"):
        travel_cost_matrix([[0.0, 0.0, 0.0]], [[0.0, np.inf, 0.0]])


def test_zero_direction_raises():
    with pytest.raises(ValueError, match="unit"):
        travel_cost_matrix([[0.0, 0.0, 0.0]], [[0.0, 0.0, 0.0]])
    with pytest.raises(ValueError, match="unit"):
        travel_cost_matrix([[0.0, 0.0, 0.0]], [[0.0, 0.0, 0.5]])  # non-unit


def test_travel_cost_vector_validates_too():
    with pytest.raises(ValueError):
        travel_cost_vector(np.zeros(3), np.zeros(3), [[0.0, 0.0, 0.0]], [Z])
    with pytest.raises(ValueError):
        travel_cost_vector(np.zeros(2), Z, [[0.0, 0.0, 0.0]], [Z])  # bad shape


# ---------------------------------------------------------------------------
# matrix properties (task step 6, item 8)
# ---------------------------------------------------------------------------


def test_matrix_symmetry_diagonal_nonneg():
    rng = np.random.default_rng(0)
    p = rng.normal(size=(8, 3))
    d = rng.normal(size=(8, 3))
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    C = travel_cost_matrix(p, d, alpha=ALPHA, beta=BETA)
    np.testing.assert_array_equal(C, C.T)  # exactly symmetric
    assert np.diag(C).max() == 0.0
    assert (C >= 0).all()
    assert (C[~np.eye(8, dtype=bool)] > 0).all()  # distinct poses -> positive


# ---------------------------------------------------------------------------
# home pose (task step 6, item 9)
# ---------------------------------------------------------------------------


def test_home_pose_definition():
    center = np.array([0.0, 0.0, 0.0])
    p, v = home_pose(center, (0.9, 0.0, 0.6))
    np.testing.assert_allclose(p, [0.9, 0.0, 0.6], atol=1e-15)
    # direction points at the object center: normalize(center - p)
    np.testing.assert_allclose(v, np.array([-0.9, 0.0, -0.6]) / np.linalg.norm([-0.9, 0.0, -0.6]),
                               atol=1e-15)


def test_home_costs_hand_calc():
    # home: p_h = (0.9, 0, 0.6), v_h = normalize(-0.9, 0, -0.6)
    # v0: p = (0, 0, 0.6) looking straight down (0, 0, -1)
    #   d = 0.9 m
    #   dot = (-0.9, 0, -0.6)/||.|| . (0, 0, -1) = 0.6 / sqrt(1.17)
    #   theta = arccos(0.6 / sqrt(1.17)) ~= 0.9828 rad
    #   cost = 1.2 * 0.9 + 0.4 * theta ~= 1.4731 s
    vps = make_viewpoints([[0.0, 0.0, 0.6]], [[0.0, 0.0, -1.0]])
    costs = compute_travel_costs(vps, object_center=np.zeros(3),
                                 alpha=ALPHA, beta=BETA,
                                 home_offset=(0.9, 0.0, 0.6))
    expected = ALPHA * 0.9 + BETA * np.arccos(0.6 / np.sqrt(1.17))
    assert costs.home_to_viewpoint[0] == pytest.approx(expected, abs=1e-12)
    assert costs.viewpoint_to_home[0] == pytest.approx(expected, abs=1e-12)
    np.testing.assert_allclose(costs.home_position, [0.9, 0.0, 0.6], atol=1e-15)


def test_home_legs_kept_separate_with_same_values():
    vps = make_viewpoints([[0.0, 0.0, 0.6], [0.3, 0.0, 0.2]], [Z, X])
    costs = compute_travel_costs(vps, object_center=np.zeros(3))
    np.testing.assert_allclose(costs.home_to_viewpoint, costs.viewpoint_to_home,
                               atol=1e-12)
    assert costs.home_to_viewpoint is not costs.viewpoint_to_home


# ---------------------------------------------------------------------------
# serialization + index consistency (task step 6, items 10-11)
# ---------------------------------------------------------------------------


def test_serialization_roundtrip(tmp_path):
    vps = make_viewpoints([[0.0, 0.0, 0.6], [0.3, 0.0, 0.2], [0.9, 0.1, 0.0]],
                          [Z, X, Z])
    costs = compute_travel_costs(vps, object_center=np.zeros(3))
    npz_path, json_path = costs.save(tmp_path / "E_test")
    assert npz_path.exists() and json_path.exists()
    loaded = TravelCosts.load(tmp_path / "E_test")
    np.testing.assert_array_equal(loaded.matrix, costs.matrix)
    np.testing.assert_array_equal(loaded.home_to_viewpoint, costs.home_to_viewpoint)
    np.testing.assert_array_equal(loaded.viewpoint_to_home, costs.viewpoint_to_home)
    np.testing.assert_array_equal(loaded.home_position, costs.home_position)
    np.testing.assert_array_equal(loaded.home_direction, costs.home_direction)
    np.testing.assert_array_equal(loaded.viewpoint_ids, costs.viewpoint_ids)
    assert loaded.meta["alpha"] == ALPHA and loaded.meta["beta"] == BETA


def test_index_order_consistency():
    # reversed input order must produce a permuted matrix, and the stored
    # ids must follow the input order
    p = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    d = np.array([Z, X, Z], dtype=float)
    fwd = CandidateViewpointSet(np.array([0, 1, 2]), p, d, {"n": 3})
    rev = CandidateViewpointSet(np.array([2, 1, 0]), p[::-1], d[::-1], {"n": 3})
    cf = compute_travel_costs(fwd, object_center=np.zeros(3))
    cr = compute_travel_costs(rev, object_center=np.zeros(3))
    # C_rev[i, j] == C_fwd[2-i, 2-j]
    np.testing.assert_array_equal(cr.matrix, cf.matrix[::-1, ::-1])
    np.testing.assert_array_equal(cr.viewpoint_ids, [2, 1, 0])
    np.testing.assert_array_equal(cr.home_to_viewpoint, cf.home_to_viewpoint[::-1])


# ---------------------------------------------------------------------------
# bracket integration (task step 6, item 12)
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def bracket_costs():
    from pathlib import Path

    from src.m1_discretize import DiscretizedSurface
    from src.m2_sample import CandidateViewpointSet

    root = Path(__file__).resolve().parents[1]
    vps = CandidateViewpointSet.load(root / "data" / "outputs" / "V_bracket_200.npz")
    surface = DiscretizedSurface.load(root / "data" / "outputs" / "S_bracket_1000.npz")
    return compute_travel_costs(vps, source_viewpoints="V_bracket_200.npz"), surface


def test_bracket_integration_shape_and_properties(bracket_costs):
    costs, _ = bracket_costs
    assert costs.matrix.shape == (200, 200)
    s = costs.stats()
    assert s["diag_max_err"] == 0.0
    assert s["symmetry_max_err"] < 1e-12
    # guide §8 sanity band: off-diagonal values in the 0.2-3 s range
    assert s["offdiag"]["max"] < 4.0
    assert 0.1 < s["offdiag"]["median"] < 2.5
    assert s["home_leg_max_diff"] < 1e-12


def test_bracket_integration_consistency_with_upstream(bracket_costs):
    # M4 does not drop or reorder candidates, and M3's A has one row per
    # viewpoint -- pure correspondence check (visibility is NOT part of the
    # cost formula)
    import numpy as np
    from pathlib import Path

    costs, _ = bracket_costs
    root = Path(__file__).resolve().parents[1]
    A = np.load(root / "data" / "outputs" / "A_bracket_200.npz")["A"]
    assert A.shape[0] == costs.matrix.shape[0] == 200
    assert len(costs.home_to_viewpoint) == 200

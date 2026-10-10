"""Unit tests for M3 visibility (paper05_replication.md §7).

Toy geometries (task spec §10) with hand-calculated outcomes, plus unit
tests for each criterion and the bracket integration test.
"""

import numpy as np
import pytest
import trimesh

from src.m1_discretize import DiscretizedSurface, discretize, patches_from_mesh
from src.m2_sample import CandidateViewpointSet, viewing_frames
from src.m3_visibility import (
    VisibilityMatrix,
    build_ray_mesh,
    compute_visibility,
    coverage_ratio,
    fod_check,
    fov_check,
    occlusion_check,
    viewing_angle_check,
)

BOX = trimesh.creation.box(extents=[2.0, 1.0, 1.0])


# ---------------------------------------------------------------------------
# toy scene builders
# ---------------------------------------------------------------------------


def make_surface(mesh: trimesh.Trimesh) -> DiscretizedSurface:
    ids, tri, centers, normals, areas = patches_from_mesh(mesh)
    return DiscretizedSurface(ids, tri, centers, normals, areas, mesh.bounds,
                              {"source_mesh": {"area": float(mesh.area)}})


def make_viewpoints(positions, directions) -> CandidateViewpointSet:
    positions = np.asarray(positions, dtype=np.float64)
    directions = np.asarray(directions, dtype=np.float64)
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    n = len(positions)
    return CandidateViewpointSet(
        np.arange(n, dtype=np.int32), positions, directions, {"n": n}
    )


def floor_patch(z: float = 0.0) -> trimesh.Trimesh:
    """One triangle in the z=z0 plane, outward normal +z."""
    return trimesh.Trimesh(
        vertices=np.array([[0.0, 0.0, z], [0.1, 0.0, z], [0.0, 0.1, z]]),
        faces=np.array([[0, 1, 2]]),
        process=False,
    )


def front_back_planes() -> trimesh.Trimesh:
    """Two parallel patches, both normal +z: front at z=0, back at z=-0.2."""
    return trimesh.Trimesh(
        vertices=np.array([
            [0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.0, 0.1, 0.0],  # front
            [0.0, 0.0, -0.2], [0.1, 0.0, -0.2], [0.0, 0.1, -0.2],  # back
        ]),
        faces=np.array([[0, 1, 2], [3, 4, 5]]),
        process=False,
    )


def vis(surface: DiscretizedSurface, vps: CandidateViewpointSet, **kw) -> np.ndarray:
    return compute_visibility(surface, vps, **kw).A


# ---------------------------------------------------------------------------
# toy tests (task spec §10)
# ---------------------------------------------------------------------------


def test_toy_1_head_on_single_patch_visible():
    surface = make_surface(floor_patch(0.0))
    vps = make_viewpoints([[0.0, 0.0, 0.6]], [[0.0, 0.0, -1.0]])
    A = vis(surface, vps)
    assert A.shape == (1, 1) and A.dtype == bool
    assert A[0, 0]  # FOD ok, FOV ok, angle 0 deg, no occlusion


def test_toy_2_fov_outside_and_behind():
    surface = make_surface(floor_patch(0.0))
    # viewing direction tilted 16 deg off the head-on axis (> 15 deg half FOV)
    tilted = make_viewpoints([[0.0, 0.0, 0.6]], [[0.3, 0.0, -1.0]])
    assert not vis(surface, tilted)[0, 0]
    # patch entirely behind the camera (guide §7.5 test 3)
    sideways = make_viewpoints([[0.0, 0.0, 0.6]], [[1.0, 0.0, 0.0]])
    assert not vis(surface, sideways)[0, 0]


def test_toy_3_fod_outside():
    surface = make_surface(floor_patch(0.0))
    too_close = make_viewpoints([[0.0, 0.0, 0.2]], [[0.0, 0.0, -1.0]])
    assert not vis(surface, too_close)[0, 0]  # 0.2 m < FOD min 0.40
    too_far = make_viewpoints([[0.0, 0.0, 1.1]], [[0.0, 0.0, -1.0]])
    assert not vis(surface, too_far)[0, 0]  # ~1.1 m > FOD max 1.00


def test_toy_4_viewing_angle_over_75():
    # guide §7.5 test 4: camera at (0.6, 0, 0.1) looking at the patch:
    # theta = angle between (viewpoint -> center) and +z normal ~= 80.5 deg
    surface = make_surface(floor_patch(0.0))
    c = np.array([0.0, 0.0333, 0.0])
    v = make_viewpoints([[0.6, 0.0, 0.1]], [c - [0.6, 0.0, 0.1]])
    assert not vis(surface, v)[0, 0]


def test_toy_5_occlusion_blocker():
    surface = make_surface(front_back_planes())
    vps = make_viewpoints([[0.0, 0.0, 0.6]], [[0.0, 0.0, -1.0]])
    A = vis(surface, vps)
    assert A[0, 0]  # front patch visible
    assert not A[0, 1]  # back patch occluded by the front patch


def test_toy_6_no_occlusion_single_target():
    # the ray to the target vertex hits the target's own triangles at
    # exactly the target distance: not an occlusion (self-hit ignored)
    surface = make_surface(floor_patch(0.0))
    vps = make_viewpoints([[0.0, 0.0, 0.6]], [[0.0, 0.0, -1.0]])
    assert vis(surface, vps)[0, 0]


def test_toy_7_target_self_hit_not_occlusion():
    ray_mesh = build_ray_mesh(make_surface(floor_patch(0.0)))
    origin = np.array([[0.0, 0.0, 0.6]])
    direction = np.array([[0.0, 0.0, -0.6]])  # straight to the origin vertex
    occluded = occlusion_check(origin, direction, [0.6], ray_mesh, eps=1e-3)
    assert not occluded[0]


def test_toy_8_fov_boundary_inclusive():
    # Under the M2 roll convention x_c is world-horizontal, so a vertex
    # straight below the camera always lands on the frame's vertical axis:
    # tilting v_hat around the world x-axis hits the 10.5 deg vertical
    # half-FOV exactly. (The exact 15 deg horizontal boundary is covered
    # frame-level in test_fov_check_hand_values.)
    surface = make_surface(floor_patch(0.0))
    theta = np.radians(10.5)  # exact vertical half-FOV boundary
    at_boundary = make_viewpoints([[0.0, 0.0, 0.6]],
                                  [[0.0, np.sin(theta), -np.cos(theta)]])
    assert vis(surface, at_boundary)[0, 0]  # inclusive boundary
    theta = np.radians(10.7)  # just outside
    outside = make_viewpoints([[0.0, 0.0, 0.6]],
                              [[0.0, np.sin(theta), -np.cos(theta)]])
    assert not vis(surface, outside)[0, 0]


# ---------------------------------------------------------------------------
# criterion unit tests
# ---------------------------------------------------------------------------


def test_fov_check_hand_values():
    frame = np.eye(3)  # x right, y up, z forward
    tan15 = np.tan(np.radians(15.0))
    tan10_5 = np.tan(np.radians(10.5))
    pts = np.array([
        [0.0, 0.0, 1.0],          # dead center: inside
        [0.99 * tan15, 0.0, 1.0],  # horizontal inside
        [tan15, 0.0, 1.0],         # horizontal boundary: inclusive
        [1.01 * tan15, 0.0, 1.0],  # horizontal outside
        [0.0, 0.99 * tan10_5, 1.0],  # vertical inside
        [0.0, 1.01 * tan10_5, 1.0],  # vertical outside
        [0.0, 0.0, -1.0],         # behind the camera
    ])
    ok = fov_check(pts, frame)
    np.testing.assert_array_equal(ok, [True, True, True, False, True, False, False])


def test_fod_check_boundaries():
    pts = np.array([[0.4, 0.0, 0.0], [1.0, 0.0, 0.0], [0.399, 0.0, 0.0], [1.001, 0.0, 0.0]])
    np.testing.assert_array_equal(fod_check(pts), [True, True, False, False])


def test_viewing_angle_check_hand_values():
    # the 75 deg boundary is inclusive (cos >= cos(75)); an exact-75
    # construction is fp-fragile, so test just inside/outside plus the
    # two trivial cases
    centers = np.array([[0.0, 0.0, 0.0]])
    normals = np.array([[0.0, 0.0, 1.0]])
    pos = np.array([
        [0.0, 0.0, 0.6],                 # 0 deg: visible
        [np.sin(np.radians(74.9)) * 0.6, 0.0, np.cos(np.radians(74.9)) * 0.6],  # inside
        [np.sin(np.radians(75.1)) * 0.6, 0.0, np.cos(np.radians(75.1)) * 0.6],  # outside
        [0.0, 0.0, -0.6],                # 180 deg (back-facing): invisible
    ])
    ok = viewing_angle_check(pos, centers, normals, max_deg=75.0)
    np.testing.assert_array_equal(ok[:, 0], [True, True, False, False])


def test_occlusion_check_blocker_vs_target():
    ray_mesh = build_ray_mesh(make_surface(front_back_planes()))
    # ray to the back-patch vertex: hits the front patch first
    origin = np.array([[0.0, 0.0, 0.6]])
    direction = np.array([[0.0, 0.0, -0.8]])
    occluded = occlusion_check(origin, direction, [0.8], ray_mesh, eps=1e-3)
    assert occluded[0]
    # ray to the front-patch vertex: first hit is the target itself
    direction2 = np.array([[0.0, 0.0, -0.6]])
    occluded2 = occlusion_check(origin, direction2, [0.6], ray_mesh, eps=1e-3)
    assert not occluded2[0]


def test_occlusion_check_epsilon_semantics():
    # camera at (0,0,0.6), target vertex at (0,0,0) -> target distance 0.6.
    # A blocker within eps of the target counts as a self-hit; beyond eps
    # it occludes.
    target = floor_patch(0.0)
    origin = np.array([[0.0, 0.0, 0.6]])
    direction = np.array([[0.0, 0.0, -0.6]])

    def with_blocker(z_blocker: float) -> trimesh.Trimesh:
        blocker = trimesh.Trimesh(
            vertices=np.array([[0.0, 0.0, z_blocker], [0.1, 0.0, z_blocker],
                               [0.0, 0.1, z_blocker]]),
            faces=np.array([[0, 1, 2]]), process=False)
        return trimesh.util.concatenate([target, blocker])

    # blocker at z = 0.0005: hit at 0.5995 = target - 0.5*eps -> self-hit
    ok = occlusion_check(origin, direction, [0.6], with_blocker(0.0005), eps=1e-3)
    assert not ok[0]
    # blocker at z = 0.002: hit at 0.598 = target - 2*eps -> occluded
    ok = occlusion_check(origin, direction, [0.6], with_blocker(0.002), eps=1e-3)
    assert ok[0]


def test_vertex_aggregation_all_vertices_required():
    # a large patch: one vertex dead-center (visible), the others far
    # outside the FOV -> the whole patch must be invisible
    mesh = trimesh.Trimesh(
        vertices=np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
        faces=np.array([[0, 1, 2]]), process=False)
    surface = make_surface(mesh)
    vps = make_viewpoints([[0.0, 0.0, 0.6]], [[0.0, 0.0, -1.0]])
    A = vis(surface, vps)
    assert A.shape == (1, 1) and not A[0, 0]


def test_camera_frame_right_handed():
    dirs = np.array([[0.0, 0.0, -1.0], [0.3, 0.2, -1.0], [0.0, 1.0, 0.0]])
    frames = viewing_frames(dirs)
    for i in range(3):
        R = frames[i]
        assert abs(np.linalg.det(R) - 1.0) < 1e-12  # right-handed
        np.testing.assert_allclose(R[2], dirs[i] / np.linalg.norm(dirs[i]), atol=1e-12)


# ---------------------------------------------------------------------------
# matrix / coverage helpers
# ---------------------------------------------------------------------------


def test_visibility_matrix_shape_dtype_stats():
    A = np.array([[True, False], [True, True], [False, False]])
    vm = VisibilityMatrix(A, {"n": 3})
    assert vm.shape == (3, 2) and vm.A.dtype == bool
    s = vm.stats()
    assert s["row_sum"] == {"min": 0, "median": 1.0, "max": 2, "mean": 1.0,
                            "zero_rows": 1}
    assert s["col_sum"]["zero_cols"] == 0
    np.testing.assert_array_equal(vm.row_sum(), [1, 2, 0])
    np.testing.assert_array_equal(vm.col_sum(), [2, 1])


def test_coverage_ratio():
    A = np.array([[True, True, False, False],
                  [False, True, True, False]])
    cov, covered = coverage_ratio(A, np.array([0]))
    assert cov == 0.5
    np.testing.assert_array_equal(covered, [True, True, False, False])
    cov2, covered2 = coverage_ratio(A, np.array([0, 1]))
    assert cov2 == 0.75
    cov3, _ = coverage_ratio(A, np.array([], dtype=int))
    assert cov3 == 0.0


def test_serialization_roundtrip(tmp_path):
    A = np.eye(5, dtype=bool)
    vm = VisibilityMatrix(A, {"seed": 0})
    npz_path, json_path = vm.save(tmp_path / "A_test")
    assert npz_path.exists() and json_path.exists()
    loaded = VisibilityMatrix.load(tmp_path / "A_test")
    np.testing.assert_array_equal(loaded.A, A)
    assert loaded.meta["stats"]["shape"] == [5, 5]


# ---------------------------------------------------------------------------
# bracket integration (M1 + M2 -> A)
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def bracket_visibility():
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    surface = DiscretizedSurface.load(root / "data" / "outputs" / "S_bracket_1000.npz")
    viewpoints = CandidateViewpointSet.load(root / "data" / "outputs" / "V_bracket_400.npz")
    A = compute_visibility(surface, viewpoints,
                           source_surface="S_bracket_1000.npz",
                           source_viewpoints="V_bracket_400.npz")
    return A, surface


def test_bracket_integration_shape_density(bracket_visibility):
    A, _ = bracket_visibility
    assert A.shape == (400, 976)
    assert A.A.dtype == bool
    assert 0.0 < A.A.mean() < 0.5  # sane density, not all-zero / all-one


def test_bracket_integration_zero_columns_are_the_bottom_face(bracket_visibility):
    # The zero-visibility patches are the workpiece's bottom face: M2's
    # table-plane rejection keeps every viewpoint at z >= -0.01, above the
    # bottom face (z = -0.06), so the viewing angle towards the downward
    # normal always exceeds 75 deg. This is a structural property of the
    # M2 sampling design (object on a table, as in the paper), NOT an M3
    # bug -- diagnosed, not "fixed" by touching M2.
    A, surface = bracket_visibility
    cs = A.col_sum()
    bottom = surface.normals[:, 2] < -0.5
    assert bottom.sum() > 100  # the bracket does have a substantial bottom
    assert (cs[bottom] == 0).all()  # 100% of the bottom face invisible
    non_bottom_invisible = int(((cs == 0) & ~bottom).sum())
    assert non_bottom_invisible <= 2  # everything else is covered
    # union coverage = visible surface fraction
    assert (cs > 0).mean() == pytest.approx(0.83, abs=0.02)


def test_bracket_integration_zero_rows_expected(bracket_visibility):
    # some viewpoints are beyond FOD max (M2 shell artifact): their rows
    # may be all-zero; that is expected, not a bug
    A, _ = bracket_visibility
    s = A.stats()
    assert s["row_sum"]["zero_rows"] <= 0.35 * A.shape[0]


def test_bracket_integration_n200_coverage(bracket_visibility):
    # with only 200 candidates, glancing-angle patches lose coverage too;
    # the union coverage floor is a sanity check only
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    surface = DiscretizedSurface.load(root / "data" / "outputs" / "S_bracket_1000.npz")
    viewpoints = CandidateViewpointSet.load(root / "data" / "outputs" / "V_bracket_200.npz")
    A = compute_visibility(surface, viewpoints)
    cs = A.col_sum()
    assert (cs > 0).mean() >= 0.65

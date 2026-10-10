"""Unit tests for M1 surface discretization (paper05_replication.md §5).

Self-check criteria from the guide:
  * patch count within m +/- 5%
  * normals point outward (watertight meshes)
  * edge-length std/mean < 0.5 (uniformity diagnostic)
"""

import numpy as np
import pytest
import trimesh

from src.m1_discretize import (
    DiscretizedSurface,
    check_outward_normals,
    discretize,
    load_mesh,
    patches_from_mesh,
    prepare_mesh,
    resample_uniform,
)

BOX = trimesh.creation.box(extents=[2.0, 1.0, 1.0])  # area 10, watertight
SPHERE = trimesh.creation.icosphere(subdivisions=4)  # 5120 faces, watertight


def make_bracket() -> trimesh.Trimesh:
    """Watertight L-profile extrusion (hand-calculable: area = 0.1024)."""
    pts = np.array(
        [[0, 0], [0.2, 0], [0.2, 0.04], [0.06, 0.04], [0.06, 0.12], [0, 0.12]],
        dtype=float,
    )
    tris = np.array([[0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 5]])
    return trimesh.creation.extrude_triangulation(pts, tris, height=0.12)


# ---------------------------------------------------------------------------
# patch record geometry (hand-calculated)
# ---------------------------------------------------------------------------


def test_single_triangle_patch_hand_calc():
    mesh = trimesh.Trimesh(
        vertices=np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
        faces=np.array([[0, 1, 2]]),
        process=False,
    )
    ids, verts, centers, normals, areas = patches_from_mesh(mesh)
    assert len(ids) == 1 and ids[0] == 0
    np.testing.assert_allclose(verts[0], mesh.vertices[mesh.faces[0]])
    np.testing.assert_allclose(centers[0], [1 / 3, 1 / 3, 0.0], atol=1e-15)
    np.testing.assert_allclose(normals[0], [0.0, 0.0, 1.0], atol=1e-15)
    assert areas[0] == pytest.approx(0.5, abs=1e-15)


def test_patch_record_structure_and_ids():
    surface = discretize(BOX, m=300)
    assert len(surface) == len(surface.patch_ids)
    np.testing.assert_array_equal(surface.patch_ids, np.arange(len(surface)))
    for patch in list(surface.patches())[:5]:
        assert patch.center.shape == (3,)
        assert patch.vertices.shape == (3, 3)
        assert patch.normal.shape == (3,)
        assert isinstance(patch.area, float) and patch.area > 0
        # center = vertex mean, area = 0.5 * |cross(v1-v0, v2-v0)|
        np.testing.assert_allclose(patch.center, patch.vertices.mean(axis=0))
        cross = np.cross(patch.vertices[1] - patch.vertices[0], patch.vertices[2] - patch.vertices[0])
        assert patch.area == pytest.approx(0.5 * np.linalg.norm(cross))
        # unit normal
        assert np.linalg.norm(patch.normal) == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# resampling: count within m +/- 5%
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("m", [500, 1000])
def test_discretize_box_count_within_5pct(m):
    surface = discretize(BOX, m=m)
    assert abs(len(surface) - m) <= 0.05 * m
    assert surface.meta["checks"]["patch_count_within_5pct"]


def test_discretize_decimation_path_count():
    # 5120 faces -> 500: exercises quadric decimation + subdivision
    surface = discretize(SPHERE, m=500)
    assert abs(len(surface) - 500) <= 0.05 * 500
    assert surface.meta["discretization"]["decimated_from"] == len(SPHERE.faces)
    # decimation + subdivision keep the mesh watertight with outward normals
    ok, total = surface.meta["checks"]["outward_normals"]
    assert ok == total
    assert surface.meta["checks"]["normals_unit"]


def test_discretize_bracket_count():
    surface = discretize(make_bracket(), m=800)
    assert abs(len(surface) - 800) <= 0.05 * 800


def test_small_m_decimation_fallback():
    # face-count granularity (12 -> 28 -> 32 -> 64 -> ...) skips the
    # m +/- 5% window for m=50; the fallback decimates the plateau just
    # above m (64) down to exactly m
    surface = discretize(BOX, m=50)
    assert abs(len(surface) - 50) <= 0.05 * 50
    assert surface.meta["discretization"]["final_decimated_from"] == 64


def test_small_m_granularity_warns_when_fallback_cannot_help():
    # m=15: the fallback cannot close the gap (decimation of such a tiny
    # mesh fails the area/count guard), so the closest achievable count
    # (12) is returned with a warning
    with pytest.warns(UserWarning):
        surface = discretize(BOX, m=15)
    assert 0.5 * 15 <= len(surface) <= 1.5 * 15


# ---------------------------------------------------------------------------
# outward normals (guide §5 self-check)
# ---------------------------------------------------------------------------


def test_box_outward_normals_ray_parity():
    # guide §5 self-check: center + eps*n must lie outside a closed mesh
    resampled, _ = resample_uniform(BOX.copy(), 1000)
    _, _, centers, normals, _ = patches_from_mesh(resampled)
    ok = check_outward_normals(resampled, centers, normals)
    assert ok is not None and ok.all()


def test_box_normals_per_face_analytic():
    # box extents [2,1,1]: faces at x=+-1, y=+-0.5, z=+-0.5
    surface = discretize(BOX, m=1000)
    for axis, sign in [(0, 1.0), (0, -1.0), (1, 1.0), (1, -1.0), (2, 1.0), (2, -1.0)]:
        coord = BOX.bounds[1, axis] if sign > 0 else BOX.bounds[0, axis]
        face = np.isclose(surface.centers[:, axis], coord, atol=1e-9)
        assert face.sum() > 0
        expected = np.zeros(3)
        expected[axis] = sign
        # numpy 2.x assert_allclose no longer broadcasts implicitly
        np.testing.assert_allclose(
            surface.normals[face], np.broadcast_to(expected, surface.normals[face].shape), atol=1e-12
        )


def test_inverted_mesh_fails_outward_check():
    inverted = SPHERE.copy()
    inverted.invert()
    ids, verts, centers, normals, areas = patches_from_mesh(inverted)
    ok = check_outward_normals(inverted, centers, normals)
    assert ok is not None and not ok.any()  # every patch flagged


def test_non_watertight_outward_check_returns_none():
    open_mesh = trimesh.Trimesh(vertices=np.array([[0.0, 0, 0], [1, 0, 0], [0, 1, 0]]), faces=[[0, 1, 2]])
    ok = check_outward_normals(open_mesh, np.array([[1 / 3, 1 / 3, 0.0]]), np.array([[0.0, 0, 1]]))
    assert ok is None


# ---------------------------------------------------------------------------
# area / uniformity
# ---------------------------------------------------------------------------


def test_box_subdivision_preserves_area():
    surface = discretize(BOX, m=500)
    # midpoint subdivision of planar faces keeps the surface area exactly
    assert surface.areas.sum() == pytest.approx(BOX.area, rel=1e-9)
    assert surface.meta["mesh_area"] == pytest.approx(BOX.area)


def test_bracket_hand_calc_area():
    bracket = make_bracket()
    assert bracket.is_watertight
    assert bracket.area == pytest.approx(0.1024, rel=1e-12)
    surface = discretize(bracket, m=800)
    assert surface.areas.sum() == pytest.approx(0.1024, rel=1e-9)


@pytest.mark.parametrize(
    "mesh,m",
    [(BOX, 1000), (SPHERE, 500), (make_bracket(), 800)],
    ids=["box", "sphere-decimated", "bracket"],
)
def test_edge_length_uniformity(mesh, m):
    surface = discretize(mesh, m=m)
    cv = surface.stats()["edge_std_over_mean"]
    assert cv < 0.5


# ---------------------------------------------------------------------------
# mesh preparation / loading
# ---------------------------------------------------------------------------


def test_prepare_mesh_centers_bbox_at_origin():
    moved = BOX.copy()
    moved.apply_translation([3.0, 4.0, 5.0])
    prepared = prepare_mesh(moved, center_origin=True)
    np.testing.assert_allclose(prepared.bounds.mean(axis=0), 0.0, atol=1e-12)


def test_prepare_mesh_scales_to_target_diagonal():
    prepared = prepare_mesh(BOX, center_origin=False, target_diagonal=1.0)
    diag = np.linalg.norm(prepared.bounds[1] - prepared.bounds[0])
    assert diag == pytest.approx(1.0, rel=1e-9)


def test_prepare_mesh_removes_degenerate_faces():
    verts = np.vstack([BOX.vertices, [[0.5, 0.0, 0.0]]])
    faces = np.vstack([BOX.faces, [[len(BOX.vertices), len(BOX.vertices), len(BOX.vertices)]]])
    mesh = trimesh.Trimesh(vertices=verts, faces=faces)
    prepared = prepare_mesh(mesh, center_origin=False)
    assert len(prepared.faces) == len(BOX.faces)
    assert len(prepared.vertices) == len(BOX.vertices)


@pytest.mark.parametrize("ext", ["obj", "stl"])
def test_load_mesh_roundtrip(tmp_path, ext):
    path = tmp_path / f"box.{ext}"
    BOX.export(path)
    loaded = load_mesh(path)
    assert len(loaded.faces) == len(BOX.faces)
    assert loaded.area == pytest.approx(BOX.area, rel=1e-9)


def test_load_mesh_rejects_non_mesh_file(tmp_path):
    path = tmp_path / "not_a_mesh.txt"
    path.write_text("hello")
    with pytest.raises(Exception):
        load_mesh(path)


# ---------------------------------------------------------------------------
# serialization / report
# ---------------------------------------------------------------------------


def test_serialization_roundtrip(tmp_path):
    surface = discretize(BOX, m=200)
    npz_path, json_path = surface.save(tmp_path / "S_test")
    assert npz_path.exists() and json_path.exists()

    loaded = DiscretizedSurface.load(tmp_path / "S_test")
    assert len(loaded) == len(surface)
    for attr in ["patch_ids", "vertices", "centers", "normals", "areas", "bounds"]:
        np.testing.assert_array_equal(getattr(loaded, attr), getattr(surface, attr))
    assert loaded.meta["target_patches"] == 200
    assert loaded.meta["checks"]["outward_normals"] == [len(surface), len(surface)]


def test_report_contains_required_labels():
    surface = discretize(make_bracket(), m=400)
    report = surface.format_report()
    for label in ["Mesh:", "Vertices:", "Faces:", "Patch count:", "Bounding box:",
                  "Normal statistics:", "Area statistics:"]:
        assert label in report

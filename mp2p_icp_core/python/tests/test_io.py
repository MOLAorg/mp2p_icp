"""Tests of point cloud file readers and writers: PLY, PCD, LAS, TXT, XYZ."""

from __future__ import annotations

import struct

import numpy as np
import pytest

import mrpt.maps

import mp2p_icp
from mp2p_icp import pcd as pcd_module


def _assert_fields_equal(a: dict, b: dict) -> None:
    assert set(a) == set(b)
    for k in a:
        assert a[k].dtype == b[k].dtype, k
        np.testing.assert_array_equal(a[k], b[k], err_msg=k)


def _fields_without_uint32(fields: dict) -> dict:
    # The PLY, LAS and TXT writers handle float, double, uint16 and uint8 fields.
    return {k: v for k, v in fields.items() if v.dtype != np.uint32}


@pytest.mark.parametrize("binary", [True, False])
def test_ply_round_trip(tmp_path, all_fields, binary):
    fields = _fields_without_uint32(all_fields)
    path = tmp_path / "cloud.ply"
    mp2p_icp.save_ply(mp2p_icp.layer_from_numpy(fields), path, binary=binary)
    header = path.read_bytes()[:400]
    assert b"property uchar red" in header
    assert (b"binary_little_endian" in header) == binary
    layer = mp2p_icp.load_ply(path)
    assert isinstance(layer, mrpt.maps.CGenericPointsMap)
    loaded = mp2p_icp.layer_to_numpy(layer)
    if binary:
        _assert_fields_equal(fields, loaded)
    else:
        # ASCII PLY files keep 9 significant digits for floats:
        assert set(loaded) == set(fields)
        for k in fields:
            np.testing.assert_allclose(loaded[k], fields[k], rtol=1e-7)


def test_ply_options(tmp_path, all_fields):
    layer = mp2p_icp.layer_from_numpy(all_fields)
    path = tmp_path / "subset.ply"
    mp2p_icp.save_ply(layer, path, fields=["z", "x", "intensity"])
    loaded = mp2p_icp.ply.read_ply(path)
    assert list(loaded) == ["z", "x", "intensity"]
    with pytest.raises(RuntimeError):
        mp2p_icp.save_ply(layer, path, fields=["x", "nope"])
    mp2p_icp.save_ply(layer, path, fields=["x", "nope"], ignore_missing_fields=True)
    assert np.all(mp2p_icp.ply.read_ply(path)["nope"] == 0)
    with pytest.raises(TypeError):
        mp2p_icp.save_ply(layer, path, fields="x")
    # Transformed points (e.g. into the ENU frame):
    mp2p_icp.save_ply(layer, path, transform=[1, 2, 3, 0, 0, 0])
    np.testing.assert_allclose(
        mp2p_icp.ply.read_ply(path)["y"], all_fields["y"] + 2, rtol=1e-6, atol=1e-6
    )


def test_ply_from_other_tools(tmp_path):
    # ASCII PLY with normals as nx,ny,nz, signed integers and a face element
    path = tmp_path / "other.ply"
    path.write_text(
        "ply\nformat ascii 1.0\ncomment made by hand\nelement vertex 2\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property float nx\nproperty float ny\nproperty float nz\nproperty int id\n"
        "element face 1\nproperty list uchar int vertex_indices\nend_header\n"
        "0 1 2 0 0 1 -5\n3 4 5 1 0 0 7\n3 0 1 1\n"
    )
    f = mp2p_icp.layer_to_numpy(mp2p_icp.load_ply(path))
    np.testing.assert_array_equal(f["normal_z"], [1, 0])
    np.testing.assert_array_equal(f["id"], [-5, 7])
    assert f["id"].dtype == np.float64
    with pytest.raises(FileNotFoundError):
        mp2p_icp.load_ply(tmp_path / "missing.ply")


@pytest.mark.parametrize("binary", [True, False])
def test_pcd_round_trip(tmp_path, all_fields, binary):
    path = tmp_path / "cloud.pcd"
    mp2p_icp.save_pcd(mp2p_icp.layer_from_numpy(all_fields), path, binary=binary)
    header = path.read_bytes()[:500].decode("ascii", errors="replace")
    assert "FIELDS x y z" in header and " rgb" in header
    loaded = mp2p_icp.layer_to_numpy(mp2p_icp.load_pcd(path))
    if binary:
        _assert_fields_equal(all_fields, loaded)
    else:
        assert set(loaded) == set(all_fields)
        for k in all_fields:
            np.testing.assert_allclose(loaded[k], all_fields[k], rtol=1e-7)
            assert loaded[k].dtype == all_fields[k].dtype


def _lzf_compress_simple(data: bytes) -> bytes:
    """A naive LZF compressor (back references to the previous 8 KB), to test
    the decompressor with both literal runs and back references."""
    out = bytearray()
    lit = bytearray()
    i = 0
    n = len(data)

    def flush():
        for j in range(0, len(lit), 32):
            chunk = lit[j : j + 32]
            out.append(len(chunk) - 1)
            out.extend(chunk)
        lit.clear()

    while i < n:
        best_len = 0
        best_off = 0
        for off in range(1, min(i, 8192) + 1):
            length = 0
            while i + length < n and length < 264 and data[i + length - off] == data[i + length]:
                length += 1
            if length > best_len:
                best_len, best_off = length, off
            if best_len >= 32:
                break
        if best_len >= 3:
            flush()
            ln = best_len - 2
            off = best_off - 1
            if ln < 7:
                out.append((ln << 5) | (off >> 8))
            else:
                out.append((7 << 5) | (off >> 8))
                out.append(ln - 7)
            out.append(off & 0xFF)
            i += best_len
        else:
            lit.append(data[i])
            i += 1
    flush()
    return bytes(out)


def test_pcd_binary_compressed(tmp_path):
    n = 40
    x = np.zeros(n, np.float32)  # long runs: overlapping back references
    y = np.linspace(0, 1, n).astype(np.float32)
    z = np.tile(np.array([1, 2], np.float32), n // 2)
    intensity = np.arange(n, dtype=np.uint16)
    raw = x.tobytes() + y.tobytes() + z.tobytes() + intensity.tobytes()
    comp = _lzf_compress_simple(raw)
    assert pcd_module._lzf_decompress(comp, len(raw)) == raw
    header = (
        "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 2\nTYPE F F F U\nCOUNT 1 1 1 1\n"
        f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary_compressed\n"
    )
    path = tmp_path / "compressed.pcd"
    path.write_bytes(header.encode() + struct.pack("<II", len(comp), len(raw)) + comp)
    f = pcd_module.read_pcd(path)
    np.testing.assert_array_equal(f["x"], x)
    np.testing.assert_array_equal(f["y"], y)
    np.testing.assert_array_equal(f["z"], z)
    np.testing.assert_array_equal(f["intensity"], intensity)


def test_pcd_pcl_conventions(tmp_path):
    # PCL-style ascii file with a packed rgb float, padding and a 2-count field
    rgb = np.array([(255 << 16) | (128 << 8) | 7], dtype=np.uint32).view(np.float32)[0]
    path = tmp_path / "pcl.pcd"
    path.write_text(
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z rgb _ uv\nSIZE 4 4 4 4 1 4\nTYPE F F F F U F\n"
        "COUNT 1 1 1 1 1 2\nWIDTH 1\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 1\nDATA ascii\n"
        f"1 2 3 {rgb:.9g} 0 0.5 0.25\n"
    )
    f = pcd_module.read_pcd(path)
    assert (f["color_r"][0], f["color_g"][0], f["color_b"][0]) == (255, 128, 7)
    assert "_" not in f and f["uv_1"][0] == 0.25


def test_las(tmp_path, all_fields):
    layer = mp2p_icp.layer_from_numpy(_fields_without_uint32(all_fields))
    path = tmp_path / "cloud.las"
    mp2p_icp.save_las(layer, path, system_id="pytest")
    data = path.read_bytes()
    assert data[:4] == b"LASF"
    assert data[24:25] == b"\x01" and data[25:26] == b"\x04"  # version 1.4
    assert data[26:32] == b"pytest"
    n_points = struct.unpack("<Q", data[247:255])[0]
    assert n_points == len(all_fields["x"])

    laspy = pytest.importorskip("laspy")
    las = laspy.read(path)
    np.testing.assert_allclose(las.x, all_fields["x"], atol=1e-3)
    np.testing.assert_allclose(las.intensity, np.clip(all_fields["intensity"], 0, 65535), atol=1)
    np.testing.assert_array_equal(las.red, all_fields["color_r"].astype(np.uint16) * 256)
    np.testing.assert_array_equal(las["ring"], all_fields["ring"])
    np.testing.assert_allclose(las.gps_time, all_fields["gps_time"])


def test_txt(tmp_path, all_fields):
    fields = _fields_without_uint32(all_fields)
    layer = mp2p_icp.layer_from_numpy(fields)
    path = tmp_path / "cloud.txt"
    mp2p_icp.save_txt(layer, path)
    names = path.read_text().splitlines()[0].split()
    assert names[:3] == ["x", "y", "z"] and set(names) == set(fields)
    table = np.loadtxt(path, skiprows=1)
    for i, name in enumerate(names):
        np.testing.assert_allclose(table[:, i], fields[name], rtol=1e-6, atol=1e-6)

    mp2p_icp.save_txt(layer, path, header=False, fields=["z", "ring"])
    table = np.loadtxt(path)
    assert table.shape == (len(fields["x"]), 2)
    np.testing.assert_array_equal(table[:, 1], fields["ring"])


def test_xyz_file(demos_dir, tmp_path):
    bunny = mp2p_icp.load_xyz_file(demos_dir / "bunny_decim.xyz.gz")
    assert isinstance(bunny, mrpt.maps.CSimplePointsMap)
    assert bunny.size() > 1000
    with pytest.raises(FileNotFoundError):
        mp2p_icp.load_xyz_file(tmp_path / "missing.xyz")


def test_export_whole_map_layers(demos_dir, tmp_path):
    # Layers of a .mm file through the exporters, as the mm2* CLI tools:
    m = mp2p_icp.MetricMap.from_file(demos_dir / "global_001.mm")
    for name, layer in m.layers.items():
        mp2p_icp.save_ply(layer, tmp_path / f"{name}.ply")
        mp2p_icp.save_pcd(layer, tmp_path / f"{name}.pcd")
        mp2p_icp.save_las(layer, tmp_path / f"{name}.las")
        mp2p_icp.save_txt(layer, tmp_path / f"{name}.txt")
        n = layer.size()
        assert mp2p_icp.load_ply(tmp_path / f"{name}.ply").size() == n
        assert mp2p_icp.load_pcd(tmp_path / f"{name}.pcd").size() == n

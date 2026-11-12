"""Conversions between mp2p_icp / MRPT point clouds and other Python libraries.

All of them are optional dependencies, imported only when used: pandas (and
pyarrow, for Parquet), Open3D, PyVista and laspy.

Point clouds go through :func:`mp2p_icp.layer_to_numpy` and
:func:`mp2p_icp.layer_from_numpy`, so they keep all their per-point fields.
MRPT field naming conventions: ``intensity``, ``ring``, ``t``,
``color_r, color_g, color_b`` (uint8) or ``color_rf, color_gf, color_bf``
(float, in [0,1]), ``normal_x, normal_y, normal_z``.
"""

from __future__ import annotations

import os
from typing import Any, Dict, Optional, Sequence, Union

import numpy as np
import mrpt.maps

from . import _core
from ._util import FieldDict, require, supported_fields

_RGB_U8 = ("color_r", "color_g", "color_b")
_RGB_F = ("color_rf", "color_gf", "color_bf")
_NORMALS = ("normal_x", "normal_y", "normal_z")

LayerLike = Union["mrpt.maps.CMetricMap", FieldDict]


def _fields(layer: LayerLike) -> FieldDict:
    if isinstance(layer, dict):
        return {k: np.asarray(v) for k, v in layer.items()}
    return _core.layer_to_numpy(layer)


def _colors_float(f: FieldDict) -> Optional[np.ndarray]:
    """Returns (N,3) colors in [0,1], if the cloud has them."""
    if all(n in f for n in _RGB_F):
        return np.column_stack([f[n] for n in _RGB_F]).astype(np.float64)
    if all(n in f for n in _RGB_U8):
        return np.column_stack([f[n] for n in _RGB_U8]).astype(np.float64) / 255.0
    return None


def _colors_u8(f: FieldDict) -> Optional[np.ndarray]:
    """Returns (N,3) uint8 colors, if the cloud has them."""
    if all(n in f for n in _RGB_U8):
        return np.column_stack([f[n] for n in _RGB_U8]).astype(np.uint8)
    c = _colors_float(f)
    if c is None:
        return None
    return np.clip(np.round(c * 255.0), 0, 255).astype(np.uint8)


def _expand_columns(name: str, a: np.ndarray) -> FieldDict:
    """(N,) arrays as is; (N,k) arrays into name_0 ... name_{k-1}."""
    a = np.asarray(a)
    if a.ndim == 1:
        return {name: a}
    if a.ndim == 2 and a.shape[1] == 1:
        return {name: a[:, 0]}
    if a.ndim == 2:
        return {f"{name}_{i}": a[:, i] for i in range(a.shape[1])}
    raise ValueError(f"Field '{name}' has an unsupported shape {a.shape}")


# ---------------------------------------------------------------------------
# pandas and Parquet
# ---------------------------------------------------------------------------
def to_dataframe(layer: LayerLike) -> Any:
    """Returns a pandas DataFrame with one column per point field."""
    pd = require("pandas", "to_dataframe()")
    return pd.DataFrame(_fields(layer))


def from_dataframe(
    df: Any, columns: Optional[Sequence[str]] = None
) -> "mrpt.maps.CGenericPointsMap":
    """Creates a point cloud from a pandas DataFrame with (at least) columns
    ``x`` and ``y``. Each column becomes a point field; signed integer columns
    are stored as floats."""
    cols = list(columns) if columns is not None else list(df.columns)
    return _core.layer_from_numpy(supported_fields({str(c): df[c].to_numpy() for c in cols}))


def to_parquet(layer: LayerLike, path: Union[str, os.PathLike], **kwargs: Any) -> None:
    """Saves the point fields as a Parquet file (needs pandas and pyarrow).
    Extra arguments go to ``pandas.DataFrame.to_parquet()``."""
    require("pyarrow", "to_parquet()")
    to_dataframe(layer).to_parquet(path, **kwargs)


def from_parquet(
    path: Union[str, os.PathLike], columns: Optional[Sequence[str]] = None
) -> "mrpt.maps.CGenericPointsMap":
    """Loads a point cloud from a Parquet file with one column per field
    (needs pandas and pyarrow)."""
    pd = require("pandas", "from_parquet()")
    require("pyarrow", "from_parquet()")
    return from_dataframe(pd.read_parquet(path, columns=columns))


# ---------------------------------------------------------------------------
# Open3D
# ---------------------------------------------------------------------------
def to_open3d(layer: LayerLike, tensor: bool = False) -> Any:
    """Converts a point cloud into Open3D.

    Args:
        layer: point cloud layer, or dict of fields.
        tensor: if False, returns an ``open3d.geometry.PointCloud`` with points,
            colors and normals (if present). If True, returns an
            ``open3d.t.geometry.PointCloud`` with ``positions``, ``colors``,
            ``normals``, plus all the other fields (e.g. ``intensity``) as
            per-point attributes of shape (N,1).
    """
    o3d = require("open3d", "to_open3d()")
    f = _fields(layer)
    xyz = np.column_stack([f["x"], f["y"], f["z"]])
    colors = _colors_float(f)
    normals = np.column_stack([f[n] for n in _NORMALS]) if all(n in f for n in _NORMALS) else None

    if not tensor:
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
        if colors is not None:
            pc.colors = o3d.utility.Vector3dVector(colors)
        if normals is not None:
            pc.normals = o3d.utility.Vector3dVector(normals.astype(np.float64))
        return pc

    pc = o3d.t.geometry.PointCloud()
    pc.point.positions = o3d.core.Tensor(xyz.astype(np.float32))
    if colors is not None:
        pc.point.colors = o3d.core.Tensor(colors.astype(np.float32))
    if normals is not None:
        pc.point.normals = o3d.core.Tensor(normals.astype(np.float32))
    skip = {"x", "y", "z", *_RGB_U8, *_RGB_F, *_NORMALS}
    for name, a in f.items():
        if name not in skip:
            pc.point[name] = o3d.core.Tensor(np.ascontiguousarray(a.reshape(-1, 1)))
    return pc


def from_open3d(pcd: Any) -> "mrpt.maps.CGenericPointsMap":
    """Creates a point cloud from an Open3D ``geometry.PointCloud`` or
    ``t.geometry.PointCloud``. Colors become ``color_rf, color_gf, color_bf``
    (float, [0,1]), or ``color_r, color_g, color_b`` for uint8 tensor colors;
    normals become ``normal_x, normal_y, normal_z``; other tensor attributes
    keep their names."""
    o3d = require("open3d", "from_open3d()")
    fields: Dict[str, np.ndarray] = {}
    if isinstance(pcd, o3d.geometry.PointCloud):
        xyz = np.asarray(pcd.points)
        attrs: Dict[str, np.ndarray] = {}
        if pcd.has_colors():
            attrs["colors"] = np.asarray(pcd.colors)
        if pcd.has_normals():
            attrs["normals"] = np.asarray(pcd.normals)
    else:
        xyz = pcd.point.positions.numpy()
        attrs = {k: pcd.point[k].numpy() for k in pcd.point if k != "positions"}

    fields["x"], fields["y"], fields["z"] = (xyz[:, i].astype(np.float32) for i in range(3))
    for name, a in attrs.items():
        if name == "colors" and a.ndim == 2 and a.shape[1] >= 3:
            names = _RGB_U8 if a.dtype == np.uint8 else _RGB_F
            dtype = np.uint8 if a.dtype == np.uint8 else np.float32
            for i, n in enumerate(names):
                fields[n] = a[:, i].astype(dtype)
        elif name == "normals" and a.ndim == 2 and a.shape[1] == 3:
            for i, n in enumerate(_NORMALS):
                fields[n] = a[:, i].astype(np.float32)
        else:
            fields.update(_expand_columns(name, a))
    return _core.layer_from_numpy(supported_fields(fields))


# ---------------------------------------------------------------------------
# PyVista
# ---------------------------------------------------------------------------
def to_pyvista(layer: LayerLike) -> Any:
    """Converts a point cloud into a ``pyvista.PolyData``, with all point fields
    as ``point_data`` arrays. If the cloud has colors, they are also added as an
    (N,3) uint8 ``RGB`` array, for ``plotter.add_mesh(..., rgb=True)``."""
    pv = require("pyvista", "to_pyvista()")
    f = _fields(layer)
    poly = pv.PolyData(np.column_stack([f["x"], f["y"], f["z"]]).astype(np.float32))
    for name, a in f.items():
        if name not in ("x", "y", "z"):
            poly.point_data[name] = a
    rgb = _colors_u8(f)
    if rgb is not None:
        poly.point_data["RGB"] = rgb
    return poly


def from_pyvista(mesh: Any) -> "mrpt.maps.CGenericPointsMap":
    """Creates a point cloud from the points and ``point_data`` of any PyVista
    dataset. (N,k) arrays become fields ``name_0 ... name_{k-1}``, except
    ``RGB`` / ``RGBA`` and ``Normals``, mapped into the MRPT color and normal
    fields if these are not present already."""
    xyz = np.asarray(mesh.points)
    fields: Dict[str, np.ndarray] = {
        "x": xyz[:, 0].astype(np.float32),
        "y": xyz[:, 1].astype(np.float32),
        "z": xyz[:, 2].astype(np.float32),
    }
    data = {name: np.asarray(mesh.point_data[name]) for name in mesh.point_data.keys()}
    for name, a in data.items():
        if name in ("RGB", "RGBA") and a.ndim == 2:
            if any(n in data for n in _RGB_U8 + _RGB_F):
                continue
            names = _RGB_U8 if a.dtype == np.uint8 else _RGB_F
            for i, n in enumerate(names):
                fields[n] = a[:, i]
        elif name == "Normals" and a.ndim == 2:
            if any(n in data for n in _NORMALS):
                continue
            for i, n in enumerate(_NORMALS):
                fields[n] = a[:, i].astype(np.float32)
        else:
            fields.update(_expand_columns(name, a))
    return _core.layer_from_numpy(supported_fields(fields))


# ---------------------------------------------------------------------------
# laspy (LAS / LAZ)
# ---------------------------------------------------------------------------
# LAS standard dimensions read into MRPT fields, with their types:
_LAS_STANDARD = {
    "classification": np.uint8,
    "return_number": np.uint8,
    "number_of_returns": np.uint8,
    "user_data": np.uint8,
    "point_source_id": np.uint16,
}


def from_laspy(las: Any, origin: Optional[Sequence[float]] = None) -> "mrpt.maps.CGenericPointsMap":
    """Creates a point cloud from a ``laspy.LasData`` object, or from a LAS/LAZ
    file path (read with ``laspy.read()``).

    Coordinates are stored as float32 (MRPT point clouds), which only keeps
    millimeter resolution for coordinates up to a few kilometers: pass an
    ``origin`` (x, y, z) to subtract from all points, e.g. for UTM data.

    ``intensity`` becomes a float field, colors ``color_r, color_g, color_b``
    (uint8), ``gps_time`` a float64 field, ``scan_angle`` a float field (in
    degrees), and classification, returns, user data, point source ID and
    extra dimensions keep their names.
    """
    laspy = require("laspy", "from_laspy()")
    if isinstance(las, (str, os.PathLike)):
        las = laspy.read(las)

    o = np.zeros(3) if origin is None else np.asarray(origin, dtype=np.float64)
    fields: Dict[str, np.ndarray] = {
        "x": (np.asarray(las.x, dtype=np.float64) - o[0]).astype(np.float32),
        "y": (np.asarray(las.y, dtype=np.float64) - o[1]).astype(np.float32),
        "z": (np.asarray(las.z, dtype=np.float64) - o[2]).astype(np.float32),
    }
    dims = set(las.point_format.dimension_names)
    if "intensity" in dims:
        fields["intensity"] = np.asarray(las.intensity, dtype=np.float32)
    if {"red", "green", "blue"} <= dims:
        rgb = [np.asarray(las[c], dtype=np.uint16) for c in ("red", "green", "blue")]
        # 16-bit colors per the LAS spec, though some writers use 8-bit values:
        shift = 8 if max(int(c.max(initial=0)) for c in rgb) > 255 else 0
        for c, n in zip(rgb, _RGB_U8):
            fields[n] = (c >> shift).astype(np.uint8)
    if "gps_time" in dims:
        fields["gps_time"] = np.asarray(las.gps_time, dtype=np.float64)
    if "scan_angle" in dims:  # LAS 1.4 formats: 0.006 degree units
        fields["scan_angle"] = np.asarray(las.scan_angle, dtype=np.float32) * 0.006
    elif "scan_angle_rank" in dims:
        fields["scan_angle"] = np.asarray(las.scan_angle_rank, dtype=np.float32)
    for name, dtype in _LAS_STANDARD.items():
        if name in dims:
            fields[name] = np.asarray(las[name]).astype(dtype)
    for name in las.point_format.extra_dimension_names:
        fields.update(_expand_columns(name, np.asarray(las[name])))
    return _core.layer_from_numpy(supported_fields(fields))


def to_laspy(layer: LayerLike, scale: float = 0.001) -> Any:
    """Converts a point cloud into a ``laspy.LasData`` (LAS 1.4, point format 7
    or 6, without colors), e.g. to write LAZ files with ``las.write("x.laz")``.

    Standard fields (``intensity``, colors, ``gps_time``, ``classification``,
    ``return_number``, ``number_of_returns``, ``user_data``,
    ``point_source_id``) go into the LAS standard dimensions, and all other
    fields into extra dimensions. ``scale`` is the coordinate resolution [m].
    """
    laspy = require("laspy", "to_laspy()")
    f = _fields(layer)
    rgb = _colors_u8(f)
    header = laspy.LasHeader(point_format=7 if rgb is not None else 6, version="1.4")
    xyz = np.column_stack([f["x"], f["y"], f["z"]]).astype(np.float64)
    header.offsets = xyz.min(axis=0) if len(xyz) else np.zeros(3)
    header.scales = np.array([scale, scale, scale])

    used = {"x", "y", "z", *_RGB_U8, *_RGB_F}
    extra = [
        (n, a)
        for n, a in f.items()
        if n not in used and n not in _LAS_STANDARD and n not in ("intensity", "gps_time")
    ]
    for name, a in extra:
        header.add_extra_dim(laspy.ExtraBytesParams(name=name, type=a.dtype))

    las = laspy.LasData(header)
    las.x, las.y, las.z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    if "intensity" in f:
        las.intensity = np.clip(np.round(f["intensity"]), 0, 65535).astype(np.uint16)
    if rgb is not None:
        las.red, las.green, las.blue = (rgb[:, i].astype(np.uint16) * 256 for i in range(3))
    if "gps_time" in f:
        las.gps_time = f["gps_time"].astype(np.float64)
    for name, dtype in _LAS_STANDARD.items():
        if name in f:
            las[name] = f[name].astype(dtype)
    for name, a in extra:
        las[name] = a
    return las

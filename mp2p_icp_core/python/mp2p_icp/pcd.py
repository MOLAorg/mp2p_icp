"""Reader and writer of PCD (Point Cloud Library) point cloud files, in pure
Python.

Reads ``ascii``, ``binary`` and ``binary_compressed`` files; writes ``ascii``
and ``binary`` ones. PCL packed colors (an ``rgb`` or ``rgba`` field) are
converted to and from the MRPT ``color_r, color_g, color_b`` fields.
"""

from __future__ import annotations

import os
import struct
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple, Union

import numpy as np
import mrpt.maps

from . import _core
from ._util import FieldDict, supported_fields, to_supported_dtype

_PCD_TO_NUMPY = {
    ("F", 4): "f4", ("F", 8): "f8",
    ("U", 1): "u1", ("U", 2): "u2", ("U", 4): "u4", ("U", 8): "u8",
    ("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4", ("I", 8): "i8",
}  # fmt: skip


def _lzf_decompress(data: bytes, out_len: int) -> bytes:
    """Decompresses an LZF stream, as used by PCD ``binary_compressed`` files."""
    out = bytearray(out_len)
    ip = 0
    op = 0
    n = len(data)
    while ip < n:
        ctrl = data[ip]
        ip += 1
        if ctrl < 32:
            # Literal run of ctrl+1 bytes:
            length = ctrl + 1
            out[op : op + length] = data[ip : ip + length]
            ip += length
            op += length
        else:
            # Back reference:
            length = ctrl >> 5
            ref = op - ((ctrl & 0x1F) << 8) - 1
            if length == 7:
                length += data[ip]
                ip += 1
            ref -= data[ip]
            ip += 1
            length += 2
            if ref < 0 or op + length > out_len:
                raise ValueError("Corrupted LZF data in PCD file")
            if ref + length <= op:
                out[op : op + length] = out[ref : ref + length]
            else:
                # Overlapping copy, e.g. runs of repeated bytes:
                for i in range(length):
                    out[op + i] = out[ref + i]
            op += length
    if op != out_len:
        raise ValueError("Corrupted LZF data in PCD file")
    return bytes(out)


def _parse_header(f) -> Tuple[Dict[str, List[str]], str]:
    header: Dict[str, List[str]] = {}
    while True:
        line = f.readline()
        if not line:
            raise ValueError("Unexpected end of file in the PCD header")
        tokens = line.decode("ascii", errors="replace").strip().split()
        if not tokens or tokens[0].startswith("#"):
            continue
        key = tokens[0].upper()
        if key == "DATA":
            return header, tokens[1].lower()
        header[key] = tokens[1:]


def _unpack_rgb(packed: np.ndarray) -> FieldDict:
    """PCL packed colors (float32 or uint32 0xAARRGGBB) into separate channels."""
    v = np.ascontiguousarray(packed).view(np.uint32)
    return {
        "color_r": ((v >> 16) & 0xFF).astype(np.uint8),
        "color_g": ((v >> 8) & 0xFF).astype(np.uint8),
        "color_b": (v & 0xFF).astype(np.uint8),
    }


def read_pcd(path: Union[str, os.PathLike]) -> FieldDict:
    """Reads a PCD file as a dict ``{field: 1-D array}``.

    Fields with ``COUNT > 1`` become ``name_0, name_1...``; padding fields
    (``_``) are dropped; packed ``rgb`` / ``rgba`` fields are unpacked into
    ``color_r, color_g, color_b`` (uint8).
    """
    with open(path, "rb") as f:
        header, data_format = _parse_header(f)
        names = header["FIELDS"]
        sizes = [int(s) for s in header.get("SIZE", ["4"] * len(names))]
        types = [t.upper() for t in header.get("TYPE", ["F"] * len(names))]
        counts = [int(c) for c in header.get("COUNT", ["1"] * len(names))]
        if "POINTS" in header:
            n = int(header["POINTS"][0])
        else:
            n = int(header["WIDTH"][0]) * int(header.get("HEIGHT", ["1"])[0])

        columns: List[Tuple[str, str]] = []  # expanded (name, dtype)
        for name, size, typ, count in zip(names, sizes, types, counts):
            if (typ, size) not in _PCD_TO_NUMPY:
                raise ValueError(f"Unsupported PCD field type: {typ}{size}")
            dt = _PCD_TO_NUMPY[(typ, size)]
            if count == 1:
                columns.append((name, dt))
            else:
                columns.extend((f"{name}_{i}", dt) for i in range(count))
        # Unique names for repeated padding fields:
        columns = [(f"_{i}" if c == "_" else c, t) for i, (c, t) in enumerate(columns)]
        dtype = np.dtype([(c, "<" + t) for c, t in columns])

        if data_format == "ascii":
            rows = f.read().decode("ascii").splitlines()
            values = np.loadtxt(rows[:n], dtype=np.float64, ndmin=2) if n > 0 else None
            data = {
                c: (values[:, i].astype(t) if values is not None else np.zeros(0, t))
                for i, (c, t) in enumerate(columns)
            }
        elif data_format == "binary":
            buf = f.read(dtype.itemsize * n)
            if len(buf) < dtype.itemsize * n:
                raise ValueError("Unexpected end of file in the PCD data")
            records = np.frombuffer(buf, dtype=dtype, count=n)
            data = {c: records[c].astype(t) for c, t in columns}
        elif data_format == "binary_compressed":
            compressed_size, raw_size = struct.unpack("<II", f.read(8))
            raw = _lzf_decompress(f.read(compressed_size), raw_size)
            # Data is stored field by field (all values of a field together):
            data = {}
            offset = 0
            for c, t in columns:
                nbytes = np.dtype(t).itemsize * n
                data[c] = np.frombuffer(raw, dtype="<" + t, count=n, offset=offset).astype(t)
                offset += nbytes
        else:
            raise ValueError(f"Unsupported PCD DATA format: {data_format}")

    out: FieldDict = {}
    for c, a in data.items():
        if c.startswith("_"):
            continue
        if c in ("rgb", "rgba") and a.dtype.itemsize == 4:
            out.update(_unpack_rgb(a))
        else:
            out[c] = a
    return out


def load_pcd(path: Union[str, os.PathLike]) -> "mrpt.maps.CGenericPointsMap":
    """Loads a PCD file into an ``mrpt.maps.CGenericPointsMap``, with all its
    fields. See :func:`read_pcd` for the field conversions. Signed and 64-bit
    integer fields are stored as floats, since MRPT point clouds only hold 8, 16
    and 32-bit unsigned integer fields."""
    if not os.path.isfile(path):
        raise FileNotFoundError(f"PCD file not found: '{path}'")
    return _core.layer_from_numpy(supported_fields(read_pcd(path)))


def _pack_rgb(fields: FieldDict) -> FieldDict:
    """MRPT color channels into a PCL packed ``rgb`` float32 field."""
    names_u8 = ("color_r", "color_g", "color_b")
    names_f = ("color_rf", "color_gf", "color_bf")
    if all(n in fields for n in names_u8):
        rgb = [fields[n].astype(np.uint32) for n in names_u8]
        drop = names_u8
    elif all(n in fields for n in names_f):
        rgb = [
            np.clip(np.round(fields[n].astype(np.float64) * 255.0), 0, 255).astype(np.uint32)
            for n in names_f
        ]
        drop = names_f
    else:
        return fields
    packed = ((rgb[0] << 16) | (rgb[1] << 8) | rgb[2]).astype(np.uint32).view(np.float32)
    out = {k: v for k, v in fields.items() if k not in drop}
    out["rgb"] = packed
    return out


def write_pcd(
    fields: Mapping[str, np.ndarray], path: Union[str, os.PathLike], binary: bool = True
) -> None:
    """Writes a dict ``{field: 1-D array}`` as a PCD v0.7 file (``binary`` or
    ``ascii``). MRPT color channels are written as a PCL packed ``rgb``
    field."""
    data = _pack_rgb({k: to_supported_dtype(np.asarray(v).reshape(-1)) for k, v in fields.items()})
    names = list(data.keys())
    n = len(next(iter(data.values()))) if data else 0
    if any(len(a) != n for a in data.values()):
        raise ValueError("All fields must have the same length")

    types = []
    for name in names:
        dt = data[name].dtype
        types.append(("F" if dt.kind == "f" else "U", dt.itemsize))

    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        f"FIELDS {' '.join(names)}\n"
        f"SIZE {' '.join(str(s) for _, s in types)}\n"
        f"TYPE {' '.join(t for t, _ in types)}\n"
        f"COUNT {' '.join('1' for _ in names)}\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        f"DATA {'binary' if binary else 'ascii'}\n"
    )
    dtype = np.dtype([(name, "<" + data[name].dtype.str[1:]) for name in names])
    records = np.empty(n, dtype=dtype)
    for name in names:
        records[name] = data[name]

    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        if binary:
            f.write(records.tobytes())
        else:
            fmts = []
            for name in names:
                dt = data[name].dtype
                fmts.append("%.9g" if dt == np.float32 else "%.17g" if dt.kind == "f" else "%d")
            # The packed rgb float must be written with all its bits:
            if "rgb" in names:
                fmts[names.index("rgb")] = "%.9g"
            table = np.column_stack([data[name].astype(np.float64) for name in names])
            np.savetxt(f, table.reshape(n, len(names)), fmt=fmts)


def save_pcd(
    layer: Union["mrpt.maps.CMetricMap", Mapping[str, np.ndarray]],
    path: Union[str, os.PathLike],
    binary: bool = True,
    fields: Optional[Sequence[str]] = None,
) -> None:
    """Saves a point cloud layer (or a dict of field arrays) as a PCD file.

    Args:
        layer: an ``mrpt.maps`` point cloud (or any layer providing one), or a
            dict ``{field: 1-D array}``.
        path: output file.
        binary: ``binary`` (default) or ``ascii`` PCD data.
        fields: optional subset of fields to write, in this order. Default:
            all.
    """
    data: Dict[str, Any] = (
        dict(layer) if isinstance(layer, Mapping) else _core.layer_to_numpy(layer)
    )
    if fields is not None:
        missing = [f for f in fields if f not in data]
        if missing:
            raise KeyError(f"Fields not found in the point cloud: {missing}")
        data = {f: data[f] for f in fields}
    write_pcd(data, path, binary=binary)

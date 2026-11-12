"""Reader of PLY point clouds with all their per-vertex properties.

PLY files are written by :func:`mp2p_icp.save_ply` (C++). This reader loads the
``vertex`` element of ASCII and binary (little or big endian) PLY files, with
all its scalar properties, so files written by ``save_ply()`` load back with
all their fields.
"""

from __future__ import annotations

import os
from typing import BinaryIO, Dict, List, Tuple, Union

import numpy as np
import mrpt.maps

from . import _core
from ._util import FieldDict, supported_fields

_PLY_TYPES = {
    "char": "i1", "int8": "i1",
    "uchar": "u1", "uint8": "u1",
    "short": "i2", "int16": "i2",
    "ushort": "u2", "uint16": "u2",
    "int": "i4", "int32": "i4",
    "uint": "u4", "uint32": "u4",
    "float": "f4", "float32": "f4",
    "double": "f8", "float64": "f8",
}  # fmt: skip

# Common PLY property names, into the MRPT point field names:
_PLY_TO_MRPT_NAMES = {"nx": "normal_x", "ny": "normal_y", "nz": "normal_z"}


class _Element:
    def __init__(self, name: str, count: int):
        self.name = name
        self.count = count
        self.props: List[Tuple[str, str]] = []  # (name, numpy dtype code)
        self.has_lists = False


def _parse_header(f: BinaryIO) -> Tuple[str, List[_Element]]:
    if f.readline().strip() != b"ply":
        raise ValueError("Not a PLY file")
    fmt = None
    elements: List[_Element] = []
    while True:
        line = f.readline()
        if not line:
            raise ValueError("Unexpected end of file in the PLY header")
        tokens = line.decode("ascii", errors="replace").split()
        if not tokens or tokens[0] in ("comment", "obj_info"):
            continue
        if tokens[0] == "end_header":
            break
        if tokens[0] == "format":
            fmt = tokens[1]
        elif tokens[0] == "element":
            elements.append(_Element(tokens[1], int(tokens[2])))
        elif tokens[0] == "property":
            if not elements:
                raise ValueError("PLY property before any element")
            if tokens[1] == "list":
                elements[-1].has_lists = True
                elements[-1].props.append((tokens[4], "list"))
            else:
                if tokens[1] not in _PLY_TYPES:
                    raise ValueError(f"Unsupported PLY property type: {tokens[1]}")
                elements[-1].props.append((tokens[2], _PLY_TYPES[tokens[1]]))
    if fmt not in ("ascii", "binary_little_endian", "binary_big_endian"):
        raise ValueError(f"Unsupported PLY format: {fmt}")
    return fmt, elements


def read_ply(path: Union[str, os.PathLike]) -> FieldDict:
    """Reads the vertices of a PLY file as a dict ``{property: 1-D array}``.

    Property names and types are kept as in the file, except ``nx, ny, nz``,
    renamed as ``normal_x, normal_y, normal_z``.
    """
    with open(path, "rb") as f:
        fmt, elements = _parse_header(f)
        names = [e.name for e in elements]
        if "vertex" not in names:
            raise ValueError("The PLY file has no 'vertex' element")
        vertex_idx = names.index("vertex")
        vertex = elements[vertex_idx]
        if vertex.has_lists:
            raise ValueError("PLY vertices with list properties are not supported")

        if fmt == "ascii":
            # Skip the lines of the elements before the vertices:
            for e in elements[:vertex_idx]:
                for _ in range(e.count):
                    f.readline()
            rows = [f.readline().decode("ascii") for _ in range(vertex.count)]
            values = np.loadtxt(rows, dtype=np.float64, ndmin=2) if rows else None
            data = {
                n: (values[:, i].astype(t) if values is not None else np.zeros(0, dtype=t))
                for i, (n, t) in enumerate(vertex.props)
            }
        else:
            endian = "<" if fmt == "binary_little_endian" else ">"
            for e in elements[:vertex_idx]:
                if e.has_lists:
                    raise ValueError(
                        "Binary PLY files with list elements before the vertices are not supported"
                    )
                f.seek(e.count * np.dtype([(n, endian + t) for n, t in e.props]).itemsize, 1)
            dtype = np.dtype([(n, endian + t) for n, t in vertex.props])
            buf = f.read(dtype.itemsize * vertex.count)
            if len(buf) < dtype.itemsize * vertex.count:
                raise ValueError("Unexpected end of file in the PLY vertex data")
            records = np.frombuffer(buf, dtype=dtype, count=vertex.count)
            data = {n: records[n].astype(np.dtype(t).newbyteorder("=")) for n, t in vertex.props}

    return {_PLY_TO_MRPT_NAMES.get(n, n): a for n, a in data.items()}


def _to_mrpt_colors(fields: FieldDict) -> FieldDict:
    """Renames PLY red/green/blue into the MRPT color field names."""
    out: Dict[str, np.ndarray] = {}
    for name, a in fields.items():
        if name in ("red", "green", "blue"):
            name = f"color_{name[0]}" + ("f" if a.dtype.kind == "f" else "")
        out[name] = a
    return out


def load_ply(path: Union[str, os.PathLike]) -> "mrpt.maps.CGenericPointsMap":
    """Loads a PLY point cloud into an ``mrpt.maps.CGenericPointsMap``, with all
    its per-vertex properties as point fields.

    ``red, green, blue`` are stored as ``color_r, color_g, color_b`` (uint8) or
    ``color_rf, color_gf, color_bf`` (float), and ``nx, ny, nz`` as
    ``normal_x, normal_y, normal_z``. Signed integer properties are stored as
    floats, since MRPT point clouds only hold unsigned integer fields.
    """
    if not os.path.isfile(path):
        raise FileNotFoundError(f"PLY file not found: '{path}'")
    return _core.layer_from_numpy(supported_fields(_to_mrpt_colors(read_ply(path))))

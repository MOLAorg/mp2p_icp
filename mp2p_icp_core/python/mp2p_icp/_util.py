"""Internal helpers shared by the pure Python modules of mp2p_icp."""

from __future__ import annotations

import importlib
from types import ModuleType
from typing import Dict

import numpy as np

# Field dtypes that MRPT point clouds can store:
SUPPORTED_DTYPES = (np.float32, np.float64, np.uint8, np.uint16, np.uint32)

FieldDict = Dict[str, np.ndarray]


def require(module: str, feature: str, pip_name: str | None = None) -> ModuleType:
    """Imports an optional dependency, or raises an ImportError explaining how
    to install it."""
    try:
        return importlib.import_module(module)
    except ImportError as e:
        raise ImportError(
            f"{feature} requires the optional package '{pip_name or module}': "
            f"pip install {pip_name or module}"
        ) from e


def to_supported_dtype(arr: np.ndarray) -> np.ndarray:
    """Converts a 1-D array into a dtype storable in MRPT point clouds, keeping
    its values: booleans and small signed integers become float32, 32-bit and
    64-bit integers become float64."""
    arr = np.asarray(arr)
    if arr.dtype.type in SUPPORTED_DTYPES:
        return arr
    if arr.dtype == np.bool_:
        return arr.astype(np.uint8)
    if arr.dtype.kind in "iu" and arr.dtype.itemsize <= 2:
        return arr.astype(np.float32)
    if arr.dtype.kind in "iuf":
        return arr.astype(np.float64)
    raise TypeError(f"Unsupported field dtype: {arr.dtype}")


def supported_fields(fields: FieldDict) -> FieldDict:
    """Applies to_supported_dtype() to all fields, as contiguous 1-D arrays."""
    return {
        name: np.ascontiguousarray(to_supported_dtype(np.asarray(a).reshape(-1)))
        for name, a in fields.items()
    }

"""
mp2p_icp: Python API of the mp2p_icp C++ libraries, part of MOLA.

Main entry points:

- ``MetricMap``: metric maps (``.mm`` files) with named layers (point clouds,
  voxel maps...), metadata and georeferencing.
- ``layer_to_numpy()`` / ``layer_from_numpy()``: point cloud fields to and
  from NumPy arrays.
- Point cloud files: ``load_ply()`` / ``save_ply()``, ``load_pcd()`` /
  ``save_pcd()``, ``save_las()``, ``save_txt()``, ``load_xyz_file()``.

Point clouds are MRPT objects (``mrpt.maps.CGenericPointsMap``...) and poses
are ``mrpt.poses`` / ``mrpt.math`` types, from the ``mrpt`` Python package.
"""

# The MRPT Python modules register the MRPT types used by the extension:
import mrpt.rtti  # noqa: F401
import mrpt.serialization  # noqa: F401
import mrpt.containers  # noqa: F401
import mrpt.math  # noqa: F401
import mrpt.img  # noqa: F401
import mrpt.poses  # noqa: F401
import mrpt.topography  # noqa: F401
import mrpt.viz  # noqa: F401
import mrpt.obs  # noqa: F401
import mrpt.maps  # noqa: F401

from ._core import (
    PT_LAYER_PLANE_CENTROIDS,
    PT_LAYER_RAW,
    ColorMode,
    Georeferencing,
    LayersView,
    MetricMap,
    PlanePatch,
    RenderParams,
    RenderParamsLines,
    RenderParamsPlanes,
    RenderParamsPointLayer,
    RenderParamsPoints,
    __version__,
    layer_from_numpy,
    layer_points_view,
    layer_to_numpy,
    layer_visualization,
    load_plugins,
    load_xyz_file,
    load_yaml,
    save_las,
    save_ply,
    save_txt,
)
from .pcd import load_pcd, save_pcd
from .ply import load_ply

__all__ = [
    "PT_LAYER_PLANE_CENTROIDS",
    "PT_LAYER_RAW",
    "ColorMode",
    "Georeferencing",
    "LayersView",
    "MetricMap",
    "PlanePatch",
    "RenderParams",
    "RenderParamsLines",
    "RenderParamsPlanes",
    "RenderParamsPointLayer",
    "RenderParamsPoints",
    "__version__",
    "layer_from_numpy",
    "layer_points_view",
    "layer_to_numpy",
    "layer_visualization",
    "load_pcd",
    "load_ply",
    "load_plugins",
    "load_xyz_file",
    "load_yaml",
    "save_las",
    "save_pcd",
    "save_ply",
    "save_txt",
]

"""
mp2p_icp: Python API of the mp2p_icp C++ libraries, part of MOLA.

Main entry points:

- ``MetricMap``: metric maps (``.mm`` files) with named layers (point clouds,
  voxel maps...), metadata and georeferencing.
- ``layer_to_numpy()`` / ``layer_from_numpy()``: point cloud fields to and
  from NumPy arrays.
- ``align()``: one-call point cloud registration with bundled configurations;
  ``ICP`` / ``icp_pipeline_from_yaml()`` for full control.
- ``FilterPipeline``, ``Generators``, ``simplemap_to_metricmap()``: map
  building pipelines.
- Point cloud files: ``load_ply()`` / ``save_ply()``, ``load_pcd()`` /
  ``save_pcd()``, ``save_las()``, ``save_txt()``, ``load_xyz_file()``.
- ``render()``: off-screen rendering of maps into images.
- ``mp2p_icp.interop``: conversions to and from pandas, Open3D, PyVista,
  laspy and Parquet (optional dependencies).

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
    FilterPipeline,
    Generators,
    Georeferencing,
    GravityPrior,
    ICP,
    IterationDetails,
    IterationInfo,
    IterTermReason,
    LayersView,
    LogRecord,
    MetricMap,
    Pairings,
    Parameters,
    PlanePatch,
    RenderParams,
    RenderParamsLines,
    RenderParamsPlanes,
    RenderParamsPointLayer,
    RenderParamsPoints,
    Results,
    __version__,
    apply_filter_pipeline,
    apply_generators,
    filter_pipeline_from_yaml,
    generators_from_yaml,
    icp_pipeline_from_yaml,
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
    simplemap_to_metricmap,
)
from .align import align, config_path, list_configs, to_metric_map
from .pcd import load_pcd, save_pcd
from .ply import load_ply
from .render import render
from . import interop

__all__ = [
    "PT_LAYER_PLANE_CENTROIDS",
    "PT_LAYER_RAW",
    "ColorMode",
    "FilterPipeline",
    "Generators",
    "Georeferencing",
    "GravityPrior",
    "ICP",
    "IterationDetails",
    "IterationInfo",
    "IterTermReason",
    "LayersView",
    "LogRecord",
    "MetricMap",
    "Pairings",
    "Parameters",
    "PlanePatch",
    "RenderParams",
    "RenderParamsLines",
    "RenderParamsPlanes",
    "RenderParamsPointLayer",
    "RenderParamsPoints",
    "Results",
    "__version__",
    "align",
    "apply_filter_pipeline",
    "apply_generators",
    "config_path",
    "filter_pipeline_from_yaml",
    "generators_from_yaml",
    "icp_pipeline_from_yaml",
    "interop",
    "layer_from_numpy",
    "layer_points_view",
    "layer_to_numpy",
    "layer_visualization",
    "list_configs",
    "load_pcd",
    "load_ply",
    "load_plugins",
    "load_xyz_file",
    "load_yaml",
    "render",
    "save_las",
    "save_pcd",
    "save_ply",
    "save_txt",
    "simplemap_to_metricmap",
    "to_metric_map",
]

"""One-call point cloud registration with bundled ICP configurations."""

from __future__ import annotations

import copy
import os
from pathlib import Path
from typing import Any, List, Mapping, Optional, Union

import numpy as np
import mrpt.maps

from . import _core

_CONFIG_DIR = Path(__file__).resolve().parent / "configs"

PointCloudLike = Union[_core.MetricMap, "mrpt.maps.CMetricMap", np.ndarray, Mapping[str, Any]]


def list_configs() -> List[str]:
    """Returns the names of the bundled ICP configurations, usable as the
    ``config`` argument of :func:`align`."""
    return sorted(p.stem for p in _CONFIG_DIR.glob("*.yaml"))


def config_path(name: str) -> Path:
    """Returns the path of a bundled ICP configuration file, e.g. to copy it
    and tune it. See :func:`list_configs`."""
    path = _CONFIG_DIR / f"{name}.yaml"
    if not path.is_file():
        raise ValueError(
            f"Unknown bundled ICP configuration '{name}'. Available: {', '.join(list_configs())}"
        )
    return path


def _resolve_config(config: Any) -> Any:
    """Bundled configuration names into their paths; anything else as is."""
    if isinstance(config, str) and config in list_configs() and not os.path.exists(config):
        return str(config_path(config))
    return config


def to_metric_map(data: PointCloudLike, layer: str = _core.PT_LAYER_RAW) -> _core.MetricMap:
    """Returns ``data`` as a :class:`MetricMap`.

    Args:
        data: a MetricMap (returned as is), an ``mrpt.maps.CMetricMap`` layer,
            an array of shape (N,3) or (N,2), or a dict of per-point field
            arrays (see :func:`layer_from_numpy`).
        layer: name of the layer for anything that is not a MetricMap.
    """
    if isinstance(data, _core.MetricMap):
        return data
    m = _core.MetricMap()
    m.layers[layer] = data
    return m


def align(
    local_map: PointCloudLike,
    global_map: PointCloudLike,
    config: Any = "3d-point2point",
    initial_guess: Any = None,
    *,
    variables: Optional[Mapping[str, Any]] = None,
    parameters: Union[_core.Parameters, Mapping[str, Any], None] = None,
    prior: Any = None,
    gravity_prior: Optional[_core.GravityPrior] = None,
    log: Optional[_core.LogRecord] = None,
    layer: str = _core.PT_LAYER_RAW,
    verbosity: str = "WARN",
) -> _core.Results:
    """Registers two point clouds or maps: finds the pose of ``local_map`` with
    respect to ``global_map``.

    Args:
        local_map: the cloud to align: a MetricMap, an ``mrpt.maps`` point
            cloud, an array of shape (N,3), or a dict of field arrays.
        global_map: the reference cloud, with the same accepted types.
        config: a bundled configuration name (see :func:`list_configs`), or an
            ICP YAML file path, dict or ``mrpt.containers.YAML`` node. If it has
            a ``filters`` list, it is applied to copies of both maps first.
        initial_guess: initial pose of the local map: ``mrpt.math.TPose3D``,
            ``mrpt.poses.CPose3D``, a 4x4 matrix, or ``[x, y, z, yaw, pitch,
            roll]`` (meters, radians). Default: identity.
        variables: values of the ``${VAR}`` variables of the configuration,
            e.g. ``{"ICP_THRESHOLD": 0.1}`` for the bundled configurations.
        parameters: optional :class:`Parameters`, or a dict with some of their
            fields, overriding those of the configuration.
        prior: optional ``mrpt.poses.CPose3DPDFGaussianInf`` pose prior.
        gravity_prior: optional :class:`GravityPrior`.
        log: optional :class:`LogRecord`, filled with this run.
        layer: layer name for inputs that are not MetricMaps. The bundled 3D
            configurations expect "raw".
        verbosity: log level of the pipeline modules: "DEBUG", "INFO", "WARN"
            or "ERROR".

    Returns:
        The ICP :class:`Results`; ``results.optimal_tf.mean`` is the pose of
        the local map in the global frame, and ``results.quality`` a value in
        [0,1] defined by the configuration (for the bundled 3D ones, the
        overlap: the fraction of local points with a global point closer than
        ``ICP_QUALITY_THRESHOLD``).
    """
    cfg = _core.load_yaml(_resolve_config(config), variables)
    if not isinstance(cfg, dict):
        raise ValueError("The ICP configuration must be a YAML map")

    icp = _core.ICP.from_yaml(cfg, verbosity=verbosity)

    local = to_metric_map(local_map, layer)
    glob = to_metric_map(global_map, layer)

    if cfg.get("filters"):
        filters = _core.FilterPipeline.from_yaml(cfg["filters"], verbosity=verbosity)
        # Filters work in place: run them on copies of the inputs.
        local = local.copy()
        glob = glob.copy()
        filters.apply(local)
        filters.apply(glob)

    params = icp.parameters
    if isinstance(parameters, _core.Parameters):
        params = parameters
    elif parameters is not None:
        params = copy.copy(params)
        for name, value in parameters.items():
            if not hasattr(params, name):
                raise AttributeError(f"Unknown ICP parameter: '{name}'")
            setattr(params, name, value)

    return icp.align(
        local,
        glob,
        initial_guess,
        parameters=params,
        prior=prior,
        gravity_prior=gravity_prior,
        log=log,
    )

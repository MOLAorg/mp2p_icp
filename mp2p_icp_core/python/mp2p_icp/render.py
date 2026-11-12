"""Off-screen rendering of metric maps into images, as the ``mm-render`` tool.

Needs the ``mrpt.opengl`` Python module and an OpenGL (EGL) capable system; it
works without a display.
"""

from __future__ import annotations

import math
import os
from typing import Any, Optional, Sequence, Tuple, Union

import numpy as np
import mrpt.img
import mrpt.maps
import mrpt.math
import mrpt.viz

from . import _core
from .align import to_metric_map

_COLORMAPS = {
    "jet": "cmJET",
    "hot": "cmHOT",
    "gray": "cmGRAYSCALE",
}


def _layer_params(point_size: float, colormap: Optional[str]) -> _core.RenderParamsPointLayer:
    rp = _core.RenderParamsPointLayer()
    rp.pointSize = point_size
    cm = _core.ColorMode()
    if colormap is None:
        cm.keep_original_cloud_color = True
        rp.force_alpha_channel = False
    else:
        if colormap not in _COLORMAPS:
            raise ValueError(f"Invalid colormap '{colormap}': use one of {list(_COLORMAPS)}")
        cm.colorMap = getattr(mrpt.img.TColormap, _COLORMAPS[colormap])
        cm.recolorizeByField = "z"
    rp.colorMode = cm
    return rp


def _bounding_box(layers: Sequence[Any]) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    bbox = None
    for layer in layers:
        # Empty layers report a degenerate box at the origin: skip them.
        if layer.isEmpty():
            continue
        bb = layer.boundingBox()
        lo = np.array([bb.min.x, bb.min.y, bb.min.z])
        hi = np.array([bb.max.x, bb.max.y, bb.max.z])
        bbox = (lo, hi) if bbox is None else (np.minimum(bbox[0], lo), np.maximum(bbox[1], hi))
    return bbox


def render(
    map: Union[_core.MetricMap, "mrpt.maps.CMetricMap", np.ndarray],
    width: int = 1280,
    height: int = 720,
    azimuth: float = -45.0,
    elevation: float = 35.0,
    *,
    distance: Optional[float] = None,
    target: Optional[Sequence[float]] = None,
    fov: float = 45.0,
    layers: Optional[Sequence[str]] = None,
    background: Sequence[float] = (1.0, 1.0, 1.0),
    colormap: Optional[str] = None,
    point_size: float = 1.0,
    output: Union[str, os.PathLike, None] = None,
) -> np.ndarray:
    """Renders a map (or a single layer or point array) into an RGB image.

    Args:
        map: a MetricMap, an ``mrpt.maps`` layer, or an (N,3) point array.
        width, height: image size, in pixels.
        azimuth, elevation: camera orientation, in degrees.
        distance: camera distance to the target [m]. Default: fit the bounding
            box of the rendered layers.
        target: point the camera looks at (x, y, z). Default: the center of the
            bounding box of the rendered layers.
        fov: vertical field of view [deg]; 0 for an orthographic view. With
            MRPT versions whose Python ``CCamera`` cannot set it, its default
            (30 deg) is used instead.
        layers: names of the layers to render. Default: all.
        background: background color (r, g, b), in [0,1].
        colormap: if given ("jet", "hot", "gray"), points are colored by height
            with it; otherwise, layers keep their own colors.
        point_size: point size, in pixels.
        output: optional image file to save (e.g. ``.png``, ``.jpg``).

    Returns:
        The image, as a NumPy array of shape (height, width, 3) and dtype uint8.
    """
    import mrpt.opengl  # optional: needs OpenGL libraries

    m = to_metric_map(map)
    names = list(layers) if layers is not None else list(m.layers.keys())
    selected = []
    for name in names:
        if name not in m.layers:
            raise KeyError(f"Layer not found in the map: '{name}'")
        selected.append(m.layers[name])

    rp = _layer_params(point_size, colormap)
    objs = mrpt.viz.CSetOfObjects()
    for layer in selected:
        objs.insert(_core.layer_visualization(layer, rp))

    bbox = _bounding_box(selected)
    center = np.zeros(3)
    diag = 10.0
    if bbox is not None:
        center = 0.5 * (bbox[0] + bbox[1])
        diag = max(1.0, float(np.linalg.norm(bbox[1] - bbox[0])))
    if target is not None:
        center = np.asarray(target, dtype=np.float64).reshape(3)

    cam = mrpt.viz.CCamera()
    cam.setAzimuthDegrees(float(azimuth))
    cam.setElevationDegrees(float(elevation))
    if hasattr(cam, "setPointingAt"):
        cam.setPointingAt(float(center[0]), float(center[1]), float(center[2]))
    else:
        # Older MRPT Python APIs: the camera always looks at the origin, so
        # move the scene instead.
        objs.setLocation(float(-center[0]), float(-center[1]), float(-center[2]))

    # Default distance: fit the sphere enclosing the bounding box into the
    # narrower of the two image dimensions.
    aspect = width / height
    radius = 0.5 * diag
    if fov <= 0 and hasattr(cam, "setOrthogonal"):
        cam.setOrthogonal(True)
        # The orthographic view height is half the zoom distance:
        default_dist = 4.0 * radius / min(1.0, aspect)
    else:
        if fov <= 0 or not hasattr(cam, "setProjectiveFOVdeg"):
            fov = 30.0  # CCamera default
        else:
            cam.setProjectiveFOVdeg(float(fov))
        half_v = 0.5 * math.radians(fov)
        half_h = math.atan(math.tan(half_v) * aspect)
        default_dist = radius / math.sin(min(half_v, half_h))
    zoom = float(distance if distance is not None else default_dist)
    cam.setZoomDistance(zoom)

    scene = mrpt.viz.Scene()
    scene.insert(objs)
    vp = scene.getViewport()
    vp.setCustomBackgroundColor(mrpt.img.TColorf(*[float(c) for c in background[:3]]))
    if hasattr(vp, "setViewportClipDistances"):
        vp.setViewportClipDistances(0.1, 10.0 * diag + zoom)

    renderer = mrpt.opengl.CFBORender(width, height)
    renderer.setCamera(cam)
    img = renderer.render_RGB(scene)
    if output is not None and not img.saveToFile(os.fspath(output)):
        raise OSError(f"Error saving image to: '{output}'")
    return np.array(img.as_numpy()[:, :, :3], dtype=np.uint8, copy=True)

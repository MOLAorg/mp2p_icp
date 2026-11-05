.. _app_mm-render:

===============================
Application: ``mm-render``
===============================

CLI tool to render the layers of a metric map file (``.mm``) into an image file
(``.png``, ``.jpg``), off-screen and without any GUI. It is intended for scripted
generation of figures for papers, slides or documentation, with an exactly
reproducible camera.

Each layer is rendered with its own visualization method, so any map type works:
point clouds, voxel maps, NDT, TSDF meshes, grid maps, etc. Voxel maps are drawn as
occupied voxels colored by height. Custom map types from ``libmola_metric_maps.so``
are supported, since that plugin is always loaded if available.

The camera orbits around a target point, with the same azimuth / elevation / distance
convention as :ref:`mm-viewer <app_mm-viewer>`. If ``--target`` or ``--distance``
are not given, they are chosen to frame the bounding box of the rendered layers.

Usage
-----

.. code-block:: bash

    Usage: mm-render [OPTIONS] input

    Positionals:
      input TEXT REQUIRED         Load this metric map file (*.mm)

    Options:
      -h,--help                   Print this help message and exit
      -o,--output TEXT REQUIRED   Output image file (*.png, *.jpg)
      -y,--layers TEXT ...        Comma-separated list of layers to render (default: all layers)
      -l,--load-plugins TEXT      One or more (comma separated) *.so files to load as plugins. libmola_metric_maps.so is always tried.
      --width UINT [1920]         Image width, in pixels
      --height UINT [1080]        Image height, in pixels
      --azimuth FLOAT [-45]       Camera azimuth, in degrees
      --elevation FLOAT [35]      Camera elevation, in degrees
      --distance FLOAT            Camera distance to the target point, in meters (default: fit the layers bounding box)
      --target FLOAT x 3          Camera target point 'x,y,z', in meters (default: center of the layers bounding box)
      --fov FLOAT [45]            Vertical field of view, in degrees. Use 0 for orthographic
      --background TEXT [FFFFFF]  Background color, as a hex RRGGBB string
      --colormap TEXT:{jet,hot,gray}
                                  Recolor points and voxels by height with this colormap (default: keep the layer colors; voxel maps use 'jet')
      --point-size FLOAT [1]      Point size, in pixels
      --ssao                      Enable screen-space ambient occlusion


Example usage
=============

Render all layers with an automatic camera:

.. code-block:: bash

    mm-render myMap.mm -o map.png

Render one point cloud layer, recolored by height, with bigger points:

.. code-block:: bash

    mm-render myMap.mm -y points -o points.png --colormap jet --point-size 2.5

Render several layers of the same map from a fixed camera, so all images can be
compared side by side (e.g. maps generated with :ref:`sm2mm <app_sm2mm>` from a
pipeline with ``CVoxelMap``, ``mola::TSDF`` and ``mola::NDT`` layers):

.. code-block:: bash

    CAM="--azimuth -110 --elevation 38 --distance 46 --target 1,31,0 --fov 40"
    for ly in voxels tsdf ndt; do
      mm-render map.mm -y $ly -o render_$ly.png --width 2400 --height 1500 $CAM --ssao
    done

Top view with an orthographic camera:

.. code-block:: bash

    mm-render myMap.mm -o top.png --elevation 90 --fov 0

.. note::

   Rendering uses an off-screen OpenGL context (EGL), so it works on headless machines
   and over SSH, as long as an OpenGL driver (or Mesa's software renderer) is available.

.. _app_mm2kml:

===============================
Application: ``mm2kml``
===============================

Overview
--------

``mm2kml`` exports a **georeferenced** metric map (\*.mm) as a Google Earth ``*.kmz`` file,
optionally together with the robot trajectory. It is a quick way to visually check the
quality of a map georeferencing against satellite imagery, and to produce fly-by videos of
maps.

.. image:: https://mrpt.github.io/imgs/mm2kml_google_earth.jpg
   :width: 100%

Example fly-by video, generated with the ``--tour-duration`` option:

.. raw:: html

   <div style="width: 100%; overflow: hidden;">
     <video controls muted style="width: 100%;">
       <source src="https://mrpt.github.io/videos/mm2kml_flyby_husky_ual.mp4" type="video/mp4">
     </video>
   </div>

The KMZ file contains:

- **Point cloud, 2D overlay**: a top-down image of the point cloud (``GroundOverlay``) draped on
  the terrain. It is light and always aligned with the imagery.
- **Point cloud, 3D model**: a COLLADA model with one small solid per point, after voxel
  decimation to keep Google Earth responsive. Points are colored by their height above the
  local ground.
- **Trajectory** (optional): a line clamped to the ground.
- **Fly-by tour** (optional): a ``gx:Tour`` orbiting around the map.

Points close to the ground (``--min-height-above-ground``) are removed by default, so the
satellite imagery remains visible below walls, trees, poles, etc.

Usage
-----

.. code-block:: bash

    Usage: mm2kml [OPTIONS] input

    Positionals:
      input TEXT REQUIRED         Georeferenced metric map file (*.mm)

    Options:
      -h,--help                   Print this help message and exit
      -o,--output TEXT REQUIRED   Output Google Earth file (*.kmz)
      -l,--layer TEXT ...         Point cloud layer to export. If not provided, all point cloud layers are merged. This argument can appear several times.
      -t,--trajectory TEXT        Optional trajectory file in TUM format, drawn as a line clamped to the ground. See --trajectory-frame.
      --trajectory-frame TEXT:{map,enu} [map]
                                  Frame of the --trajectory poses: 'map' (the local frame of the input map) or 'enu' (East-North-Up, e.g. a trajectory already converted with the map georeferencing).
      --cloud TEXT:{both,overlay,model,none} [both]
                                  How to export the point cloud: 'overlay' (top-down image draped on the terrain), 'model' (3D COLLADA model, one small solid per decimated point), 'both', or 'none'.
      --min-height-above-ground FLOAT [0.35]
                                  Points lower than this height [m] above the local ground are not exported, so the satellite imagery remains visible. Use 0 to keep all points.
      --ground-cell FLOAT [1]     Cell size [m] of the grid used to estimate the local ground height.
      --color-max-height FLOAT    Height above ground [m] mapped to the top of the color scale. Default (0): the 98% percentile of point heights.
      --overlay-resolution FLOAT  Ground overlay resolution [m/pixel]. Default (0): automatic, the finest one (>=5 cm) keeping the image under 2048 pixels.
      --model-max-points UINT [100000]
                                  Maximum number of points in the 3D model. The cloud is voxel-decimated to fit.
      --model-point-size FLOAT    Size [m] of each point in the 3D model. Default (0): from the decimation voxel size.
      --model-altitude-offset FLOAT [0]
                                  Extra altitude [m] for the 3D model, in case it does not sit on the terrain.
      --heading FLOAT [0]         Heading [deg] of the initial view.
      --tour-duration FLOAT       If >0, adds a 'Fly-by' tour [s] orbiting around the map. In Google Earth Pro, it can be recorded as a video with Tools > Movie Maker.
      --tour-tilt FLOAT [60]      Camera tilt [deg] for the fly-by tour.
      --load-plugins TEXT         One or more (comma separated) *.so files to load as plugins

Examples
--------

Export a map built with MOLA, including its trajectory and a 40 s fly-by tour:

.. code-block:: bash

    mm2kml map.mm -t trajectory.tum -o map.kmz --tour-duration 40

If the trajectory file is already in East-North-Up coordinates:

.. code-block:: bash

    mm2kml map.mm -t trajectory_enu.tum --trajectory-frame enu -o map.kmz

Export only the light 2D overlay, e.g. for large maps:

.. code-block:: bash

    mm2kml map.mm --cloud overlay -o map.kmz

Maps using custom map classes (e.g. from ``mola_metric_maps``) need the plugin:

.. code-block:: bash

    mm2kml map.mm -o map.kmz --load-plugins libmola_metric_maps.so

Notes
-----

- The input map must contain georeferencing information (see :ref:`mm-georef <app_mm-georef>`).
- Open the KMZ in Google Earth Pro (desktop) or Google Earth Web. Google Earth's own
  "3D Buildings" layer (which also includes 3D trees) may hide the 2D overlay and parts of
  the 3D model. If it is disabled, keep in mind that the satellite imagery is flat: in tilted
  views, elevated points (e.g. tree canopies) appear shifted with respect to the objects in the
  imagery because of parallax. Enable "3D Buildings" to compare heights against Google 3D
  geometry.
- The 3D model is placed so the ground below the map origin lies on Google Earth terrain.
  Use ``--model-altitude-offset`` to correct it if the model floats or sinks.
- Google Earth does not accept ground overlay images larger than 2048 pixels, hence the
  automatic resolution. For large maps, the overlay becomes coarser: crop the map first
  with :ref:`mm-filter <app_mm-filter>` if more detail is needed.
- To make a video of the fly-by: in Google Earth Pro, double-click the "Fly-by" tour in the
  Places panel to play it, or record it with *Tools > Movie Maker*.

.. _app_mm2grid:

===============================
Application: ``mm2grid``
===============================

Overview
--------

``mm2grid`` is a command-line tool that exports a ``COccupancyGridMap2D`` layer
from a MOLA metric map (\\*.mm) file as a pair of files compatible with the ROS
`map_server <http://wiki.ros.org/map_server>`_ and
`nav2_map_server <https://docs.nav2.org>`_ packages:

- A **PNG image** encoding the occupancy probabilities as gray levels.
- A **YAML metadata file** describing the map resolution, origin, and
  interpretation thresholds.

The output can be loaded directly by ``ros2 run nav2_map_server map_server``
or the ROS 1 equivalent without any further post-processing.

Features
--------

- Exports any ``COccupancyGridMap2D`` layer from a ``.mm`` metric map file
- Produces a lossless PNG image (grayscale, 1 pixel = 1 cell)
- Generates a fully compatible ROS ``map_server`` / ``nav2_map_server`` YAML file
- Automatically detects the grid layer when only one is present
- Configurable occupancy/free thresholds, negate flag, and interpretation mode

Usage
-----

.. code-block:: bash

   mm2grid <input.mm> [-l <layer_name>] [-o <output_base>]
           [--occupied-thresh <value>] [--free-thresh <value>]
           [--mode trinary|scale|raw] [--negate]

Arguments
^^^^^^^^^

- ``input`` (required): Input metric map file (\\*.mm).
- ``-l, --layer <name>`` (optional): Name of the layer to export. The layer
  must be of type ``COccupancyGridMap2D``. If omitted and the map contains
  exactly one occupancy-grid layer, that layer is selected automatically.
  An error is raised if multiple occupancy-grid layers are present and no
  ``--layer`` is specified.
- ``-o, --output <base>`` (optional): Base name (without extension) for the
  output files. Two files will be written: ``<base>.png`` and ``<base>.yaml``.
  Defaults to ``<input_basename>_<layerName>``.
- ``--occupied-thresh <value>`` (optional): Cells with occupancy probability
  above this value are treated as occupied (black in the image). Default: ``0.65``.
- ``--free-thresh <value>`` (optional): Cells with occupancy probability below
  this value are treated as free (white in the image). Default: ``0.196``.
- ``--mode <trinary|scale|raw>`` (optional): Interpretation mode written into
  the YAML file, used by ``nav2_map_server``. Default: ``trinary``.
  See the `nav2_map_server documentation
  <https://docs.nav2.org/configuration/packages/configuring-map-server.html>`_
  for a description of each mode.
- ``--negate`` (optional): If set, the color interpretation is inverted:
  black pixels are free and white pixels are occupied. Sets ``negate: 1`` in
  the YAML. Default: not set (``negate: 0``).

Examples
^^^^^^^^

Export the only occupancy-grid layer from a map, using default settings:

.. code-block:: bash

   mm2grid mymap.mm

Export a specific layer:

.. code-block:: bash

   mm2grid mymap.mm --layer gridmap

Specify a custom output base name:

.. code-block:: bash

   mm2grid mymap.mm --layer gridmap --output /tmp/my_floor_plan

Use custom thresholds suitable for a high-confidence map:

.. code-block:: bash

   mm2grid mymap.mm --occupied-thresh 0.75 --free-thresh 0.25

Export in ``scale`` mode for nav2_map_server:

.. code-block:: bash

   mm2grid mymap.mm --mode scale

Output
------

Two files are created, named after the output base (default:
``<input_basename>_<layerName>``):

.. code-block:: text

   <output_base>.png
   <output_base>.yaml

For example, running ``mm2grid building.mm --layer floor1`` produces:

- ``building_floor1.png``
- ``building_floor1.yaml``

Image Format
------------

The PNG image is an 8-bit grayscale file where each pixel corresponds to one
grid cell, following the ROS ``map_server`` convention:

- **White (255)**: free space (occupancy probability close to 0)
- **Black (0)**: occupied space (occupancy probability close to 1)
- **Gray (~205)**: unknown / unobserved cells (occupancy probability ~0.5)

The image row order is set so that row 0 corresponds to the lowest Y
coordinate in the map, matching the ``origin`` (bottom-left corner) convention
expected by ``map_server``.

YAML Format
-----------

The generated YAML file follows the
`map_server YAML format <http://wiki.ros.org/map_server#YAML_format>`_:

.. code-block:: yaml

   image: <output_base>.png
   resolution: 0.050000
   origin: [-10.000000, -10.000000, 0.000000]
   negate: 0
   occupied_thresh: 0.65
   free_thresh: 0.196
   mode: trinary

Where:

- ``image``: path to the PNG file (just the filename, so the pair is
  relocatable).
- ``resolution``: cell size in metres, taken directly from the grid map.
- ``origin``: real-world pose ``[x, y, yaw]`` of the **bottom-left** pixel,
  i.e. ``[xMin, yMin, 0.0]`` from the grid map's coordinate bounds.
- ``negate``, ``occupied_thresh``, ``free_thresh``: controlled by the
  corresponding CLI flags (see above).
- ``mode``: written only when ``nav2_map_server`` semantics are needed;
  ignored by ROS 1 ``map_server``.

Loading the Result in ROS 2
---------------------------

The output files can be used directly as a static map in a Nav2 launch
configuration:

.. code-block:: yaml

   # map_server params
   map_server:
     ros__parameters:
       yaml_filename: /path/to/building_floor1.yaml

Or loaded on the command line:

.. code-block:: bash

   ros2 run nav2_map_server map_server --ros-args \
       -p yaml_filename:=/path/to/building_floor1.yaml

Loading the Result in ROS 1
----------------------------

.. code-block:: bash

   rosrun map_server map_server /path/to/building_floor1.yaml

See Also
--------

- :ref:`app_mm-info` - display a summary of any metric map file
- :ref:`app_mm2txt` - export point cloud layers from a metric map as CSV/TXT

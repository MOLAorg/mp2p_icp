.. _app_mm2ply:

===============================
Application: ``mm2ply``
===============================

Overview
--------

``mm2ply`` is a command-line tool that exports the layers of a MOLA metric map (\*.mm) file as PLY point cloud files. It converts metric map files into the widely-supported PLY format, making it easy to visualize and process point cloud data in various 3D viewers and processing tools.

Features
--------

- Exports all point cloud layers from a metric map file
- Supports both ASCII and binary PLY formats
- Preserves all point fields (coordinates, colors, intensities, etc.)
- Automatic field name mapping for standard color attributes
- Generic field handling for custom point cloud attributes

Usage
-----

.. code-block:: bash

   mm2ply -i <input.mm> [-o <output_prefix>] [-b]

Arguments
^^^^^^^^^

- ``-i, --input <file.mm>`` (required): Input metric map file
- ``-o, --output <prefix>`` (optional): Prefix for output PLY files. If not specified, uses the input filename without extension
- ``-b, --binary`` (optional): Export in binary format instead of ASCII (default: ASCII)

Examples
^^^^^^^^

Export a metric map to ASCII PLY files:

.. code-block:: bash

   mm2ply -i mymap.mm

Export with a custom output prefix:

.. code-block:: bash

   mm2ply -i mymap.mm -o processed/map

Export in binary format for smaller file sizes:

.. code-block:: bash

   mm2ply -i mymap.mm -b

Output
------

The tool creates separate PLY files for each point cloud layer in the metric map. Files are named using the pattern:

.. code-block:: text

   <prefix>_<layer_name>.ply

For example, if your map contains layers named "raw" and "filtered", you'll get:

- ``mymap_raw.ply``
- ``mymap_filtered.ply``

Supported Point Fields
----------------------

The tool automatically exports all point attributes, including:

- **Coordinates**: x, y, z (float)
- **Colors**: Automatically maps ``color_r/color_rf`` → ``red``, ``color_g/color_gf`` → ``green``, ``color_b/color_bf`` → ``blue``
- **Custom float fields**: Exported as float properties
- **Custom double fields**: Exported as double properties
- **Custom uint16 fields**: Exported as ushort properties
- **Custom uint8 fields**: Exported as uchar properties

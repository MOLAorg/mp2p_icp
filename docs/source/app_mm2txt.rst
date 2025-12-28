.. _app_mm2txt:

===============================
Application: ``mm2txt``
===============================

Overview
--------

``mm2txt`` is a command-line tool that exports the layers of a MOLA metric map (\*.mm) file as CSV/TXT files. It converts point cloud layers from metric map files into human-readable text format, making it easy to process and analyze point cloud data in spreadsheet applications, scripting languages, or custom data processing pipelines.

Features
--------

- Exports all or selected point cloud layers from a metric map file
- Automatically detects and exports all point fields (coordinates, intensities, colors, timestamps, etc.)
- Generates header row with field names for easy data parsing
- Supports multiple point cloud types (CGenericPointsMap, CPointsMapXYZI, CPointsMapXYZIRT)
- Space-delimited format compatible with most data analysis tools
- Selective layer export for processing only the data you need

Usage
-----

.. code-block:: bash

   mm2txt <input.mm> [-l <layer_name>] [-l <layer_name>] ...

Arguments
^^^^^^^^^

- ``input`` (required): Input metric map file (\*.mm)
- ``-l, --layer <name>`` (optional): Layer to export. If not provided, all layers will be exported. This argument can appear multiple times to export specific layers

Examples
^^^^^^^^

Export all layers from a metric map:

.. code-block:: bash

   mm2txt mymap.mm

Export only specific layers:

.. code-block:: bash

   mm2txt mymap.mm -l raw -l filtered

Export a single layer:

.. code-block:: bash

   mm2txt mymap.mm -l global

Output
------

The tool creates separate TXT files for each exported point cloud layer. Files are named using the pattern:

.. code-block:: text

   <input_filename>_<layer_name>.txt

For example, if your map file is ``mymap.mm`` and contains layers named "raw" and "filtered", you'll get:

- ``mymap.mm_raw.txt``
- ``mymap.mm_filtered.txt``

File Format
-----------

Each output file is a space-delimited text file with:

- **Header row**: Column names for all point fields
- **Data rows**: One row per point, with values for each field

Example output:

.. code-block:: text

   x y z intensity ring time
   1.234 5.678 0.123 45.6 0 12345.678
   2.345 6.789 0.234 67.8 1 12345.679
   ...

Supported Point Fields
----------------------

The tool automatically exports all available point attributes:

- **Coordinates**: x, y, z (mandatory, float)
- **Float fields**: intensity, normals, curvature, custom float attributes
- **Uint16 fields**: ring, color channels, custom uint16 attributes
- **Double fields**: time, timestamps, custom double attributes (MRPT ≥ 2.15.3)

The specific fields exported depend on the point cloud type in each layer.

Supported Point Cloud Types
---------------------------

- **CGenericPointsMap**: Generic point clouds with custom fields
- **CPointsMapXYZI** (Deprecated): Point clouds with intensity values
- **CPointsMapXYZIRT** (Deprecated): Point clouds with intensity, ring, and time information

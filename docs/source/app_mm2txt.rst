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
- Selectively exports specific fields in a custom order
- Automatically detects and exports all point fields (coordinates, intensities, colors, timestamps, etc.)
- Generates header row with field names for easy data parsing
- Supports multiple point cloud types (CGenericPointsMap, CPointsMapXYZI, CPointsMapXYZIRT)
- Space-delimited format compatible with most data analysis tools
- Selective layer export for processing only the data you need

Usage
-----

.. code-block:: bash

   mm2txt <input.mm> [-l <layer_name>] [--export-fields <field1,field2,...>] [--ignore-missing-fields]

Arguments
^^^^^^^^^

- ``input`` (required): Input metric map file (\*.mm)
- ``-l, --layer <name>`` (optional): Layer to export. If not provided, all layers will be exported. This argument can appear multiple times to export specific layers
- ``--export-fields <field1,field2,...>`` (optional): Comma-separated list of fields to export in the specified order. If not provided, all available fields will be exported. Spaces around commas are allowed
- ``--ignore-missing-fields`` (optional): If defined, the lack of any of the ``--export-fields`` in the map will be considered a warning instead of an error.

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

Export only specific fields in a custom order:

.. code-block:: bash

   mm2txt mymap.mm --export-fields "x,y,z,intensity"

Export selected fields from specific layers:

.. code-block:: bash

   mm2txt mymap.mm -l raw --export-fields "x, y, z, ring, time"

Export only 2D coordinates and intensity:

.. code-block:: bash

   mm2txt mymap.mm --export-fields "x,y,intensity"

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

Example output (all fields):

.. code-block:: text

   x y z intensity ring time
   1.234 5.678 0.123 45.6 0 12345.678
   2.345 6.789 0.234 67.8 1 12345.679
   ...

Example output (selected fields):

.. code-block:: text

   x y z intensity
   1.234 5.678 0.123 45.6
   2.345 6.789 0.234 67.8
   ...

Field Selection
---------------

When using the ``--export-fields`` option:

- Fields must be specified as a comma-separated list
- Spaces around commas are allowed (e.g., ``"x, y, z"`` or ``"x,y,z"``)
- Fields will be exported in the exact order specified
- All specified fields must exist in the point cloud, or an error will be raised (unless ``--ignore-missing-fields`` is added)
- The tool validates field availability and reports available fields if a requested field is not found
- This option is only supported for **CGenericPointsMap** point clouds; for legacy types (CPointsMapXYZI, CPointsMapXYZIRT), all fields are exported with a warning

Supported Point Fields
----------------------

The tool automatically exports all available point attributes:

- **Coordinates**: x, y, z (mandatory, float)
- **Float fields**: intensity, normals, curvature, custom float attributes
- **Uint16 fields**: ring, color channels, custom uint16 attributes
- **Double fields**: time, timestamps, custom double attributes (MRPT ≥ 2.15.3)
- **Uint8 fields**: custom uint8 attributes (MRPT ≥ 2.15.3)

The specific fields exported depend on the point cloud type in each layer. Use ``--export-fields`` to select only the fields you need.

Supported Point Cloud Types
---------------------------

- **CGenericPointsMap**: Generic point clouds with custom fields (full support for ``--export-fields``)
- **CPointsMapXYZI** (Deprecated): Point clouds with intensity values (``--export-fields`` not supported)
- **CPointsMapXYZIRT** (Deprecated): Point clouds with intensity, ring, and time information (``--export-fields`` not supported)
.. _app_txt2mm:

===============================
Application: ``txt2mm``
===============================

``txt2mm`` is a command-line utility provided by the **MOLA** and **mp2p_icp** frameworks
for converting plain-text point cloud data (TXT/CSV) into a MOLA-compatible **metric map** (``.mm``) file.

Usage
-----

.. code-block:: console

   txt2mm [--label <label>] [--id <ID>] [--column-t <index>] [--column-r <index>]
          [--column-i <index>] [--column-x <index>] [-l <layer>] 
          -f <format> -o <out.mm> -i <input.txt> [--version] [-h]

Required Arguments
------------------

``-i <input.txt>``, ``--input <input.txt>``
   Path to the input file (TXT or CSV).
   Each row must represent one 3D point. Columns can be separated by spaces or commas.
   Column mapping depends on the selected format.

``-o <out.mm>``, ``--output <out.mm>``
   Output file in MOLA **metric map** (``.mm``) format.

``-f <format>``, ``--format <format>``
   Point cloud format. **Mandatory flag**.

   Supported values:

   - ``xyz`` : Only XYZ coordinates.
   - ``xyzi`` : XYZ + intensity.
   - ``xyzirt`` : XYZ + intensity + ring + timestamp.
   - ``xyzrgb`` : XYZ + RGB (0–255 scale).
   - ``xyzrgb_normalized`` : XYZ + RGB (normalized [0–1] values).

Optional Arguments
------------------

``--label <label>``
   Metric map label string. Default: none.

``--id <ID>``
   Metric map numeric identifier. Default: none.

``-l <layer>``, ``--layer <layer>``
   Target layer name within the metric map. Default: ``raw``.

``--column-x <index>``
   Column index for the **X** coordinate. Default: 0.

``--column-i <index>``
   Column index for the **intensity** channel. Default: 3.

``--column-r <index>``
   Column index for the **ring** channel. Default: 4.

``--column-t <index>``
   Column index for the **timestamp** channel. Default: 5.

Utility Flags
-------------

``--version``
   Show version information and exit.

``-h``, ``--help``
   Show usage information and exit.

``--``, ``--ignore_rest``
   Ignore the rest of the arguments following this flag.

Examples
--------

Convert a plain text file with ``XYZI`` data into a metric map:

.. code-block:: console

   txt2mm -i scan_xyzi.txt -o scan.mm -f xyzi

Convert a CSV file with ``XYZRGB`` data, mapping RGB values normalized between [0–1]:

.. code-block:: console

   txt2mm -i scan_xyzrgb.csv -o scan.mm -f xyzrgb_normalized --label "lidar_frame_01"

Use a different target layer name and assign a numeric ID:

.. code-block:: console

   txt2mm -i lidar_points.txt -o map_layer.mm -f xyzirt -l "velodyne" --id 42


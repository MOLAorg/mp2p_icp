.. _app_mm2las:

===============================
Application: ``mm2las``
===============================

Overview
--------

``mm2las`` is a command-line tool that exports the layers of a MOLA metric map (``*.mm``) file as LAS point cloud files. 
It converts metric map files into the industry-standard **LAS format version 1.4**, enabling seamless integration with modern GIS applications, 
LIDAR processing suites, and 3D viewers like CloudCompare, QGIS, or ArcGIS.

Features
--------

- **LAS 1.4 Compliance**: Exports data using the modern LAS 1.4 specification.
- **Point Format 8**: Automatically utilizes Point Data Record Format 8 (38 bytes base) to support XYZ, Intensity, RGB, and NIR data in a single record.
- **Extra Dimensions**: Automatically exports non-standard metric map fields as "Extra Bytes" Variable Length Records (VLRs), allowing no data loss for custom attributes.
- **Extended Precision**: Supports high-precision coordinate encoding (1mm scale factor) and 64-bit point counters for massive datasets.
- **Advanced LIDAR Support**: Now includes native support for GPS Time, NIR (Near Infrared), Scan Angle, and Point Source IDs.
- **Automatic Color Mapping**: Converts various MRPT color formats (8-bit, 16-bit, or float) into the 16-bit LAS RGB standard.

Usage
-----

.. code-block:: bash

   mm2las -i <input.mm> [-o <output_prefix>] [--export-fields <field1,field2,...>]

Arguments
^^^^^^^^^

- ``-i, --input <file.mm>`` (required): Input metric map file.
- ``-o, --output <prefix>`` (optional): Prefix for output LAS files. If not specified, uses the input filename.
- ``--export-fields <field1,field2,...>`` (optional): Comma-separated list of fields to export. If omitted, all available fields are exported, with non-standard fields becoming Extra Dimensions.
- ``--system-id <string>``: Sets the System Identifier in the LAS header (default: "mm2las").
- ``--generating-software <string>``: Sets the Generating Software in the LAS header (default: "MOLA mm2las").


Examples
^^^^^^^^

Export a metric map to LAS files with default fields:

.. code-block:: bash

   mm2las -i mymap.mm

Export with a custom output prefix:

.. code-block:: bash

   mm2las -i mymap.mm -o processed/map

Export only coordinates and intensity:

.. code-block:: bash

   mm2las -i mymap.mm --export-fields "x,y,z,intensity"

Export with RGB colors:

.. code-block:: bash

   mm2las -i mymap.mm --export-fields "x,y,z,red,green,blue"


Field Selection
---------------

When using the ``--export-fields`` option:

- **Standard Mapping**: Fields like ``intensity``, ``red``, ``green``, ``blue``, ``nir``, and ``gps_time`` are mapped to standard LAS 1.4 slots.
- **Color Aliases**: Recognizes MRPT-specific names like ``color_r``, ``color_g``, ``color_b`` (8-bit) or ``color_rf``, ``color_gf``, ``color_bf`` (float) and converts them to 16-bit LAS RGB.
- **Extra Dimensions**: Any field name not recognized as a standard LAS field (e.g., ``reflectivity``, ``ambient``, ``label``) is automatically appended to the end of each point record as an Extra Byte dimension.

Supported LAS Fields
--------------------

Mandatory Fields
^^^^^^^^^^^^^^^^

- **x, y, z**: Encoded as 32-bit scaled integers with 1mm precision and minimum-value offsets.

Optional Standard Fields
^^^^^^^^^^^^^^^^^^^^^^^^

- **intensity**: 16-bit unsigned integer (0-65535).
- **RGB & NIR**: 16-bit unsigned integers. RGB is required for Point Format 8.
- **gps_time**: Double-precision floating point.
- **scan_angle**: Scaled 16-bit integer.
- **classification/user_data**: 8-bit unsigned integers.
- **return_number/number_of_returns**: Bit-packed return information.

LAS Format Details
------------------

- **LAS Version**: 1.4.
- **Point Format**: 8 (Base size: 38 bytes + Extra Dimensions).
- **WKT Support**: Uses LAS 1.4 Global Encoding to signal coordinate reference system compatibility.
- **Coordinate precision**: 0.001 (1mm scale).
- **Maximum points**: Supports >4.3 billion points via 64-bit Extended Point Counters.

Compression
-----------

The tool outputs uncompressed LAS files. To create compressed LAZ files:

Using LASzip:

.. code-block:: bash

   mm2las -i mymap.mm
   laszip mymap_layer.las  # Creates mymap_layer.laz

Using PDAL:

.. code-block:: bash

   mm2las -i mymap.mm
   pdal translate mymap_layer.las mymap_layer.laz

LAZ files typically achieve 7-20× compression ratios while maintaining lossless data.

Limitations
-----------

- **Waveform Data**: Not supported in the current export logic.
- **Direct LAZ**: Compressed LAZ files are not written directly; use ``laszip`` or ``pdal`` for post-compression.
- **Coordinate Systems**: While the file is LAS 1.4 compliant, specific CRS WKT strings are not yet automatically embedded in the VLRs.



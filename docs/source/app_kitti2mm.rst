.. _app_kitti2mm:

===============================
Application: ``kitti2mm``
===============================

This CLI tool converts KITTI dataset LIDAR binary files (``.bin``) with (X,Y,Z, Intensity) data into mp2p_icp metric map files (``.mm``).

The KITTI dataset stores point cloud data in a binary format. 
This tool reads those files and converts them into the mp2p_icp metric map format, optionally setting the layer name, numeric ID, and label string for the resulting map.


.. code-block:: bash

    USAGE: 

    kitti2mm  [-l <raw>] [--label <label>] [--id <ID>] -o <out.mm> -i
              <kitti-00.bin> [--] [--version] [-h]


    Where: 

    -l <raw>,  --layer <raw>
        Target layer name (Default: "raw").

    --label <label>
        Metric map label string (Default: none).

    --id <ID>
        Metric map numeric ID (Default: none).

    -o <out.mm>,  --output <out.mm>
        (required)  Output file to write to.

    -i <kitti-00.bin>,  --input <kitti-00.bin>
        (required)  KITTI .bin pointcloud file.

    --,  --ignore_rest
        Ignores the rest of the labeled arguments following this flag.

    --version
        Displays version information and exits.

    -h,  --help
        Displays usage information and exits.


Example usage
=============

Convert a KITTI binary point cloud file to a metric map:

.. code-block:: bash

    kitti2mm -i /path/to/kitti/velodyne/000000.bin -o output.mm

Convert with a custom layer name and label:

.. code-block:: bash

    kitti2mm -i scan.bin -o scan.mm -l lidar_points --label "KITTI Sequence 00 Frame 42" --id 42

.. _app_sm-cli:

===============================
Application: ``sm-cli``
===============================

CLI tool to manipulate and inspect simple-maps:

Available commands:

.. code-block:: bash

    sm-cli cut                Cut part of a .simplemap file into a new file.
    sm-cli export-keyframes   Export KF poses (opt: twist too) as TUM format.
    sm-cli export-rawlog      Export KFs as rawlog for inspection.
    sm-cli info               Analyze a .simplemap file.
    sm-cli join               Join two or more .simplemap files into one.
    sm-cli level              Levels a .simplemap from its keyframe heights.
    sm-cli level-walls        Levels a .simplemap from its wall/floor normals.
    sm-cli tf                 Applies a SE(3) transform by the left to a map.
    sm-cli trim               Extracts part of a .simplemap inside a given box.
    sm-cli --version          Shows program version.
    sm-cli --help             Shows this information.

    Or use `sm-cli <COMMAND> --help` for further options

|

sm-cli cut
---------------
Cuts a simple-map by keyframe indices, saving the smaller simple-map to a new file:

.. code-block:: bash

    sm-cli cut --help
    Usage:

        sm-cli cut <filename> --from <FIRST_KF_INDEX> --to <LAST_KF_INDEX> --output <OUTPUT.simplemap>


|


sm-cli export-keyframes
-------------------------
Saves the key-frames in a simple-map as a trajectory file in TUM format:

Refer to the tutorial for example data file and command line: :ref:`building-maps_sect_inspect_sm`.

.. code-block:: bash

    sm-cli export-keyframes <filename> --output <OUTPUT.tum> [--output-twist <TWIST.txt>]


|


sm-cli export-rawlog
----------------------
Export all keyframes in the simplemap, including pose and twist information, metadata (see MOLA-LO paper),
and raw sensor observations (3D LiDAR scans, GNSS data, etc.) to a RawLog file, which can be easily
browsed with RawLogViewer.

Refer to the tutorial for example data file and command line: :ref:`building-maps_sect_inspect_sm`.


.. code-block:: bash

    sm-cli export-rawlog <filename.simplemap> --output <OUTPUT.rawlog>

|

sm-cli info
----------------------
Shows basic information about the contents of a simple map.

Refer to the tutorial for example data file and command line: :ref:`building-maps_sect_inspect_sm`.

.. code-block:: bash

    sm-cli info <filename.simplemap>

|

sm-cli level
----------------------
Takes an input simple-map and optimizes its key-frame poses such as they lie on an horizontal plane as much as possible,
saving the result in another simple-map file. This can be used when a map has an unintentional tilt for some reason, for example, wrong or missing sensor extrinsics.

.. code-block:: bash

    sm-cli level <input.simplemap> <output.simplemap>

This assumes a vehicle moving on flat ground. For handheld or aerial maps of buildings, whose trajectory
changes height on purpose, use ``sm-cli level-walls`` instead.

|

sm-cli level-walls
----------------------
Corrects a global pitch/roll error of a simple-map by rotating it about the map origin, so that walls become
as vertical, and floors and ceilings as horizontal, as possible. A typical source of such error is the initial
attitude of a LiDAR-inertial run, estimated from a short accelerometer average.

.. code-block:: bash

    sm-cli level-walls <input.simplemap> <output.simplemap>
        [--pipeline <sm2mm-pipeline.yaml>]  # keyframes to points (default: built-in)
        [--voxel 0.05]                      # analysis cloud voxel size [m]
        [--normal-radius 0.4]               # neighborhood for local normals [m]
        [--wall-nz 0.25] [--flat-nz 0.95]   # |n.up| thresholds for walls / floors
        [--max-correction-deg 5]            # refuse larger corrections
        [--estimate-only]                   # print the rotation, do not write output
        [--windows N]                       # also estimate on N equal time windows

How it works:

- Keyframes are converted into a voxel-decimated point cloud with an sm2mm pipeline (by default, the default
  generator plus a per-keyframe voxel filter). With ``--pipeline``, all point layers it outputs are merged.
- A normal is estimated for each point with a planar neighborhood.
- Starting from :math:`u=+Z`, normals are classified as walls (:math:`|n \cdot u|` < ``wall-nz``) or floors and
  ceilings (:math:`|n \cdot u|` > ``flat-nz``), and :math:`u` is updated to the eigenvector of the smallest
  eigenvalue of :math:`M = \frac{1}{|W|}\sum_W n n^T + \frac{1}{|F|}\sum_F (I - n n^T)`, a few times.
- The map is rotated by the minimal rotation that takes :math:`u` to :math:`+Z` (so the heading is kept).
  Only keyframe poses change, exactly as with ``sm-cli tf``, whose equivalent command is printed.

The command aborts if :math:`u` is not observable (e.g. one wall direction and no floors), or if the correction
exceeds ``--max-correction-deg``. ``--windows N`` repeats the estimate on N time windows: similar values mean a
fixed gauge error that a single rotation removes, while differing values mean drift, which it can not fix.

.. note::

   It assumes a mostly rectilinear structure with vertical walls and level floors (indoors, buildings).
   Do not use it on outdoor or sloped scenes.

|

sm-cli tf
----------------------
Transforms a given simple-map by applying a SE(3) transformation by the left (=left-multiplying homogeneous matrices).

.. code-block:: bash

    sm-cli tf <input.simplemap> <output.simplemap> "[x y z yaw_deg pitch_deg roll_deg]"

|

sm-cli trim
----------------------
Extracts part of a simple-map, leaving only those key-frames that lie within a given bounding box.

.. code-block:: bash

    sm-cli trim <filename> --min-corner "[xmin ymin zmin]" --max-corner "[xmax ymax zmax]" --output <OUTPUT.simplemap>


|

sm-cli join
----------------------
Merges two or more simple-maps in one single map. No map alignment or registration is performed by this simple tool,
so the maps should be already aligned beforehand, or the resulting simple-map being the input to a loop-closure pipeline.

.. code-block:: bash

    sm-cli join <filename_1> [<filename_2> ...] --output <MERGED.simplemap>

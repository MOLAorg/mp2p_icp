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
    sm-cli level              Makes a .simplemap file level (horizontal).
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

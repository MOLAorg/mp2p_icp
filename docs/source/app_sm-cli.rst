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
    sm-cli level              Makes a .simplemap file level (horizontal).
    sm-cli trim               Extracts part of a .simplemap inside a given box.
    sm-cli join               Join two or more .simplemap files into one.
    sm-cli --version          Shows program version.
    sm-cli --help             Shows this information.

    Or use `sm-cli <COMMAND> --help` for further options


sm-cli cut
---------------

Cuts a simple-map by keyframe indices, saving the smaller simple-map to a new file:

.. code-block:: bash

    sm-cli cut --help
    Usage:

        sm-cli cut <filename> --from <FIRST_KF_INDEX> --to <LAST_KF_INDEX> --output <OUTPUT.simplemap>


sm-cli export-keyframes
-------------------------

Saves the key-frames in a simple-map as a trajectory file in TUM format:

.. code-block:: bash

    sm-cli export-keyframes <filename> --output <OUTPUT.tum> [--output-twist <TWIST.txt>]

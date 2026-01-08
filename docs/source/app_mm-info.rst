.. _app_mm-info:

===============================
Application: ``mm-info``
===============================

This CLI tool reads a metric map file (``.mm``) and displays a summary of its contents.

The tool provides information about the layers, point counts, and other metadata contained in the metric map file.


.. code-block:: bash

    USAGE: 

    mm-info  <myMap.mm> [--] [--version] [-h]


    Where: 

    <myMap.mm>
        (required)  Load this metric map file (*.mm)

    --,  --ignore_rest
        Ignores the rest of the labeled arguments following this flag.

    --version
        Displays version information and exits.

    -h,  --help
        Displays usage information and exits.


Example usage
=============

Display information about a metric map:

.. code-block:: bash

    mm-info myMap.mm

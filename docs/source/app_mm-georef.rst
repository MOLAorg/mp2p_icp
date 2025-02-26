.. _app_mm-georef:

===============================
Application: ``mm-georef``
===============================

This cli tool allows: 

- Extracting geo-referencing information from a metric map file (``*.mm``) into an independent binary file (``*.georef``), and
- The inverse operation, injecting geo-referencing information from a binary file (``*.georef``) into a metric map file (``*.mm``).


.. code-block:: bash

    USAGE: 

    mm-georef  [-l <foobar.so>] [--inject-to-map] [--extract-from-map] -g
                <myMap.georef> -m <theMap.mm> [--] [--version] [-h]


    Where: 

    -l <foobar.so>,  --load-plugins <foobar.so>
        One or more (comma separated) *.so files to load as plugins

    --inject-to-map
        Reads the geo-referencing data from an input file and stores it into
        the existing map file.file

    --extract-from-map
        Reads the geo-referencing data from the map and saves it to a .georef
        file

    -g <myMap.georef>,  --georef <myMap.georef>
        (required)  Input/Output binary `.georef` file with geo-referencing
        metadata

    -m <theMap.mm>,  --mao <theMap.mm>
        (required)  Input/Output .mm file to operate on

    --,  --ignore_rest
        Ignores the rest of the labeled arguments following this flag.

    --version
        Displays version information and exits.

    -h,  --help
        Displays usage information and exits.


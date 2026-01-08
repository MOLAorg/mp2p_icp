.. _app_mm-filter:

===============================
Application: ``mm-filter``
===============================

This CLI tool applies :ref:`sm2mm pipelines <sm2mm_pipelines>` to metric map files (``.mm``).

The tool can operate in two modes:

- **Pipeline mode**: Applies a complete mp2p_icp_filters pipeline defined in a YAML configuration file to transform and filter the point cloud data in the metric map.
- **Rename mode**: Simply renames a layer within the metric map from one name to another.


.. code-block:: bash

    USAGE: 

    mm-filter  [-v <INFO>] [--rename-layer <"NAME|NEW_NAME">] [-p
               <pipeline.yaml>] [-l <foobar.so>] -o <out.mm> -i <input.mm>
               [--] [--version] [-h]


    Where: 

    -v <INFO>,  --verbosity <INFO>
        Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)

    --rename-layer <"NAME|NEW_NAME">
        Alternative operation: instead of applying a pipeline, just renames a
        layer from NAME to NEW_NAME.

    -p <pipeline.yaml>,  --pipeline <pipeline.yaml>
        YAML file with the mp2p_icp_filters pipeline to load. It must contain
        a `filters:` section. See the app README for examples:
        https://github.com/MOLAorg/mp2p_icp/tree/develop/apps/mm-filter

    -l <foobar.so>,  --load-plugins <foobar.so>
        One or more (comma separated) *.so files to load as plugins

    -o <out.mm>,  --output <out.mm>
        (required)  Output .mm file to write to

    -i <input.mm>,  --input <input.mm>
        (required)  Input .mm file

    --,  --ignore_rest
        Ignores the rest of the labeled arguments following this flag.

    --version
        Displays version information and exits.

    -h,  --help
        Displays usage information and exits.


Example usage
=============

Apply a filter pipeline to a metric map:

.. code-block:: bash

    mm-filter -i input.mm -o filtered.mm -p my_pipeline.yaml

Apply a filter pipeline with custom verbosity:

.. code-block:: bash

    mm-filter -i input.mm -o filtered.mm -p my_pipeline.yaml -v DEBUG

Rename a layer within a metric map:

.. code-block:: bash

    mm-filter -i input.mm -o output.mm --rename-layer "raw|processed"

Load custom plugins and apply a pipeline:

.. code-block:: bash

    mm-filter -i input.mm -o output.mm -p pipeline.yaml -l my_custom_filters.so
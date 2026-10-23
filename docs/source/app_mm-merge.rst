.. _app_mm-merge:

===============================
Application: ``mm-merge``
===============================

This CLI tool merges two or more metric map files (``*.mm``) into a single one, by
collecting the layers of all input maps into the output map.

.. code-block:: bash

    Usage: mm-merge [OPTIONS]

    Options:
      -h,--help                   Print this help message and exit
      -i,--input TEXT ... REQUIRED
                                  Input .mm files to merge (at least two). Repeat the flag or list several files. Non-layer data (id, label) is taken from the first input.
      -o,--output TEXT REQUIRED   Output .mm file to write to
      --on-conflict TEXT:{error,rename}=error
                                  What to do if a layer name exists in more than one input: 'error' aborts listing the conflicting layers; 'rename' appends the suffix '_<N>' to the layer of the N-th input (1-based) and keeps both.
      --rename TEXT ...           Renames a layer of one input before merging, with format "N:OLD_NAME|NEW_NAME", N being the 1-based index of the input file. Can be repeated.
      -l,--load-plugins TEXT      One or more (comma separated) *.so files to load as plugins


Merge rules
-------------

- **Layers**: all layers of all inputs are copied to the output map. Layers are never
  combined: if the same layer name exists in more than one input, that is a conflict
  (see below).
- **Lines and planes**: concatenated.
- **id and label**: taken from the first input.
- **metadata**: keys from all inputs are merged. If a key exists in more than one input,
  the value from the earliest input is kept, and a warning is printed.
- **Georeferencing**: all georeferenced inputs must have exactly the same geodetic
  reference and ``T_enu_to_map`` pose, otherwise the tool aborts. Inputs without
  georeferencing are accepted, and the output keeps the georeferencing of the inputs
  that have it. Note that no coordinate transformation is applied: all inputs are
  assumed to be in the same local frame.

Layer name conflicts
----------------------

By default (``--on-conflict error``), the tool aborts and lists the conflicting layer
names. Conflicts can be resolved in two ways:

- ``--rename "N:OLD_NAME|NEW_NAME"``: explicitly renames a layer of the N-th input file
  (1-based) before merging. It can be repeated.
- ``--on-conflict rename``: any remaining conflicting layer of the N-th input is renamed
  to ``<name>_<N>``.

Explicit renames are applied first, so both options can be combined.

Example usage
---------------

.. code-block:: bash

    # Fails if both maps have a layer with the same name:
    mm-merge -i map1.mm -i map2.mm -o merged.mm

    # Rename the "points" layer of the second map before merging:
    mm-merge -i map1.mm -i map2.mm -o merged.mm --rename "2:points|points_map2"

    # Merge three maps, automatically renaming conflicting layers
    # (e.g. "points", "points_2", "points_3"):
    mm-merge -i map1.mm map2.mm map3.mm -o merged.mm --on-conflict rename

Use :ref:`mm-info <app_mm-info>` to inspect the layers of the input and output maps, and
:ref:`mm-filter <app_mm-filter>` (``--rename-layer``) to rename layers afterwards.

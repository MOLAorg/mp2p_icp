.. _app_icp-run:

===============================
Application: ``icp-run``
===============================

``icp-run`` runs one ICP alignment between two point clouds or metric maps, with an ICP
pipeline defined in a :ref:`YAML file <mp2p_icp_yaml_pipelines>`. It is meant to test and tune
pipelines: with ``--generate-debug-log``, the whole optimization can be inspected afterwards
in :ref:`icp-log-viewer <app_icp-log-viewer>`.

Each input (local and global) can be a metric map (``*.mm``), an observation of a rawlog
(``dataset.rawlog:N``, the N-th entry), the local map of an ICP log (``*.icplog``), or a
text file with one ``x y z`` point per line.

Example
-------

From the ``mp2p_icp_core`` directory of the sources:

.. code-block:: bash

    # 2D ICP with point-to-point pairings:
    icp-run \
      --input-local demos/local_001.mm \
      --input-global demos/global_001.mm \
      -c demos/icp-settings-2d-lidar-example-point2point.yaml \
      --generate-debug-log

    # Inspect the debug log:
    icp-log-viewer

Usage
-----

.. code-block:: text

    Usage: icp-run [OPTIONS]

    Options:
      -h,--help                   Print this help message and exit
      --input-local TEXT REQUIRED Local input point cloud/map. It is interpreted as a rawlog
                                  entry if using the format `<RAWLOG_FILE.rawlog>:<N>` to
                                  select the N-th entry in the rawlog; otherwise, if the file
                                  extension is `.mm` it is loaded as a serialized metric_map_t
                                  object; if it is a `.icplog` file, the local map from that
                                  icp log is taken as input; in any other case, the file is
                                  assumed to be a 3D pointcloud stored as a Nx3 ASCII matrix
                                  file.
      --input-global TEXT REQUIRED
                                  Global input point cloud/map. Same format than input-local.
      -c,--config TEXT REQUIRED   YAML config file describing the ICP pipeline.
      --config-generators TEXT    YAML config file describing the Generators. Can be also
                                  defined via an entry `generators` in the main `--config`
                                  yaml file. Can be used when processing a rawlog as input; if
                                  not present, a default Generator object will be used.
      --config-filters-local TEXT YAML config file describing a filtering pipeline for local
                                  map. If not provided, and the main --config yaml file
                                  contains a `filters` entry (can be overriden with
                                  --entry-name-filters-local), it will be used instead.
      --config-filters-global TEXT
                                  YAML config file describing a filtering pipeline for global
                                  map. If not provided, and the main --config yaml file
                                  contains a `filters` entry (can be overriden with
                                  --entry-name-filters-global), it will be used instead.
      --entry-name-filters-global TEXT
                                  Overrides the map name in the YAML configuration file for
                                  global map filter.
      --entry-name-filters-local TEXT
                                  Overrides the map name in the YAML configuration file for
                                  local map filter.
      --guess TEXT                SE(3) transformation of local wrt global, to use as initial
                                  guess for the ICP algorithm. Format:
                                  "[x y z yaw_deg pitch_deg roll_deg]"
      -d,--generate-debug-log     Enforces generation of the .icplog debug log files for
                                  posterior visualization with icp-log-viewer, overriding the
                                  `generateDebugFiles` value in the configuration YAML file.
      --profiler                  Enables the ICP profiler.
      -l,--load-plugins TEXT      One or more (comma separated) *.so files to load as plugins

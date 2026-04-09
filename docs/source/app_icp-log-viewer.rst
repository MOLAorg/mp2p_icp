.. _app_icp-log-viewer:

=====================================
Application: ``icp-log-viewer``
=====================================

``icp-log-viewer`` is an interactive GUI tool to inspect and debug ICP (Iterative Closest Point)
registration sessions stored as ``*.icplog`` files.

.. raw:: html

   <div style="width: 100%; overflow: hidden;">
     <video controls autoplay loop muted style="width: 100%;">
       <source src="https://mrpt.github.io/videos/mp2p_icp-log-viewer-demo.mp4" type="video/mp4">
     </video>
   </div>


How to launch
------------------

Once mp2p_icp is installed, move to the directory where your ``*.icplog`` files have been stored and run:

.. code-block:: bash

    icp-log-viewer

You can also point it directly at a single file or a specific directory:

.. code-block:: bash

    # Load all *.icplog files in a directory
    icp-log-viewer -d /path/to/logs/

    # Load a single file
    icp-log-viewer -f /path/to/log.icplog

    # Only load files where the ICP quality is >= 0.5 (50%)
    icp-log-viewer -d /path/to/logs/ --min-quality 0.5


.. dropdown:: Complete command line argument help

    .. code-block:: bash

        USAGE:

          icp-log-viewer  [--autoplay-period <period [seconds]>] [-q <quality [0,1]>]
                          [-l <foobar.so>] [-f <log.icplog>] [-d <.>] [-e <icplog>]
                          [--] [--version] [-h]


        Where:

          --autoplay-period <period [seconds]>
            The period (in seconds) between timestamps to load and show in
            autoplay mode.

          -q <quality [0,1]>,  --min-quality <quality [0,1]>
            Minimum ICP quality (range [0,1], i.e. 0%-100%) to load a log file.
            Files whose ICP result quality is below this threshold are skipped.
            This is useful to focus inspection on well-converged ICP results.

          -l <foobar.so>,  --load-plugins <foobar.so>
            One or more (comma separated) *.so files to load as plugins

          -f <log.icplog>,  --file <log.icplog>
            Load just this one single log *.icplog file.

          -d <.>,  --directory <.>
            Directory in which to search for *.icplog files.

          -e <icplog>,  --file-extension <icplog>
            Filename extension to look for. Default is ``icplog``.

          --,  --ignore_rest
            Ignores the rest of the labeled arguments following this flag.

          --version
            Displays version information and exits.

          -h,  --help
            Displays usage information and exits.


GUI and feature explanation
-------------------------------

The viewer window is divided into a left control panel and a main 3D viewport.

**Log selector**

A slider at the top of the panel lets you step through all loaded ``*.icplog`` entries
(filtered by ``--min-quality`` if requested at launch).  Use the **Back** / **Forward** buttons
for single-step navigation, or enable **Autoplay** to advance automatically at the rate set by
``--autoplay-period``.

**3D viewport**

The main 3D view shows the *global* map and the *local* point cloud registered against it.
The initial-guess pose and the final ICP pose can be toggled independently.

**ICP statistics panel**

Numerical outputs shown for each selected log entry:

- **Quality** — the ICP quality score in [0, 1] produced by the configured
  ``QualityEvaluator`` (e.g. ``QualityEvaluator_PairedRatio``).
- **Log pose** — the final estimated SE(3) pose of the local map.
- **Initial guess** — the pose used to seed the ICP iteration.
- **Init→Final** — the correction applied by ICP on top of the initial guess.
- **Covariance / condition number** — uncertainty of the result.
- **Pairings** — number of point correspondences in the last ICP iteration.

**Pairings visualisation**

Enable **Show pairings** to draw the point correspondences found at the last ICP
iteration.  Individual pairing types (point-to-point, point-to-plane, point-to-line,
covariance-to-covariance) can be toggled independently.

**Filtering by quality at load time**

When a directory contains many log files (e.g. from a full SLAM session) it is often
useful to inspect only the low-quality registrations that may have caused drift.  Use
``--min-quality`` to restrict the loaded set:

.. code-block:: bash

    # Inspect only high-quality (≥ 80 %) registrations
    icp-log-viewer -d logs/ --min-quality 0.8

    # Inspect only low-quality (< 20 %) registrations — invert by running with 0
    # and then sorting/filtering externally, or set a low threshold and browse manually
    icp-log-viewer -d logs/ --min-quality 0.0

.. MP2P_ICP documentation master file

======================
About `mp2p_icp`
======================

A collection of C++ libraries and tools for multi-primitive-to-primitive (MP2P) matching,
optimization, ICP-related algorithms, and point-cloud pipeline design. 
Part of the `MOLA <index.html>`_ project.

.. The toctree now lives in the root MOLAorg/mola repo

How to cite it
==============

.. rst-class:: fa fa-github

`mp2p_icp <https://github.com/MOLAorg/mp2p_icp>`_ has been used in these papers:

- The ``mola_lidar_odometry`` system, presented in :cite:`blanco2025mola_lo`.
- Formerly, it was used in 2019 in :cite:`blanco2019modular`.

.. _mp2p_icp-install:

Install mp2p_icp
======================

.. note::

    You are reading installation instructions for the mp2p_icp package only.
    For installing the complete MOLA odometry and SLAM system, see :ref:`installing`.


How to install the C++ mp2p_icp libraries and :ref:`applications <mp2p_icp_applications>`:

.. note::

    Since this repository was split into the ``mp2p_icp_core`` (headless libraries and CLI
    applications, no GUI dependencies), ``mp2p_icp_viz`` (GUI applications: ``mm-viewer``,
    ``icp-log-viewer``), and ``mp2p_icp`` (metapackage, depends on both) ROS packages,
    standalone (non-colcon) plain-CMake builds are no longer supported. Build within a ROS
    workspace, either the whole thing via the ``mp2p_icp`` metapackage, or just
    ``mp2p_icp_core`` if you only need the headless libraries/apps.

.. tab-set::
    .. tab-item:: From ROS
        :selected:

        Probably the easiest way to get ``mp2p_icp``:

        .. code-block:: bash

            sudo apt install ros-${ROS_DISTRO}-mp2p-icp

        To only install the headless libraries and CLI applications (no GUI dependencies):

        .. code-block:: bash

            sudo apt install ros-${ROS_DISTRO}-mp2p-icp-core

    .. tab-item:: Compile (with colcon)

        You can build ``mp2p_icp`` within a ROS 2 workspace using colcon, just as with any other package:

        .. code-block:: bash

            mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
            git clone --recurse-submodules https://github.com/MOLAorg/mp2p_icp.git
            cd ~/ros2_ws/
            colcon build --symlink-install --packages-up-to mp2p_icp --cmake-args -DCMAKE_BUILD_TYPE=Release
            . install/setup.bash

        Or, to only build the headless libraries/apps:

        .. code-block:: bash

            colcon build --symlink-install --packages-up-to mp2p_icp_core --cmake-args -DCMAKE_BUILD_TYPE=Release

Demos
=================

icp-run demos
---------------------

.. code-block:: bash

    # 2D icp with point-to-point pairings only:
    icp-run \
      --input-local mp2p_icp_core/demos/local_001.mm \
      --input-global mp2p_icp_core/demos/global_001.mm \
      -c mp2p_icp_core/demos/icp-settings-2d-lidar-example-point2point.yaml \
      --generate-debug-log

    # Inspect the debug log:
    icp-log-viewer


.. code-block:: bash

    # 2D icp with point-to-line pairings:
    icp-run \
      --input-local mp2p_icp_core/demos/local_001.mm \
      --input-global mp2p_icp_core/demos/global_001.mm \
      -c mp2p_icp_core/demos/icp-settings-2d-lidar-example-point2line.yaml \
      --generate-debug-log

    # Inspect the debug log:
    icp-log-viewer

.. .. raw:: html
.. 
..    <div style="width: 100%; overflow: hidden;">
..      <video controls autoplay loop muted style="width: 100%;">
..        <source src="https://xxx.mp4" type="video/mp4">
..      </video>
..    </div>

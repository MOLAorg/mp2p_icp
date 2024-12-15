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

- The ``mola_lidar_odometry`` system, presented in :cite:`blanco2024mola_lo`.
- Formerly, it was used in 2019 in :cite:`blanco2019modular`.

.. _mp2p_icp-install:

Install mp2p_icp
======================

.. note::

    You are reading installation instructions for the mp2p_icp package only.
    For installing the complete MOLA odometry and SLAM system, see :ref:`installing`.


How to install the C++ mp2p_icp libraries and :ref:`applications <mp2p_icp_applications>`:

.. tab-set::
    .. tab-item:: From ROS
        :selected:

        Probably the easiest way to get ``mp2p_icp``:

        .. code-block:: bash

            sudo apt install ros-${ROS_DISTRO}-mp2p-icp


    .. tab-item:: Compile (no ROS)

        Get the sources
        -------------------

        Clone the project git repository and its submodules:

        .. code-block:: bash

            mkdir -p ~/code/mp2p_icp && cd ~/code/mp2p_icp
            git clone --recurse-submodules https://github.com/MOLAorg/mp2p_icp.git

        Get the build dependencies
        ----------------------------
        - A C++17 compiler
        - CMake >=3.4
        - MRPT >= 2.11.5

        .. tab-set::
            .. tab-item:: Building MRPT from sources

                Follow the `installation instructions <https://docs.mrpt.org/reference/latest/compiling.html>`_ for MRPT

            .. tab-item:: Get from ROS 2
                :selected:

                .. code-block:: bash

                    sudo apt install ros-$ROS_DISTRO-mrpt2

            .. tab-item:: Get from PPA

                .. code-block:: bash

                    # MRPT, from this PPA, or from its ROS package, or build from sources if preferred:
                    sudo add-apt-repository ppa:joseluisblancoc/mrpt
                    sudo apt update
                    sudo apt install libmrpt-dev mrpt-apps


        Compile
        ---------------------
        Classic cmake stuff:

        .. code-block:: bash

            mkdir build-Release
            cmake -B build-Release -S . -DCMAKE_BUILD_TYPE=Release
            cmake --build build-Release
            (cd build-Release  && make test)  # to run tests

    .. tab-item:: Compile (with colcon)

        You can build ``mp2p_icp`` within a ROS 2 workspace using colcon, just as with any other package:

        .. code-block:: bash

            mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
            git clone --recurse-submodules https://github.com/MOLAorg/mp2p_icp.git
            cd ~/ros2_ws/
            colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
            . install/setup.bash


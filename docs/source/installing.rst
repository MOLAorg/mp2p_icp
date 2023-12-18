.. _installing:

======================
Installing
======================

1. From a ROS distribution
==============================

Probably the easiest way to get `mp2p_icp`:

.. code-block:: bash

    sudo apt install ros-${ROS_DISTRO}-mp2p-icp


2. Compile from sources (pure CMake)
========================================

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
- MRPT >= 2.11.3

For modern Ubuntu distributions, install all dependencies with:

.. code-block:: bash

    # MRPT >2.1.0, for now from this PPA (or build from sources if preferred):
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

3. Compile from sources with colcon (for ROS 2 apps)
======================================================

You can build `mp2p_icp` within a ROS 2 workspace using colcon, just as with any other package:

.. code-block:: bash

    mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
    git clone --recurse-submodules https://github.com/MOLAorg/mp2p_icp.git
    cd ~/ros2_ws/
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    . install/setup.bash


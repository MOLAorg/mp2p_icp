.. _mp2p_icp_python:

======================
Python API
======================

The ``mp2p_icp`` Python package wraps the ``mp2p_icp_core`` C++ libraries: metric maps
(``*.mm`` files), ICP pipelines, filters and generators, plus point cloud file formats and
conversions to other Python libraries.

It builds on the ``mrpt`` Python package (MRPT 3): point clouds are ``mrpt.maps`` objects
(``CGenericPointsMap``, ``CSimplePointsMap``...) and poses are ``mrpt.poses`` / ``mrpt.math``
types, so both APIs can be mixed freely.

Install
=======

**pip** (Linux x86_64 and aarch64, Python 3.11 to 3.14): ``pip install mp2p-icp``. It also
installs the ``mrpt3`` wheel, whose MRPT libraries it uses.

**ROS 2**: the bindings are built with ``mp2p_icp_core`` whenever ``pybind11`` is found
(``pybind11-dev`` on Debian/Ubuntu). After ``source install/setup.bash`` (or the ROS
distribution setup file), ``import mp2p_icp`` just works.

Do not mix both in the same Python environment.

Optional dependencies, imported only by the functions that need them: ``pandas``,
``pyarrow`` (Parquet), ``open3d``, ``pyvista``, ``laspy``. With pip, install them with the
extras ``mp2p-icp[pandas]`` (pandas and pyarrow), ``[open3d]``, ``[pyvista]``, ``[las]``, or
``[all]``.

Metric maps and point clouds
============================

.. code-block:: python

    import numpy as np
    import mp2p_icp

    m = mp2p_icp.MetricMap.from_file("map.mm")  # raises on errors
    print(m)                                   # contents summary
    print(m.layers.keys(), m.id, m.label, m.metadata, m.georeferencing)

    # Layers: a dict-like view of mrpt.maps.CMetricMap objects
    layer = m.layers["raw"]

    # All per-point fields as NumPy arrays (copies): x, y, z, intensity, ring, t...
    fields = mp2p_icp.layer_to_numpy(layer)

    # New layers from NumPy: an (N,3) array, or a dict of 1-D field arrays
    keep = fields["z"] > 0.5
    m.layers["high"] = {name: a[keep] for name, a in fields.items()}
    # ...or explicitly, into an mrpt.maps.CGenericPointsMap:
    pc = mp2p_icp.layer_from_numpy(np.random.rand(100, 3))

    m.metadata = {"note": "points above 0.5 m"}
    m.save("out.mm")

Maps built with MOLA may have layers of classes from other libraries (``mola::...``). If the
``mola_metric_maps`` Python module is installed (``pip install mola-metric-maps``), it is
imported automatically the first time a map, an ICP log or a pipeline needs one of its classes.
With the ROS packages, load the library instead, before reading such maps:
``mp2p_icp.load_plugins("libmola_metric_maps.so")``.

Point cloud files:

- ``load_ply()`` / ``save_ply()``: PLY, with all point fields.
- ``load_pcd()`` / ``save_pcd()``: PCD (ascii, binary; binary_compressed for reading).
- ``save_las()``: LAS 1.4, as the ``mm2las`` tool (``mp2p_icp.interop.from_laspy()`` reads
  LAS/LAZ files with ``laspy``).
- ``save_txt()``: text, as the ``mm2txt`` tool; ``load_xyz_file()`` reads ``x y z`` text
  files.

Registration (ICP)
==================

One call, with a bundled configuration (``mp2p_icp.list_configs()``: ``2d-point2point``,
``3d-point2point``, ``3d-point2plane``). Inputs may be NumPy arrays, ``mrpt.maps`` point
clouds, or ``MetricMap`` objects:

.. code-block:: python

    res = mp2p_icp.align(local_xyz, global_xyz, "3d-point2plane",
                         initial_guess=[0, 0, 0, 0, 0, 0],  # x y z yaw pitch roll
                         variables={"ICP_THRESHOLD": 1.0, "ICP_DECIMATION": 0.2})
    print(res.optimal_tf.mean, res.quality, res.terminationReason)
    T = res.optimal_pose_matrix  # 4x4 NumPy array

The bundled configurations are YAML files (``mp2p_icp.config_path(name)``) with
``${VARIABLES}`` for their main thresholds: copy and modify one for custom pipelines.

Full control, with any ICP YAML configuration file (or a dict):

.. code-block:: python

    icp, params = mp2p_icp.icp_pipeline_from_yaml("icp-pipeline.yaml", variables={"X": 1})
    params.maxIterations = 50

    def on_iteration(info):  # optional, called at each iteration
        print(info.iteration, info.pose, info.num_pairings)
        return False  # True stops the iterations

    icp.set_iteration_hook(on_iteration)

    log = mp2p_icp.LogRecord()  # optional: record of the run, as .icplog files
    res = icp.align(local_map, global_map, initial_guess, params, log=log)
    log.save("run.icplog")  # can be opened with icp-log-viewer

Pipeline parameters written as formulas of variables (e.g. ``threshold: "2*SCALE"``) are set
with ``icp.update_variables({"SCALE": 0.5})``.

Filters, generators and simple-maps
===================================

.. code-block:: python

    filters = mp2p_icp.filter_pipeline_from_yaml("mm-filter-pipeline.yaml")
    filters.apply(m)  # in place

    m = mp2p_icp.simplemap_to_metricmap("dataset.simplemap", "sm2mm-pipeline.yaml")

``Generators`` convert MRPT observations (``mrpt.obs``) into map layers.

Rendering
=========

``mp2p_icp.render()`` renders maps off-screen into images (NumPy arrays), as
:ref:`mm-render <app_mm-render>` does. It needs the ``mrpt.opengl`` module and an OpenGL
(EGL) capable system, but no display.

.. code-block:: python

    img = mp2p_icp.render(m, 1280, 720, azimuth=-45, elevation=35, colormap="jet")

Other libraries
===============

``mp2p_icp.interop`` converts point clouds, with their fields, to and from pandas
DataFrames, Parquet files, Open3D (legacy and tensor) point clouds, PyVista ``PolyData``
and ``laspy`` objects:

.. code-block:: python

    from mp2p_icp import interop

    df = interop.to_dataframe(layer)
    pcd = interop.to_open3d(layer)
    layer2 = interop.from_laspy("cloud.laz")

C++ extensions
==============

The pip wheel also ships the C++ SDK of these libraries (``mp2p_icp.sdk``: headers, shared
libraries, CMake configs, and the ``mola_common`` CMake functions), as the ``mrpt3`` wheel does
for MRPT (``mrpt.sdk``), so other wheels can build C++ code and Python modules on top of them.
With scikit-build-core, ``find_package(mp2p_icp_map)`` (or ``mp2p_icp``, ``mp2p_icp_filters``,
``mola_common``...) just works, since both SDKs are ``cmake.prefix`` entry points. Other build
systems add ``mp2p_icp.sdk.get_cmake_prefix_path()`` and ``mrpt.sdk.get_cmake_prefix_path()``
to ``CMAKE_PREFIX_PATH``.

pybind11 modules built this way can return and accept the types bound here
(``mp2p_icp.MetricMap``...) if they use the same pybind11 version
(``mp2p_icp.sdk.PYBIND11_VERSION``) and ``import mp2p_icp`` first. See
``packaging/pip/README-dev.md`` in the repository for the details.

Notes
=====

- Calls into the C++ libraries (ICP, filters, file I/O...) release the Python GIL, so
  they can run in parallel from several Python threads, on different objects.
- Functions raise Python exceptions on errors (``FileNotFoundError``, ``RuntimeError``...).
- ``MetricMap.metadata``, ``georeferencing``, ``lines`` and ``planes`` return copies:
  assign them to change them. ``MetricMap.layers`` returns the layer objects themselves.

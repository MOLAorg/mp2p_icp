.. _app_mm-viewer:

===============================
Application: ``mm-viewer``
===============================

GUI application to visualize metric map files (``.mm``).

Supported input file types
--------------------------

- **Metric map files** (``.mm``): the native mp2p_icp format, containing one or
  more named layers of arbitrary map types.

- **Binary point cloud files** (``.bin``): files containing an externally-stored
  ``mrpt::maps::CGenericPointsMap``-derived object, serialized with MRPT's
  ``CCompressedInputStream`` (supports uncompressed, gzip, and zstandard formats).
  The deserialized cloud is loaded as a single layer named after the file stem.

Usage
-----

.. code-block:: bash

    USAGE:

    mm-viewer  [-s <scene.3dscene>] ...  [-t <trajectory.tum>] [-l
                <foobar.so>] [--] [--version] [-h] <myMap.mm>


    Where:

    -s <scene.3dscene>,  --add-3d-scene <scene.3dscene>  (accepted multiple
        times)
        Adds an extra 3D scene file (*.3dscene) for visualization.

    -t <trajectory.tum>,  --trajectory <trajectory.tum>
        Also draw a trajectory, given by a TUM file trajectory.

    -l <foobar.so>,  --load-plugins <foobar.so>
        One or more (comma separated) *.so files to load as plugins

    --,  --ignore_rest
        Ignores the rest of the labeled arguments following this flag.

    --version
        Displays version information and exits.

    -h,  --help
        Displays usage information and exits.

    <myMap.mm>
        Load this metric map file (``*.mm``) or binary point cloud (``*.bin``)



Camera fly-by videos
--------------------

The **Travelling** panel defines a camera path from keyframes:

1. Move the camera to a desired view, set the keyframe time (seconds) and press
   **Add current view**. Repeat for each keyframe. Keyframes in the list can be
   selected and then moved to (**Go to**, or double-click), replaced by the current
   view (**Update**), or deleted. **Save path...** / **Load path...** store the path
   as a text file with one ``t x y z azimuth_deg elevation_deg zoom`` line per keyframe.
2. **Play** previews the path in real time. ``Linear`` interpolation moves at constant
   speed between keyframes, while ``Spline`` (Catmull-Rom) passes through all of them
   with smooth velocity. The time slider moves the camera to any point of the path.
3. **Record** renders every frame off-screen at the given size and FPS, and saves them as
   ``frame_000000.png``, ``frame_000001.png``, ... in the chosen folder. Encode them into a
   video with, for example:

   .. code-block:: bash

       ffmpeg -framerate 30 -i mm-viewer-frames/frame_%06d.png -c:v libx264 -pix_fmt yuv420p video.mp4

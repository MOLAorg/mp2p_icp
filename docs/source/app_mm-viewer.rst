.. _app_mm-viewer:

===============================
Application: ``mm-viewer``
===============================

Write me!

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
        Load this metric map file (*.mm)



=================
Demos
=================

.. contents:: :local:



icp-run demos
---------------------

.. code-block:: bash

    # 2D icp with point-to-point pairings only:
    icp-run \
      --input-local demos/local_001.mm \
      --input-global demos/global_001.mm \
      -c demos/icp-settings-2d-lidar-example-point2point.yaml \
      --generate-debug-log

    # Inspect the debug log:
    icp-log-viewer


.. code-block:: bash

    # 2D icp with point-to-line pairings:
    icp-run \
      --input-local demos/local_001.mm \
      --input-global demos/global_001.mm \
      -c demos/icp-settings-2d-lidar-example-point2line.yaml \
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

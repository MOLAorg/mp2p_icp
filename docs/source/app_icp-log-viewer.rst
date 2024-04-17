
=====================================
Application: ``icp-log-viewer``
=====================================

Debug ICP pipelines as never before!

.. raw:: html

   <div style="width: 100%; overflow: hidden;">
     <video controls autoplay loop muted style="width: 100%;">
       <source src="mp2p_icp-log-viewer-demo.mp4" type="video/mp4">
     </video>
   </div>


How to launch
------------------

Once mp2p_icp is installed, move to the directory where your `*.icplog` files have been stored and run:

.. code-block:: bash

    icp-log-viewer


.. dropdown:: Complete command line argument help

    .. code-block:: bash

        USAGE:

          icp-log-viewer  [--autoplay-period <period [seconds]>] [-l <foobar.so>]
                          [-f <log.icplog>] [-d <.>] [-e <icplog>] [--]
                          [--version] [-h]


        Where: 

          --autoplay-period <period [seconds]>
            The period (in seconds) between timestamps to load and show in
            autoplay mode.

          -l <foobar.so>,  --load-plugins <foobar.so>
            One or more (comma separated) *.so files to load as plugins

          -f <log.icplog>,  --file <log.icplog>
            Load just this one single log *.icplog file.

          -d <.>,  --directory <.>
            Directory in which to search for *.icplog files.

          -e <icplog>,  --file-extension <icplog>
            Filename extension to look for. Default is `icplog`

          --,  --ignore_rest
            Ignores the rest of the labeled arguments following this flag.

          --version
            Displays version information and exits.

          -h,  --help
            Displays usage information and exits.


GUI and feature explanation
-------------------------------

Write me!

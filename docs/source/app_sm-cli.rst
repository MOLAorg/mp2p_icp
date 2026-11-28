.. _app_sm-cli:

===============================
Application: ``sm-cli``
===============================

CLI tool to manipulate and inspect simple-maps:

Available commands:

.. code-block:: bash

    sm-cli cut                Cut part of a .simplemap file into a new file.
    sm-cli export-keyframes   Export KF poses (opt: twist too) as TUM format.
    sm-cli export-rawlog      Export KFs as rawlog for inspection.
    sm-cli info               Analyze a .simplemap file.
    sm-cli join               Join two or more .simplemap files into one.
    sm-cli level              Levels a .simplemap from its keyframe heights.
    sm-cli level-walls        Levels a .simplemap from its wall/floor normals.
    sm-cli partition          Splits a .simplemap into submaps of the same place.
    sm-cli tf                 Applies a SE(3) transform by the left to a map.
    sm-cli trim               Extracts part of a .simplemap inside a given box.
    sm-cli --version          Shows program version.
    sm-cli --help             Shows this information.

    Or use `sm-cli <COMMAND> --help` for further options

|

sm-cli cut
---------------
Cuts a simple-map by keyframe indices, saving the smaller simple-map to a new file:

.. code-block:: bash

    sm-cli cut --help
    Usage:

        sm-cli cut <filename> --from <FIRST_KF_INDEX> --to <LAST_KF_INDEX> --output <OUTPUT.simplemap>


|


sm-cli export-keyframes
-------------------------
Saves the key-frames in a simple-map as a trajectory file in TUM format:

Refer to the tutorial for example data file and command line: :ref:`building-maps_sect_inspect_sm`.

.. code-block:: bash

    sm-cli export-keyframes <filename> --output <OUTPUT.tum> [--output-twist <TWIST.txt>]


|


sm-cli export-rawlog
----------------------
Export all keyframes in the simplemap, including pose and twist information, metadata (see MOLA-LO paper),
and raw sensor observations (3D LiDAR scans, GNSS data, etc.) to a RawLog file, which can be easily
browsed with RawLogViewer.

Refer to the tutorial for example data file and command line: :ref:`building-maps_sect_inspect_sm`.


.. code-block:: bash

    sm-cli export-rawlog <filename.simplemap> --output <OUTPUT.rawlog>

|

sm-cli info
----------------------
Shows basic information about the contents of a simple map.

Refer to the tutorial for example data file and command line: :ref:`building-maps_sect_inspect_sm`.

.. code-block:: bash

    sm-cli info <filename.simplemap>

|

sm-cli level
----------------------
Takes an input simple-map and optimizes its key-frame poses such as they lie on an horizontal plane as much as possible,
saving the result in another simple-map file. This can be used when a map has an unintentional tilt for some reason, for example, wrong or missing sensor extrinsics.

.. code-block:: bash

    sm-cli level <input.simplemap> <output.simplemap>

This assumes a vehicle moving on flat ground. For handheld or aerial maps of buildings, whose trajectory
changes height on purpose, use ``sm-cli level-walls`` instead.

|

sm-cli level-walls
----------------------
Corrects a global pitch/roll error of a simple-map by rotating it about the map origin, so that walls become
as vertical, and floors and ceilings as horizontal, as possible. A typical source of such error is the initial
attitude of a LiDAR-inertial run, estimated from a short accelerometer average.

.. code-block:: bash

    sm-cli level-walls <input.simplemap> <output.simplemap>
        [--pipeline <sm2mm-pipeline.yaml>]  # keyframes to points (default: built-in)
        [--voxel 0.05]                      # analysis cloud voxel size [m]
        [--normal-radius 0.4]               # neighborhood for local normals [m]
        [--wall-nz 0.25] [--flat-nz 0.95]   # |n.up| thresholds for walls / floors
        [--max-correction-deg 5]            # refuse larger corrections
        [--estimate-only]                   # print the rotation, do not write output
        [--windows N]                       # also estimate on N equal time windows
        [--piecewise SECONDS]               # level each time window on its own
        [--by-submap <partition.yaml>]      # level each submap (sm-cli partition) on its own
        [--max-link-length 5.0]             # --by-submap: longest same-place link [m]
        [--link-weight 10]                  # --by-submap: weight of same-place links
        [--up-step-sigma-deg 1.0]           # --by-submap: stiffness of the up field
        [--walls-only]                      # walls are the only vertical reference
        [--cluster-size 2.0]                # cells grouping normals for the sigmas [m]
        [--max-window-sigma-deg 1.0]        # --piecewise skips less certain windows
        [--max-walls-flats-deg 2.0]         # refuse when walls contradict floors
        [--windows-out <file.txt>]          # one row per window, machine readable
        [--input-tum <in.tum> --output-tum <out.tum>]  # level a dense path too

How it works:

- Keyframes are converted into a voxel-decimated point cloud with an sm2mm pipeline (by default, the default
  generator plus a per-keyframe voxel filter). With ``--pipeline``, all point layers it outputs are merged.
- A normal is estimated for each point with a planar neighborhood.
- Starting from :math:`u=+Z`, normals are classified as walls (:math:`|n \cdot u|` < ``wall-nz``) or floors and
  ceilings (:math:`|n \cdot u|` > ``flat-nz``), and :math:`u` is updated to the eigenvector of the smallest
  eigenvalue of :math:`M = \frac{1}{|W|}\sum_W n n^T + \frac{1}{|F|}\sum_F (I - n n^T)`, a few times.
- The map is rotated by the minimal rotation that takes :math:`u` to :math:`+Z` (so the heading is kept).
  Only keyframe poses change, exactly as with ``sm-cli tf``, whose equivalent command is printed.

The command aborts if :math:`u` is not observable (e.g. one wall direction and no floors), or if the correction
exceeds ``--max-correction-deg``. ``--windows N`` repeats the estimate on N time windows: similar values mean a
fixed gauge error that a single rotation removes, while differing values mean drift, which ``--piecewise`` fixes.

``--piecewise SECONDS`` levels a map whose vertical drifts during the run. :math:`u` is estimated on consecutive
time windows of that duration, its horizontal components are interpolated linearly in time between the window
centers (held constant before the first and after the last), and each keyframe :math:`k` is rotated by its own
minimal rotation :math:`C(t_k)` taking :math:`u(t_k)` to :math:`+Z`. Positions are integrated again from the first
keyframe, each step rotated by the correction at its mid time, so the trajectory stays continuous:
:math:`p'_k = p'_{k-1} + C(t_{mid}) (p_k - p_{k-1})`. Windows where :math:`u` is not observable, has no reliable
sigma (fewer than 8 cells of normals; not checked with ``--cluster-size 0``) or a sigma above ``--max-window-sigma-deg`` are skipped and interpolated
over, and a last window shorter than half the others joins the previous one. The command aborts if any window
used needs more than ``--max-correction-deg``.
Windows of 20 to 30 s worked well on indoor walks; shorter ones follow the drift more closely but are noisier.

``--by-submap <partition.yaml>`` levels a map whose vertical error depends on the place, which is the usual case:
a revisit registers against the same local map as the first visit, so both share the error of that place. A
correction that depends on time, as ``--piecewise``, gives the two visits different rotations and draws the same
floor twice. Here :math:`u` is estimated on each submap of a partition made by ``sm-cli partition`` (keyframes that
observe the same place, whenever they were visited), and the correction of each keyframe comes from a smooth field
over the graph of shared content: the submap estimates, each one weighted by its covariance (an axis the walls do not
constrain counts little, and an extra 0.3 deg stands for the systematic error of the references of a place), against
a prior that keyframes seeing the same content have similar corrections, with standard deviation
``--up-step-sigma-deg`` for two keyframes that share as much content as a typical keyframe sees. A surface seen with
two different corrections is drawn twice, apart by their difference times the range, so this stiffness trades the
curvature that can be followed for the thickness of floors and walls. Each keyframe is then rotated by its own
correction, and positions are solved by least squares, not integrated in time: on the graph of consecutive keyframes
plus links between keyframes of the same place (closer than ``--max-link-length``, with ``--link-weight`` times their
similarity), :math:`p'_j - p'_i = C_{ij} (p_j - p_i)`, with :math:`p'_0 = p_0` and :math:`C_{ij}` the correction of the
mean up of both. Two visits of a place keep their relative position. The log reports the residual of that solve (the
part of the field that is not a gradient) and, as for ``--piecewise``, how much the height difference of keyframes
that revisit a place (closer than 0.3 m in plan and 1 m in height, more than 30 s apart) changed. ``--input-tum``
moves each pose of the dense path with the two keyframes around it in time. ``--windows-out`` writes one row per
submap, with the estimate used and whether it was used.

The same safety rules as ``--piecewise`` apply, per submap:

- a submap without a reliable sigma, or with one above ``--max-window-sigma-deg``, is not used: it follows its
  neighbors in the graph;
- the map is refused when a used submap needs more than ``--max-correction-deg``, or when the walls contradict the
  floors in four or more of every five keyframes (counted by the submaps they belong to);
- a submap whose walls contradict its floors (stairs, a ramp) is estimated from its walls alone.

Walls are checked against floors. Where the walls alone and the floors alone give :math:`u` more than
``--max-walls-flats-deg`` apart (default 2), they contradict each other: one of the two references is not plumb or
level (a sloped field, a ramp, the rock faces of a quarry). The whole-map command then refuses to level and exits with
an error, also with ``--estimate-only``. ``--piecewise`` and ``--by-submap`` refuse the map when the walls contradict
the floors in four or more of every five windows (keyframes, for submaps) that gave an estimate, and otherwise they
level with all usable windows as before. Windows whose walls alone do not give :math:`u` (one wall direction, too few
walls) can neither confirm nor contradict their floors: they are reported, never refused. On 30 s windows of
college and campus recordings the walls contradicted the floors in 0-45 % of the windows (sloped lawns and streets);
on an open quarry, in 9 of 9.

``--walls-only`` drops floors and ceilings from the estimate (they are not a vertical reference where a floor
slopes, e.g. a ramp), so walls with at least two directions are needed. Each estimate also reports the standard
deviations of :math:`u_x, u_y` from a sandwich covariance, which takes the normals in cells of ``--cluster-size``
as the independent groups, and ``--windows-out`` writes one row per window
(``window kf_first kf_last t_start t_end up_x up_y up_z tilt_deg sigma_x_deg sigma_y_deg walls flats groups
observable usable walls_flats_deg flats_unconfirmed flats_contradicted``) for further processing.

``--input-tum`` levels a trajectory in TUM format with the same correction as the keyframes and saves it to
``--output-tum``: the single rotation, or with ``--piecewise`` the same interpolated up samples and re-integrated
positions. Use it for the dense path of the run (e.g. the odometry's per-scan trajectory); its timestamps must be
sorted and, with ``--piecewise``, in the same clock as the keyframes: a path whose time range does not overlap the
keyframes' is refused, and poses outside it (a warning tells how many) get the correction of the nearest window. With
``--piecewise`` the path is re-integrated on its own steps and anchored where it and the keyframes first both exist,
so there it moves as much as the keyframes; at the keyframe stamps it differs from the leveled keyframes only to second
order (the change of up between two keyframes times the excursion between them): 1-5 cm on real maps. It is also
written with ``--estimate-only``, which only skips the output simple-map. The file is read and checked (8 columns,
valid quaternions) before the map is loaded.

.. note::

   It assumes a mostly rectilinear structure with vertical walls and level floors (indoors, buildings).
   Do not use it on outdoor or sloped scenes.

|

sm-cli partition
----------------------
Splits the keyframes of a simple-map into submaps that observe the same place, whenever they were visited, for
``sm-cli level-walls --by-submap``.

.. code-block:: bash

    sm-cli partition <input.simplemap> <output_partition.yaml>
        [--pipeline <sm2mm-pipeline.yaml>]  # keyframes to points (default: built-in)
        [--sample-distance 2.0]             # distance between graph nodes [m]
        [--sample-angle-deg 30]             # rotation between graph nodes [deg]
        [--overlap-voxel 0.5]               # voxel size of the shared content [m]
        [--overlap-max-range 0]             # ignore points farther than this [m] (0: all)
        [--overlap-decimation 0.1]          # decimation of each keyframe first [m]
        [--max-ncut 0.2]                    # split only below this normalized cut
        [--min-submap-nodes 5]              # fewest nodes on each side of a split
        [--output-colored-map <out.mm>]     # points with a 'submap' field

How it works:

- Keyframes are sampled as graph nodes, one every ``--sample-distance`` or ``--sample-angle-deg``.
- Each keyframe becomes a set of voxels of ``--overlap-voxel`` in the map frame, each one tagged with the side of the
  surface seen from the sensor (the direction of its normal, oriented towards the sensor, or of the viewing ray where
  the points are not planar) and with whether it is above or below the sensor. Two keyframes then share a voxel only
  if they see the same side of the same surface: the two faces of a wall or of a floor slab are different content, and
  so are walls where they cross a slab, so storeys stay apart. The sensor is the one of each range observation (its
  ``sensorPose`` on the keyframe pose), not the keyframe frame, which may be at floor level. Poses are used as they
  are, with no registration.
- Two nodes are as similar as the voxels they share (Jaccard), all pairs at once through an inverted index of voxels.
- The graph is split by recursive normalized cuts (Shi and Malik): the Fiedler vector of each part, computed with
  Lanczos on the sparse graph, orders its nodes, and the part is cut where the normalized cut is lowest, if it is at
  most ``--max-ncut`` and both sides keep ``--min-submap-nodes`` nodes. Disconnected parts are always split.
- Every keyframe is linked to its most similar nodes, and joins the submap of the best one.

A higher ``--max-ncut`` (e.g. 0.5) splits the map into more, smaller submaps: ``level-walls --by-submap`` then follows
a tilt that changes faster from place to place, at the cost of noisier estimates per submap and of more floor
thickness where neighboring submaps disagree.

The output YAML holds, per node, its keyframe and submap; the edges between nodes with their similarity; and per
keyframe, its submap and links. The keyframe count must match the simple-map it is used with.

|

sm-cli tf
----------------------
Transforms a given simple-map by applying a SE(3) transformation by the left (=left-multiplying homogeneous matrices).

.. code-block:: bash

    sm-cli tf <input.simplemap> <output.simplemap> "[x y z yaw_deg pitch_deg roll_deg]"

|

sm-cli trim
----------------------
Extracts part of a simple-map, leaving only those key-frames that lie within a given bounding box.

.. code-block:: bash

    sm-cli trim <filename> --min-corner "[xmin ymin zmin]" --max-corner "[xmax ymax zmax]" --output <OUTPUT.simplemap>


|

sm-cli join
----------------------
Merges two or more simple-maps in one single map. No map alignment or registration is performed by this simple tool,
so the maps should be already aligned beforehand, or the resulting simple-map being the input to a loop-closure pipeline.

.. code-block:: bash

    sm-cli join <filename_1> [<filename_2> ...] --output <MERGED.simplemap>

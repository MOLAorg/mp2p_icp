.. _mp2p_icp_filters:

.. title is in mola/docs rst file

Point cloud **Filters** in ``mp2p_icp`` are used to modify or extract part of the information from an input point cloud.
This can involve tasks like removing noise, downsampling, segmenting points into different layers based on properties
(like intensity or curvature), or adjusting per-point attributes (like timestamps or intensity).
All filters inherit from :cpp:class:`mp2p_icp_filters::FilterBase` and can be configured either programmatically 
or via a YAML file using the **filter pipeline** API, e.g.
:cpp:class:`mp2p_icp_filters::filter_pipeline_from_yaml()` 
or :cpp:class:`mp2p_icp_filters::filter_pipeline_from_yaml_file()`.

____________________________________________

.. contents:: Available filters
   :depth: 1
   :local:
   :backlinks: none

____________________________________________

|
Filter: `FilterAdjustTimestamps`
--------------------------------

**Description**: Modifies the per-point timestamps of a map layer according to a set of criteria.
**Parameters**:

* **pointcloud\_layer** (:cpp:type:`std::string`): The name of the point cloud layer to process.
* **silently\_ignore\_no\_timestamps** (:cpp:type:`bool`, default: `false`): If `true`, no exception is thrown if the input layer does not contain timestamps;
an empty output is produced instead.
* **time\_offset** (:cpp:type:`double`, default: `0.0`): An optional time offset to be added on top of the adjustment.
Useful for synchronizing multiple sensors.
* **method** (:cpp:enum:`TimestampAdjustMethod`, default: `MiddleIsZero`): The criterion for adjusting timestamps:
    * **EarliestIsZero**: Adjusts such that the earliest timestamp is 0. Successive ones represent real elapsed seconds.
* **MiddleIsZero**: Adjusts such that the middle timestamp is 0. The rest are positive/negative elapsed seconds.
* **Normalize**: Normalizes all timestamps to the range :math:`[0, 1]`.
.. code-block:: yaml

    filters:
      #...
      - class_name: mp2p_icp_filters::FilterAdjustTimestamps
        params:
          pointcloud_layer: 'raw'
          silently_ignore_no_timestamps: true
          method: 'MiddleIsZero'

.. rubric:: Before → After Screenshot

.. image:: adjust_timestamps_example.png
   :alt: Screenshot showing point cloud before and after applying FilterAdjustTimestamps

|
---

Filter: `FilterBoundingBox`
---------------------------

**Description**: Splits a point cloud into points **inside** and **outside** a given 3D bounding box.
**Parameters**:

* **input\_pointcloud\_layer** (:cpp:type:`std::string`, default: `raw`): The input point cloud layer name.
* **inside\_pointcloud\_layer** (:cpp:type:`std::string`, optional): The output layer name for points **INSIDE** the bounding box. If empty, these points are discarded.
* **outside\_pointcloud\_layer** (:cpp:type:`std::string`, optional): The output layer name for points **OUTSIDE** the bounding box. If empty, these points are discarded.
* **bounding\_box\_min** (:cpp:type:`float[3]`): The :math:`(x, y, z)` coordinates of the minimum corner of the bounding box (e.g., `[-10, -10, -5]`).
Can use robocentric variables (e.g., `robot_x`).
* **bounding\_box\_max** (:cpp:type:`float[3]`): The :math:`(x, y, z)` coordinates of the maximum corner of the bounding box (e.g., `[10, 10, 5]`).
Can use robocentric variables.

.. code-block:: yaml

    filters:
      #...
      - class_name: mp2p_icp_filters::FilterBoundingBox
        params:
          input_pointcloud_layer: 'raw'
          inside_pointcloud_layer: 'close_points'
          bounding_box_min: [ -10, -10, -1 ]
          bounding_box_max: [ 10, 10, 5 ]

.. rubric:: Before → After Screenshot

.. image:: bounding_box_example.png
   :alt: Screenshot showing point cloud before 
and after applying FilterBoundingBox

|

---

Filter: `FilterByIntensity`
---------------------------

**Description**: Thresholds an input cloud by its intensity values, segmenting points into low, mid, and high intensity layers.
**Parameters**:

* **input\_pointcloud\_layer** (:cpp:type:`std::string`): The input point cloud layer name.
* **output\_layer\_low\_intensity** (:cpp:type:`std::string`, optional): Output layer for points with :math:`\text{intensity} < \text{low\_threshold}`.
* **output\_layer\_high\_intensity** (:cpp:type:`std::string`, optional): Output layer for points with :math:`\text{intensity} > \text{high\_threshold}`.
* **output\_layer\_mid\_intensity** (:cpp:type:`std::string`, optional): Output layer for points with :math:`\text{intensity} \in [\text{low\_threshold}, \text{high\_threshold}]`.
* **low\_threshold** (:cpp:type:`float`, default: `0.10`): The minimum intensity value for the 'mid' range.
* **high\_threshold** (:cpp:type:`float`, default: `0.90`): The maximum intensity value for the 'mid' range.
.. code-block:: yaml

    filters:
      #...
      - class_name: mp2p_icp_filters::FilterByIntensity
        params:
          input_pointcloud_layer: 'raw'
          output_layer_high_intensity: 'high_i'
          low_threshold: 0.1
          high_threshold: 0.9

.. rubric:: Before → After Screenshot

.. image:: by_intensity_example.png
   :alt: Screenshot showing point cloud before and after applying FilterByIntensity

|
---

Filter: `FilterByRange`
-----------------------

**Description**: Filters points based on their range (distance) from a specified center point (default is the origin :math:`(0, 0, 0)`).
**Parameters**:

* **input\_pointcloud\_layer** (:cpp:type:`std::string`, default: `raw`): The input point cloud layer name.
* **output\_layer\_between** (:cpp:type:`std::string`, optional): Output layer for points **within** the :math:`[\text{range\_min}, \text{range\_max}]` distance range.
* **output\_layer\_outside** (:cpp:type:`std::string`, optional): Output layer for points **outside** the :math:`[\text{range\_min}, \text{range\_max}]` distance range.
* **range\_min** (:cpp:type:`float`, default: `3.0`): The minimum distance threshold.
* **range\_max** (:cpp:type:`float`, default: `90.0`): The maximum distance threshold.
* **center** (:cpp:type:`float[3]`, default: `[0, 0, 0]`): The center point from which ranges are measured.
Can use robot pose variables (e.g., `robot_x`).
* **metric\_l\_infinity** (:cpp:type:`bool`, default: `false`): If `true`, the L-infinity norm (maximum component) is used for distance calculation, which is more efficient than the default L2 Euclidean norm.
.. code-block:: yaml

    filters:
      #...
      - class_name: mp2p_icp_filters::FilterByRange
        params:
          input_pointcloud_layer: 'raw'
          output_layer_between: 'valid_range'
          range_min: 3.0
          range_max: 90.0

.. rubric:: Before → After Screenshot

.. image:: by_range_example.png
   :alt: Screenshot showing point cloud before and after applying FilterByRange

|
---

Filter: `FilterByRing`
----------------------

**Description**: Keeps only a given subset of an input cloud based on the LiDAR "ring\_id" (assuming the point cloud has ring data).
**Parameters**:

* **input\_pointcloud\_layer** (:cpp:type:`std::string`): The input point cloud layer name.
* **output\_layer\_selected** (:cpp:type:`std::string`, optional): Output layer for points whose ring ID is in :cpp:member:`selected_ring_ids`.
* **output\_layer\_non\_selected** (:cpp:type:`std::string`, optional): Output layer for points whose ring ID is **NOT** in :cpp:member:`selected_ring_ids`.
* **selected\_ring\_ids** (:cpp:type:`std::set<int>`): A list of ring IDs to keep/select (e.g., `[0, 1, 5, 6]`).
.. code-block:: yaml

    filters:
      #...
      - class_name: mp2p_icp_filters::FilterByRing
        params:
          input_pointcloud_layer: 'raw'
          output_layer_selected: 'ground_rings'
          selected_ring_ids: [ 0, 1, 15, 16 ]

.. rubric:: Before → After Screenshot

.. image:: by_ring_example.png
   :alt: Screenshot showing point cloud before and after applying FilterByRing

|
---

Filter: `FilterCurvature`
-------------------------

**Description**: Classifies a **sorted** input point cloud (e.g., a single LiDAR scan line) by local curvature, estimated from the angle between a point and its immediate former and posterior neighbors.
Useful for edge extraction (LOAM-style).

**Parameters**:

* **input\_pointcloud\_layer** (:cpp:type:`std::string`, default: `raw`): The input point cloud layer name.
**Must be sorted for meaningful results.**
* **output\_layer\_larger\_curvature** (:cpp:type:`std::string`, optional): Output layer for points with larger curvature (often "edges").
* **output\_layer\_smaller\_curvature** (:cpp:type:`std::string`, optional): Output layer for points with smaller curvature (often "flatter" surfaces).
* **output\_layer\_other** (:cpp:type:`std::string`, optional): Output layer for points that do not fall into the above two categories.
* **max\_cosine** (:cpp:type:`float`, default: `0.5f`): A threshold related to the angle of the local neighborhood for classifying high curvature.
* **min\_clearance** (:cpp:type:`float`, default: `0.02f`): The minimum distance a neighbor must be to be considered in the curvature calculation (m).
* **max\_gap** (:cpp:type:`float`, default: `1.00f`): The maximum gap distance between a point and its neighbor (m).
.. code-block:: yaml

    filters:
      #...
      - class_name: mp2p_icp_filters::FilterCurvature
        params:
          input_pointcloud_layer: 'raw_sorted'
          output_layer_larger_curvature: 'edges'
          output_layer_smaller_curvature: 'planes'

.. rubric:: Before → After Screenshot

.. image:: curvature_example.png
   :alt: Screenshot showing point cloud before and after applying FilterCurvature

|
---

Filter: `FilterDecimateAdaptive`
--------------------------------

**Description**: Accepts an input point cloud, voxelizes it, and generates a new layer with an adaptive sampling to aim for a specific desired output point count.
**Parameters**:

* **input\_pointcloud\_layer** (:cpp:type:`std::string`, default: `raw`): The input point cloud layer name.
* **output\_pointcloud\_layer** (:cpp:type:`std::string`): The output layer name for the adaptively decimated cloud.
* **desired\_output\_point\_count** (:cpp:type:`unsigned int`, default: `1000`): The target number of points in the output cloud.
* **minimum\_input\_points\_per\_voxel** (:cpp:type:`unsigned int`, default: `1`): Voxels with fewer points than this threshold will not generate any output point.
* **voxel\_size** (:cpp:type:`float`, default: `0.10`): The size of the voxel grid used for downsampling (m).
* **parallelization\_grain\_size** (:cpp:type:`size\_t`, default: `16384`): Grain size for parallel processing of input clouds (used when TBB is enabled).
.. code-block:: yaml

    filters:
      #...
      - class_name: mp2p_icp_filters::FilterDecimateAdaptive
        params:
          input_pointcloud_layer: 'raw'
          output_pointcloud_layer: 'adaptively_decimated'
          desired_output_point_count: 5000
          voxel_size: 0.2

.. rubric:: Before → After Screenshot

.. image:: decimate_adaptive_example.png
   :alt: Screenshot showing point cloud before and after applying FilterDecimateAdaptive

|
---

Filter: `FilterDecimateVoxels`
------------------------------

**Description**: Builds a new layer with a decimated version of one or more input layers using a uniform voxel grid.
**Parameters**:

* **input\_pointcloud\_layer** (:cpp:type:`std::vector<std::string>`, default: `[raw]`): One or more input layers to read and merge points from.
* **error\_on\_missing\_input\_layer** (:cpp:type:`bool`, default: `true`): If `true`, an exception is thrown if an input layer is missing.
Otherwise, it's silently ignored.

* **output\_pointcloud\_layer** (:cpp:type:`std::string`): The output point cloud layer name.
New points are appended if the layer already exists.

* **voxel\_filter\_resolution** (:cpp:type:`float`, default: `1.0f`): Size of each voxel edge (m).
* **use\_tsl\_robin\_map** (:cpp:type:`bool`, default: `true`): Whether to use `tsl::robin_map` (faster for smaller clouds) or `std::map` (faster for large clouds) as the container implementation.
* **minimum\_input\_points\_to\_filter** (:cpp:type:`uint32\_t`, default: `0`): If the total number of input points is less than this, all points are passed through without decimation.
* **flatten\_to** (:cpp:type:`std::optional<double>`): If defined, the 3D points are "flattened" into a 2D planar cloud at a constant height :math:`z`.
Additional point fields (ring, intensity, timestamp) are **NOT** copied in this mode.
* **decimate\_method** (:cpp:enum:`DecimateMethod`, default: `FirstPoint`): The method to pick the representative point for each voxel:
    * **DecimateMethod::FirstPoint**: Picks the first point inserted into the voxel (the fastest method).
* **DecimateMethod::ClosestToAverage**: Picks the point closest to the average position of all voxel points.
* **DecimateMethod::VoxelAverage**: Calculates and uses the average position of all voxel points (a new point).
* **DecimateMethod::RandomPoint**: Picks one of the voxel points at random.
* **minimum_points_per_voxel** (:cpp:enum:`uint32_t`, default: `0`): Minimum number of points in each voxel to use that voxel output.
It applies to all `decimate_method` options, except `DecimateMethod::FirstPoint`.


.. code-block:: yaml

    filters:
      #...
      - class_name: mp2p_icp_filters::FilterDecimateVoxels
        params:
          input_pointcloud_layer: [ 'raw', 'intensity_low' ]
          output_pointcloud_layer: 'decimated'
          voxel_filter_resolution: 0.1
          decimate_method: 'DecimateMethod::VoxelAverage'

.. rubric:: Before → After Screenshot

.. image:: decimate_voxels_example.png
   :alt: Screenshot showing point cloud before and 
after applying FilterDecimateVoxels

|

---

Filter: `FilterDeleteLayer`
---------------------------

**Description**: Completely removes one or more point layers from the metric map.
**Parameters**:

* **pointcloud\_layer\_to\_remove** (:cpp:type:`std::vector<std::string>`): One or more layers to remove.
* **error\_on\_missing\_input\_layer** (:cpp:type:`bool`, default: `true`): If `true`, an exception is thrown if a layer to be removed does not exist.
Otherwise, it's silently ignored.

.. code-block:: yaml

    filters:
      #...
      - class_name: mp2p_icp_filters::FilterDeleteLayer
        params:
          pointcloud_layer_to_remove: [ 'raw', 'temp_layer' ]
          error_on_missing_input_layer: false

.. rubric:: Before → After Screenshot

.. image:: delete_layer_example.png
   :alt: Screenshot showing point cloud before and after applying FilterDeleteLayer

|
---

Filter: `FilterDeskew`
----------------------

**Description**: Deskew (motion compensate) a point cloud from a moving LiDAR.
**Parameters**:

* **input\_pointcloud\_layer** (:cpp:type:`std::string`, default: `raw`):  
  The input point cloud layer name.
* **output\_pointcloud\_layer** (:cpp:type:`std::string`, optional):  
  The output layer name. Required unless `in_place` is `true`.
* **in\_place** (:cpp:type:`bool`, default: `false`):  
  If `true`, the deskewed points replace the input layer (most efficient).
* **output\_layer\_class** (:cpp:type:`std::string`, default: `mrpt::maps::CPointsMapXYZI`):  
  The class name for the output layer if it needs to be created.
* **silently\_ignore\_no\_timestamps** (:cpp:type:`bool`, default: `false`):  
  If `true`, no exception is thrown if the input layer lacks timestamps.
* **method** (:cpp:enum:`MotionCompensationMethod`, default: `Linear`):  
  The motion compensation method used to interpolate or integrate point positions:
  
  * **`MotionCompensationMethod::None`** – No compensation;
all points are assumed to be acquired at the same vehicle pose.
* **`MotionCompensationMethod::Linear`** – Constant velocity (linear and angular) model using the provided `twist`.
* **`MotionCompensationMethod::IMU`** – Integration using IMU data with constant linear acceleration and angular velocity.
* **`MotionCompensationMethod::IMUh`** – Higher-order IMU integration assuming constant jerk and angular acceleration.
* **`MotionCompensationMethod::IMUt`** – Trapezoidal IMU integration using constant linear acceleration and angular velocity.
* **twist** (:cpp:type:`std::optional<mrpt::math::TTwist3D>`):  
  The velocity (linear and angular) of the vehicle in the local frame.
Only used when `method=Linear`.  
May be dynamically bound via the `mp2p_icp::Parameterizable` API.
* **points\_already\_global** (:cpp:type:`bool`, default: `false`):  
  If `true`, the input points are already expressed in global coordinates (e.g. in sm2mm pipelines).
When enabled, `robot_pose` must be provided to reference the global transformation.
* **robot\_pose** (:cpp:type:`mrpt::math::TPose3D`, optional):  
  The robot’s pose in global coordinates.  
Only used when `points_already_global=true`.
* **bias\_acc** (:cpp:type:`mrpt::math::TVector3D`, default: `[0, 0, 0]`):  
  Accelerometer bias in **sensor frame** coordinates.
Used for IMU-based motion compensation methods.

* **bias\_gyro** (:cpp:type:`mrpt::math::TVector3D`, default: `[0, 0, 0]`):  
  Gyroscope bias in **sensor frame** coordinates.
* **gravity\_vector** (:cpp:type:`mrpt::math::TVector3D`, default: `[0, 0, -9.81]`):  
  Gravity vector in **global frame** coordinates.
Used when integrating IMU data to maintain alignment with world gravity.
---

**YAML Example**

.. code-block:: yaml

    filters:
      # ...
      - class_name: mp2p_icp_filters::FilterDeskew
        params:
          input_pointcloud_layer: 'raw'
          output_pointcloud_layer: 'deskewed'
          method: 'MotionCompensationMethod::Linear'   # Normally, use `IMU` if you have an IMU, or `Linear` otherwise
          twist: [vx, vy, vz, wx, wy, wz]
          #bias_acc: [0.00, 0.0, 0.0]
          #bias_gyro: [0.0, 0.0, 0.0]
          #gravity_vector: [0, 0, -9.81]
          # IMPORTANT: In the context of sm2mm, set this to 'true' since 
          # de-skew happens with points already in global coordinates
          # so we need the robot pose to correct points:
         
    #points_already_global: true
          robot_pose:
            [robot_x, robot_y, robot_z, robot_yaw, robot_pitch, robot_roll]

---

**Notes**:

* The input layer must contain timestamps (`mrpt::maps::CPointsMapXYZIRT`).
* Timestamps are assumed to be in seconds relative to a reference time (e.g., scan start).
Use `FilterAdjustTimestamps` to adjust this if necessary.
* If timestamps are missing and `silently_ignore_no_timestamps=false`, an exception is thrown.
---

.. rubric:: Before → After Screenshot

.. image:: deskew_example.png
   :alt: Screenshot showing point cloud before and after applying FilterDeskew

|
---

Filter: `FilterFartherPointSampling`
------------------------------------

**Description**: Subsamples a cloud using the **Farther Point Sampling (FPS)** algorithm, aiming for a desired number of output points.
**Parameters**:

* **input\_pointcloud\_layer** (:cpp:type:`std::string`, default: `raw`): The input point cloud layer name.
* **output\_pointcloud\_layer** (:cpp:type:`std::string`): The output layer name for the sampled cloud.
* **desired\_output\_point\_count** (:cpp:type:`unsigned int`, default: `1000`): The target number of points in the output cloud.
.. code-block:: yaml

    filters:
      #...
      - class_name: mp2p_icp_filters::FilterFartherPointSampling
        params:
          input_pointcloud_layer: 'raw'
          output_pointcloud_layer: 'sampled'
          desired_output_point_count: 2000

.. rubric:: Before → After Screenshot

.. image:: farther_point_sampling_example.png
   :alt: Screenshot showing point cloud before and after applying FilterFartherPointSampling

|
---

Filter: `FilterMerge`
---------------------

**Description**: Takes a point cloud layer (or a :cpp:class:`mrpt::maps::CVoxelMap` layer) and inserts it into another existing layer of arbitrary metric map type.
This uses the target layer's standard `insertObservation()` method.

**Parameters**:

* **input\_pointcloud\_layer** (:cpp:type:`std::string`): The point cloud or map layer to be inserted.
* **target\_layer** (:cpp:type:`std::string`): The destination layer into which the points will be merged.
* **input\_layer\_in\_local\_coordinates** (:cpp:type:`bool`, default: `false`): If `true`, the `input\_pointcloud\_layer` is assumed to be in the vehicle frame and is transformed by `robot\_pose` before insertion.
* **robot\_pose** (:cpp:type:`mrpt::math::TPose3D`): The pose of the robot/vehicle. Required if `input\_layer\_in\_local\_coordinates` is `true`.
.. code-block:: yaml

    filters:
      #...
      - class_name: mp2p_icp_filters::FilterMerge
        params:
          input_pointcloud_layer: 'local_scan'
          target_layer: 'global_map'
          input_layer_in_local_coordinates: true

.. rubric:: Before → After Screenshot

.. image:: merge_example.png
   :alt: Screenshot showing point cloud before and after applying FilterMerge

|
---

Filter: `FilterMLS`
-------------------

**Description**: Applies a **Moving Least Squares (MLS)** surface reconstruction filter to a point cloud for smoothing, normal estimation, and optional upsampling, mimicking the functionality of PCL's :cpp:class:`pcl::MovingLeastSquares`. The filter computes a weighted polynomial surface in the local neighborhood of each point.

The computed normals are stored as new per-point fields in the output map: ``normal_x``, ``normal_y``, and ``normal_z``.

**Parameters**:

* **input\_pointcloud\_layer** (:cpp:type:`std::string`, default: `raw`): The point cloud layer used to **build** the MLS surface (i.e., the source of the local neighborhoods).
* **output\_pointcloud\_layer** (:cpp:type:`std::string`, default: `mls`): The layer where the smoothed/projected points and their computed normals will be stored.
* **distinct\_cloud\_layer** (:cpp:type:`std::string`, optional): Required if :cpp:member:`upsampling_method` is set to ``DISTINCT_CLOUD``. Points from this layer are projected onto the surface computed from :cpp:member:`input_pointcloud\_layer`.
* **search\_radius** (:cpp:type:`double`, default: `0.05`): The search radius (in meters) for finding neighbors to fit the MLS surface.
* **polynomial\_order** (:cpp:type:`int`, default: `2`): The order of the polynomial to fit (e.g., ``1`` for planar, ``2`` for quadratic).
* **min\_neighbors\_for\_fit** (:cpp:type:`int`, default: `3`): The minimum number of neighbors required within the search radius to successfully compute the MLS fit.
* **projection\_method** (:cpp:enum:`ProjectionMethod`, default: `SIMPLE`): The method used for projecting points onto the fitted surface:
    * **SIMPLE**: Projects the point along the normal direction onto the polynomial surface.
* **upsampling\_method** (:cpp:enum:`UpsamplingMethod`, default: `NONE`): The method to determine which points are processed:
    * **NONE**: The points from :cpp:member:`input_pointcloud\_layer` are smoothed/projected.
    * **DISTINCT\_CLOUD**: The points from :cpp:member:`distinct\_cloud\_layer` are projected onto the surface built from :cpp:member:`input\_pointcloud\_layer`.
* **parallelization\_grain\_size** (:cpp:type:`size\_t`, default: `1024`): Grain size for parallel processing of query points when TBB is enabled.

.. code-block:: yaml

    filters:
      #...
      - class_name: mp2p_icp_filters::FilterMLS
        params:
          # The layer containing the point cloud used to compute the MLS surface.
          # This is the 'source' cloud for the surface fitting.
          input_pointcloud_layer: "raw"

          # The layer where the smoothed/projected points and their normals will be
          # stored. If it doesn't exist, it will be created.
          output_pointcloud_layer: "mls"

          # ====================================================================
          # MLS Core Parameters
          # ====================================================================

          # Search radius (in meters) for finding neighbors around each query point.
          # A larger radius results in a smoother but potentially less detailed surface.
          search_radius: 0.10

          # Order of the polynomial to fit to the local neighborhood.
          # 1: Planar fit (fastest, basically a weighted PCA for normals).
          # 2: Quadratic fit (standard for curvature/better smoothing).
          polynomial_order: 2

          # Minimum number of neighbors required to successfully compute a fit.
          min_neighbors_for_fit: 10

          # ====================================================================
          # Smoothing / Projection Method
          # ====================================================================

          # Method for projecting points onto the fitted polynomial surface.
          # SIMPLE: Project point along the normal direction to the polynomial at
          #         the projected (u,v) location. (Fast and robust).
          projection_method: SIMPLE

          # Method for determining which points to smooth/project.
          # NONE: Smooth the 'input_pointcloud_layer' itself (standard smoothing).
          # DISTINCT_CLOUD: Build the surface from 'input_pointcloud_layer', but
          #                 project the points from 'distinct_cloud_layer' onto it
          #                 (used for upsampling or projection).
          upsampling_method: NONE

          # Required only if upsampling_method is DISTINCT_CLOUD.
          # distinct_cloud_layer: "sparse_scan"

          # ====================================================================
          # Parallelization
          # ====================================================================

          # If TBB is enabled, this is the grain size for parallel execution.
          parallelization_grain_size: 1024

.. rubric:: Before → After Screenshot

.. image:: mls_example.png
   :alt: Screenshot showing point cloud before and after applying FilterMLS

|
---

Filter: `FilterNormalizeIntensity`
----------------------------------

**Description**: Normalizes the intensity channel of a point cloud layer such that intensity values end up in the range :math:`[0, 1]`.
The data is updated **in-place** in the input/output layer.

**Parameters**:

* **pointcloud\_layer** (:cpp:type:`std::string`): The point cloud layer to process.
* **remember\_intensity\_range** (:cpp:type:`bool`, default: `false`): If `true`, the filter keeps an internal record of the minimum and maximum intensities observed in past clouds to maintain a consistent normalization.
* **fixed\_maximum\_intensity** (:cpp:type:`double`, default: `0.0`): If non-zero, this value is used as the fixed maximum intensity for normalization.
* **fixed\_minimum\_intensity** (:cpp:type:`double`, default: `0.0`): If `fixed\_maximum\_intensity` is non-zero, this value is used as the fixed minimum intensity for normalization.
.. code-block:: yaml

    filters:
      #...
      - class_name: mp2p_icp_filters::FilterNormalizeIntensity
        params:
          pointcloud_layer: 'raw'
          remember_intensity_range: true

.. rubric:: Before → After Screenshot

.. image:: normalize_intensity_example.png
   :alt: Screenshot showing point cloud before and after applying FilterNormalizeIntensity

|
---

Filter: `FilterPoleDetector`
----------------------------

**Description**: Identifies and separates points that appear to belong to a pole or vertical structure.
This is done by analyzing the min/max Z-span in 2D grid cells.
**Parameters**:

* **input\_pointcloud\_layer** (:cpp:type:`std::string`, default: `raw`): The input point cloud layer name.
* **output\_layer\_poles** (:cpp:type:`std::string`, optional): Output layer name for points that **are** poles.
* **output\_layer\_no\_poles** (:cpp:type:`std::string`, optional): Output layer name for points that **are not** poles.
* **grid\_size** (:cpp:type:`float`, default: `2.0f`): Size of the 2D grid cell used for analysis (m).
* **minimum\_relative\_height** (:cpp:type:`float`, default: `2.5f`): Minimum required height span in a cell to be considered a pole candidate.
* **maximum\_relative\_height** (:cpp:type:`float`, default: `25.0f`): Maximum allowed height span in a cell.
* **minimum\_points\_per\_cell** (:cpp:type:`uint32\_t`, default: `50`): Minimum number of points required in a cell for analysis.
.. code-block:: yaml

    filters:
      #...
      - class_name: mp2p_icp_filters::FilterPoleDetector
        params:
          input_pointcloud_layer: 'raw'
          output_layer_poles: 'poles'
          grid_size: 1.0
          minimum_relative_height: 3.0

.. rubric:: Before → After Screenshot

.. image:: pole_detector_example.png
   :alt: Screenshot showing point cloud before and after applying FilterPoleDetector

|
---

Filter: `FilterRemoveByVoxelOccupancy`
--------------------------------------

**Description**: Removes points from an input point cloud based on the occupancy status of the corresponding voxels in a separate :cpp:class:`mrpt::maps::CVoxelMap` layer.
This is typically used to separate **static** (high occupancy) and **dynamic** (low occupancy) objects.
**Parameters**:

* **input\_pointcloud\_layer** (:cpp:type:`std::string`): The input point cloud to be filtered.
* **input\_voxel\_layer** (:cpp:type:`std::string`): The layer containing the occupancy data (:cpp:class:`mrpt::maps::CVoxelMap`).
* **output\_layer\_static\_objects** (:cpp:type:`std::string`, optional): Output layer for points within high-occupancy voxels ("static objects").
* **output\_layer\_dynamic\_objects** (:cpp:type:`std::string`, optional): Output layer for points within low-occupancy voxels ("dynamic objects").
* **occupancy\_threshold** (:cpp:type:`float`, default: `0.6f`): The occupancy probability threshold. Voxels above this are considered "static".
.. code-block:: yaml

    filters:
      #...
      - class_name: mp2p_icp_filters::FilterRemoveByVoxelOccupancy
        params:
          input_pointcloud_layer: 'raw'
          input_voxel_layer: 'voxel_map'
          output_layer_static_objects: 'static'
          occupancy_threshold: 0.7

.. rubric:: Before → After Screenshot

.. image:: remove_by_voxel_occupancy_example.png
   :alt: Screenshot showing point cloud before and after applying FilterRemoveByVoxelOccupancy

|
---

Filter: `FilterVoxelSlice`
--------------------------

**Description**: Takes an input layer of type :cpp:class:`mrpt::maps::CVoxelMap` (e.g., Bonxai) and extracts a single 2D slice at a specified Z-range, collapsing the voxel column into an occupancy gridmap.
**Parameters**:

* **input\_voxel\_layer** (:cpp:type:`std::string`): The input voxel map layer (:cpp:class:`mrpt::maps::CVoxelMap`).
* **output\_gridmap\_layer** (:cpp:type:`std::string`): The output 2D occupancy grid map layer.
* **z\_min** (:cpp:type:`double`): The minimum Z-coordinate for the slice.
* **z\_max** (:cpp:type:`double`): The maximum Z-coordinate for the slice.
.. code-block:: yaml

    filters:
      #...
      - class_name: mp2p_icp_filters::FilterVoxelSlice
        params:
          input_voxel_layer: 'voxel_map'
          output_gridmap_layer: '2d_slice'
          z_min: -0.5
          z_max: 0.5

.. rubric:: Before → After Screenshot

.. image:: voxel_slice_example.png
   :alt: Screenshot showing point cloud before and after applying FilterVoxelSlice
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mp2p_icp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2024-02-16)
------------------
* Add new apps: sm-cli, mm-info, txt2mm, mm2txt, mm-filter
* Improved documentation.
* new filter FilterByIntensity
* FilterNormalizeIntensity: add option for intensity range memory
* FilterByRange: renamed params to simplify them (removed param 'keep_between')
* FIX: missing intensity channel in decimate voxel when using some decimation methods
* sm-cli: new subcommand 'level' to maximize the 'horizontality' of built maps
* add optional profiler to filter pipelines
* Contributors: Jose Luis Blanco-Claraco

1.1.1 (2024-02-07)
------------------
* MergeFilter: now also handles CVoxelMap as inputs
* more memory efficient defaults
* FilterCurvature: now based on ring_id channel
* Use hash map min_factor to speed up clear()s
* add missing hash reserve
* PointCloudToVoxelGridSingle: Fix wrong initialization of point count
* Contributors: Jose Luis Blanco-Claraco

1.1.0 (2024-01-25)
------------------
* FilterDecimateVoxels: Replace 3 bool parameters with an enum
* Fix clang warnings
* Save and visualize ICP step partial solutions
* QualityEvaluator_PairedRatio: now does not require parameters
* Add filter: Bonxai VoxelMap -> 2D gridmap. Bayesian filtering of voxel columns
* Generator: allow defining custom metric maps directly in the YAML configuration
* Contributors: Jose Luis Blanco-Claraco

1.0.0 (2024-01-20)
------------------
* Gauss-Newton solver: Add optional prior term
* Added FilterMerge and modifications to allow sm2mm to build any type maps
* sm2mm: add option for lazy-load external directory
* Decimate filter: add flatten_to option to efficiently convert 3D->2D point clouds
* FilterBoundingBox: parameter name changed for clearer split of inside / outside bbox
* Deskew: add option to bypass de-skew operation
* bump minimum required mrpt version
* Better coloring; add option to export mm layers
* Use new mrpt api to propagate point properties; add final_filter stage to sm2mm
* sm2mm: add verbosity flag
* bbox filter: allow processing variables too
* Introduce robot\_{x,y,z} variables
* Better mm-viewer; update sm2mm demo file
* Progress with RST docs
* Add missing robotPose argument to generators; progress with mm-viewer
* Add sm2mm app
* Add FILE attribute to license tag
* More dynamic parameters
* fix print format
* Add Deskew filter
* update CI to u22.04
* Introduce Parameterizable interface
* New layers: create of the same input cloud type
* Add FilterCurvature
* filter: optional additional layer for deleted points
* FIX: important error in robust gradient
* expose GN params as public
* new generators and filters
* Filters: use tsl robin_map, faster than std::unordered_map
* prefer nn_radius_search() to exploit nanoflann rknn
* Minor UI updates
* gui: autoplay
* estimate_points_eigen.h moved to the mp2p_icp_map library
* Solvers: add option to select by correction magnitude
* add [[nodiscard]] to generator API
* Add specialized implementation of voxelize for 1 pt/vx
* add Cauchy robust kernel
* Add support for TBB for parallelization
* add angularThresholdFactor; add max plane-to-pt distance
* viewer UI: show number of points per layer
* Prefer Teschner's spatial hash
* Use nn_single_search() when possible
* viewer: add follow local checkbox
* Add new filter: FilterDecimateVoxelsQuadratic
* FilterDecimateVoxels: new option use_closest_to_voxel_average
* FilterDecimateVoxels: new param use_random_point_within_voxel
* less unnecesary mem allocs
* generator: create map layers first, then filter by observation name/class filter
* port to NN radius search
* add "enabled" property to base Matcher class
* Solvers: add property 'enabled'
* Add robust kernels to GN solver
* Add optional profiler to ICP
* New parameter decimationDebugFiles
* Add plugin option to viewer
* VoxelFilter: is now ~7 times faster and does not need a bounding box parameter, thanks to using an associative container.
* viewer: add new flag -f to load one single log file
* viewer: increase slider range for max far plane
* Options to recolorize maps in icp log viewer
* Fix regression in rendering options for point clouds
* Matcher: new parameter bounding_box_intersection_check_epsilon
* New env var MP2P_ICP_GENERATE_DEBUG_FILES can be use to override generation of icp log files
* BUGFIX: Ignored sensorPose for Generator::filterPointCloud()
* Allow ICP matching against voxel metric map types
* mp2p_icp_filters::Generator now can create a map from a generic INI file (e.g. voxelmaps)
* fix references to old `pointcloud_t` -> `metric_map_t`
* Remove support for MRPT<2.4.0
* Contributors: Jose Luis Blanco-Claraco

0.2.2 (2023-09-08)
------------------
* Fix missing cmake dependencies between libraries
* Update mola_common
* Refactor into a new small library mp2p_icp_map with just the metric_map_t class
* sync mola_common submodule
* Update submodule mola_common
* Remove redundant section
* Update ROS badges
* Contributors: Jose Luis Blanco-Claraco

0.2.1 (2023-09-02)
------------------

* Update copyright date
* Update to new name of mola_common
* update ros badges
* Contributors: Jose Luis Blanco-Claraco

0.2.0 (2023-08-24)
------------------
* First release as MOLA submodule.

0.1.0 (2023-06-14)
------------------
* First official release of the mp2p_icp libraries
* Contributors: FranciscoJManasAlvarez, Jose Luis Blanco-Claraco

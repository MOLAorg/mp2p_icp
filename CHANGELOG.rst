^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mp2p_icp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* metric map data type: add new metadata YAML field
* Update broken link to ROS Index
* docs: change references to default branch master->develop
* Default generator: more details in debug traces when ignoring an observation
* Update package license tag to "BSD-3-Clause"
* Integrate vscode with colcon custom settings and clang-tidy
* Fix build unit tests with older gcc versions
* Drop apparently useless build dep
* Contributors: Jose Luis Blanco-Claraco

1.6.7 (2025-04-03)
------------------
* mm-georef cli app: support reading/writing georef info in YAML format
* georeferencing metadata now can be read/writen as YAML files
* clang-format: switch to column limit=100
* Update to robin-map v1.4.0
* Contributors: Jose Luis Blanco-Claraco

1.6.6 (2025-02-26)
------------------
* Docs: add page for mm-georef
* docs: Update 2025 paper citation
* print metric_map_t as string: show lat/lon coordinates in a format directly compatible with Google Map searches.
* New cli tool: mm-georef, to manipulate the geo-referencing metadata of metric map files
* Contributors: Jose Luis Blanco-Claraco

1.6.5 (2025-01-28)
------------------
* Add GitHub actions
* Add pole-detector filter
* mm-filter app: add --load-plugins flag too
* Add sanity check assert in FilterDeskew
* Contributors: Jose Luis Blanco-Claraco

1.6.4 (2024-12-18)
------------------
* merge two docs pages in one to shorten the docs TOC
* Update README.md: Mark ROS2 Iron as EOL
* Also use TBB for parallel solving point-to-plane pairings
* Contributors: Jose Luis Blanco-Claraco

1.6.3 (2024-11-11)
------------------
* icp-log-viewer: also reduce GUI refresh rate
* mm-viewer: avoid useless GUI refresh (CPU usage reduction)
* txt2mm: Add input filter xyzrgb
* mm-viewer: add a 'fit view to map' button
* New cli app rawlog-filter
* FilterCurvature: better handling scans with <=3 points in some rings
* new subcommand 'sm-cli tf'
* Contributors: Jose Luis Blanco-Claraco

1.6.2 (2024-09-14)
------------------
* Expose << and >> operators for geo-reference data structures
* Fix missing build_dep
* Contributors: Jose Luis Blanco-Claraco

1.6.1 (2024-09-11)
------------------
* Fix missing catkin buildtoo_depend for ROS1 builds
* Update RTTI macros for upcoming MRPT 2.14.0
* Contributors: Jose Luis Blanco-Claraco

1.6.0 (2024-09-08)
------------------
* Port Point2Plane matcher to use the new NN-for-planes API
* mp2p_icp_map library: add NearestPlaneCapable virtual API
* cmake: move from glob expressions to explicit lists of source files
* clarify eigenvalues order in headers
* Contributors: Jose Luis Blanco-Claraco

1.5.6 (2024-09-07)
------------------
* sm2mm cli: show map contents before writing to disk
* add another demo sm2mm file for the mola tutorials
* Add another sm2mm demo file w/o deskew for the mola mapping tutorial
* Matcher_Point2Plane: fix build error in armhf
* Fix build with embedded mola_common
* README: Add ROS badges for all architectures
* Contributors: Jose Luis Blanco-Claraco

1.5.5 (2024-08-27)
------------------
* Explicitly add tbb as dependency in package.xml
* Depend on new mrpt_lib packages (deprecate mrpt2)
* FIX: build errors in armhf arch
* Contributors: Jose Luis Blanco-Claraco

1.5.4 (2024-08-20)
------------------
* Do not use Eigen::Vector for compatibility with Eigen3 <3.4 in ROS Noetic
* Contributors: Jose Luis Blanco-Claraco

1.5.3 (2024-08-20)
------------------
* Re-add ROS1 Noetic as supported distribution
* Generator sanity check asserts: more informative error messages
* sm-cli: new command 'join' to merge simplemaps
* icp-log-viewer UI: new keybind 'I' to switch initial/final pose
* icp-log-viewer UI: add option to visualize voxelmaps empty space
* Contributors: Jose Luis Blanco-Claraco

1.5.2 (2024-07-24)
------------------
* Add sm2mm yaml example for dynamic/static obstacles
* Update sample sm2mm pipelines to use de-skew
* docs: add mm-filter example
* Fix pointcloud ptr typo
* More safety sanity checks added in mm-viewer and sm2mm
* BUGFIX: Generator should not create empty maps for GPS observations
* Contributors: Jose Luis Blanco-Claraco, Raúl Aguilera López

1.5.1 (2024-07-03)
------------------
* Update docs
* ICP: Add optional functors for before-logging maps
* icp-log-viewer UI: fix potential out-of-range exception when autoplay is on
* FilterAdjustTimestamps: add new param 'time_offset' useful for multiple LiDARs setups
* Contributors: Jose Luis Blanco-Claraco

1.5.0 (2024-06-21)
------------------
* ICP: Add optional user-provided per-iteration hooks
* Add new filter: FilterByRing
* Add new filter: FilterAdjustTimestamps
* Add sanity checks for point cloud fields.
* Fix typo in default class for FilterDeskew
* generators API: add bool return type to detect if observation was actually processed
* generic Generator: handle velodyne observations so timestamps are generated
* Contributors: Jose Luis Blanco-Claraco

1.4.3 (2024-06-11)
------------------
* Add pointcloud_sanity_check() auxiliary function
* Generator: more DEBUG level traces
* BUGFIX: FilterDeskew generated buggy output points if the input does not contain timestamps
* Add sanity checks for point cloud fields
* ICP log records now also store the dynamic variables. icp-log-viewer displays them.
* ICP log files: automatically create output directory if it does not exist
* Update ros2 badges (added Jazzy)
* Contributors: Jose Luis Blanco-Claraco

1.4.2 (2024-05-28)
------------------
* mm-viewer: add check-all, check-none to layer filters
* Add new filter: FilterRemoveByVoxelOccupancy
* mm-viewer: camera travelling keyframes-based animations
* mm-viewer: navigate the map with keyboard arrows; add a load button
* mm-viewer: can now also draws a TUM trajectory overlaid with the map
* UI apps: smoother rendering
* icp-log-viewer and mm-viewer: the UI now has a XYZ corner overlay
* sm-cli: command "export-kfs" now has an optional flag '--output-twist'
* FilterDeskew: ignore empty input maps
* More debug-level traces
* deskew filter: Fix case of variable names in docs
* sm-cli app: Add new command 'trim' to cut simplemaps by bounding box
* mm-viewer: show mouse pointing coordinates
* Contributors: Jose Luis Blanco-Claraco

1.4.1 (2024-05-19)
------------------
* Fix build for older mrpt versions
* ICP pipelines: Implement loading ``quality_checkpoints`` parameter from YAML config file
* Quality evaluators: add the option for 'hard discard'
* Update QualityEvaluator_Voxels to use prebuilt voxel layers from input maps. Add unit tests.
* BUGFIX: Fix deserializing georeferenced .mm files stored in <1.4.0 format
* ICP: quality evaluators can now have formulas in their parameters too
* mm-viewer and icp-log-viewer: extend zoom range so maps of tens of kms can be viewed at once
* Contributors: Jose Luis Blanco-Claraco

1.4.0 (2024-05-06)
------------------
* Update commit for robin-map to latest version (patch contributed upstream)
* icp-log-viewer: UI now has a slider for each map point size
* ICP: Add a new quality_checkpoint parameter to early abort ICP attempts
* georeferenced maps: T_enu_to_map now has a covariance field
* mm-viewer: display ENU frame too
* Contributors: Jose Luis Blanco-Claraco

1.3.3 (2024-04-30)
------------------
* Add minimum_input_points_to_filter option to FilterDecimateVoxels
* FIX: QualityEvaluator_PairedRatio throws when one of the reference maps is empty
* FIX BUG: Won't try to match 2D pointclouds if their height is different
* Clarify comments in metricmap.h about geodetic references
* Fix printing metric_map_t contents when it only has a gridmap
* Fix potential dangling references (g++ 13 warning)
* Fix potential use of uninitialized point index
* Bump cmake_minimum_required to 3.5
* Contributors: Jose Luis Blanco-Claraco

1.3.2 (2024-04-22)
------------------
* tsl::robin_map library is no longer exposed neither in the public API nor as public headers (PIMPL pattern)
  This is to prevent Debian-level collisions with other packages also exposing it.
* add first icp-log-viewer docs
* Contributors: Jose Luis Blanco-Claraco

1.3.1 (2024-04-16)
------------------
* mm-viewer and icp-log-viewer: saves UI state in persistent user config file
* FIX: missing UI refresh when clicking showPairings checkbox
* renamed apps for less verbose names: icp-run, icp-log-viewer
* ICP core now defines a variable ICP_ITERATION for use in programmable formulas in pipelines
* icp-log-viewer: much faster rendering of ICP iteration details
* mm-viewer: fix bug in calculation of bounding box
* Merge docs with main MOLA repo
* Contributors: Jose Luis Blanco-Claraco

1.3.0 (2024-03-10)
------------------
* mm-viewer: new options to visualize georeferenced maps
* New sm-cli commands: --cut, --export-keyframes, --export-rawlog
* propagate cmake deps downstream
* metric_map_t: add georeferencing optional field
* mm-filter: add --rename operation
* GetOrCreatePointLayer() moved to its own header and uses shared ptrs
* FilterMerge: add param input_layer_in_local_coordinates
* Contributors: Jose Luis Blanco-Claraco

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

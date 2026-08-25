^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mp2p_icp_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.13.1 (2026-08-25)
-------------------
* Merge pull request `#94 <https://github.com/MOLAorg/mp2p_icp/issues/94>`_ from MOLAorg/feat/mm-filter-generators-create-layer
  mm-filter: create new metric map layers via generators:
* Fix stale doc URLs missing the mp2p_icp_core path segment
  These sm2mm/mm-filter demo comments and the sm2mm.h docstring still
  pointed at pre-split paths (apps/sm2mm, apps/mm-filter, top-level
  demos/), left over from the mp2p_icp_core/mp2p_icp_viz package split.
  Same class of issue as flagged in review for the new demo added in
  this branch; fixing the other pre-existing occurrences found by a
  repo-wide search.
* Address review: fix demo URL, doc precedence, and unresolved formula check
  - Fix stale doc link in the new demo (missing mp2p_icp_core path segment).
  - CreateMetricMapFromDefinition(): docs said "exactly one" of the two
  definition sources must be given, but the implementation actually
  prefers metricMapDefinitionIniFile if both are non-empty; reword to
  match.
  - Generator::createTargetLayerIfNeeded(): call
  Parameterizable::checkAllParametersAreRealized() before creating the
  map, matching the guard Generator::process() already has. Without it,
  a metric_map_definition formula (e.g. "$f{...}") referencing a
  variable that's never bound in this code path (createTargetLayerIfNeeded
  has no attached ParameterSource) would silently serialize as its
  unresolved 0.0 placeholder instead of raising a clear error.
* Let mm-filter pipelines create new metric map layers via generators:
  mm-filter operates on an already-built .mm file, so it had no way to
  instantiate a brand-new layer of an arbitrary mrpt::maps::CMetricMap
  subclass (e.g. mola::KeyframePointCloudMap): that logic lived inside
  Generator, and only ran as a side effect of processing an observation,
  which mm-filter does not have.
  Factor the map-creation logic (TSetOfMetricMapInitializers +
  CMultiMetricMap, driven by a metric_map_definition YAML block) out of
  Generator::implProcessCustomMap() into a standalone
  mp2p_icp_filters::CreateMetricMapFromDefinition(), and expose it via a
  new Generator::createTargetLayerIfNeeded(), callable without any
  observation at hand.
  mm-filter now optionally parses a generators: section (same schema as
  sm2mm) and uses it only to pre-create each generator's target_layer if
  missing, before running filters:. A plain FilterMerge step can then
  insert an existing point cloud layer into it, closing the gap reported
  in `MOLAorg/mola#147 <https://github.com/MOLAorg/mola/issues/147>`_.
* Add feature-detection macro for nn_visit_pt2pl_candidates()
  Downstream code (e.g. mola) needs to build against both this and
  older mp2p_icp releases still distributed via ROS binary repos, which
  predate NearestPlaneCapable::nn_visit_pt2pl_candidates() and the
  PlaneCandidate/plane_candidate_visitor_t types.
* Contributors: Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco

2.13.0 (2026-08-21)
-------------------
* Merge pull request `#91 <https://github.com/MOLAorg/mp2p_icp/issues/91>`_ from MOLAorg/feat/ndt-blend-matcher
  Add Matcher_NDT_Blend: a smooth alternative to the nearest-plane argmin
* Restore include ordering in the matcher registry
* Add Matcher_Points_Blend and Matcher_Points_KnnPlane
  Two more correspondence rules, at opposite ends of the same axis, so that the
  question "does the discreteness of the correspondence rule drive the
  sensitivity" can be measured rather than argued.
  Matcher_Points_Blend is the point-based counterpart of Matcher_NDT_Blend: the
  target is the distance-weighted mean of the map points in a radius instead of
  the nearest one, so it moves continuously as the query crosses the
  perpendicular bisector between two map points. temperature 0 runs the very same
  nn_single_search() query as Matcher_Points_DistanceThreshold with
  pairingsPerPoint 1, so it reproduces it exactly. The neighborhood is a radius
  query and never a fixed-k one: a point entering or leaving a top-k list is a
  harder flip than the one being removed. Its intrinsic cost is that a weighted
  mean of surface points is pulled toward the interior of the neighborhood, so
  the temperature has to be chosen against the map's point spacing.
  Matcher_Points_KnnPlane goes the other way: k nearest map points, a
  least-squares plane through them, and three hard gates. Every gate is a
  constant or a function of a static property of the measurement, so nothing in
  it reads back the registration's own quality. The plane is fitted by solving
  A x = -1 with a column-pivoting Householder QR in single precision rather than
  by an eigen fit, because the two are not numerically the same on
  near-degenerate neighborhoods and this matcher exists to reproduce a protocol
  faithfully. One consequence worth knowing: that form cannot represent a plane
  through the origin.
  Two hazards found while writing them, both silent:
  - nn_radius_search() applies a small hard cap on the number of results whenever
  maxPoints is nonzero, ranking candidates and keeping the best few. Only
  maxPoints 0 keeps a radius query a radius query, so that is what the blend
  passes, and the parameter that could change it is documented as such.
  - A k-NN query asked for more neighbors than the map holds can still return
  exactly k, padding the tail by repeating an earlier point under a fabricated
  zero distance. The "reject if fewer than k were found" gate therefore cannot
  be implemented as a size check; each neighbor is validated against its own
  coordinates instead.
  Matcher_NDT_Blend gains optional weight-distribution statistics, gated on
  MP2P_ICP_BLEND_STATS, reporting how many candidates a query really combines,
  and now rejects negative temperature and searchRadius rather than silently
  falling back to the unblended path.
* Formatting, and keep the zero-temperature bounding-box test identical too
  At temperature 0 the search radius now reverts to distanceThreshold, so the
  bounding-box early-return matches Matcher_Point2Plane as well, not just the
  per-point query. Behavior is unchanged wherever the boxes overlap, which is
  every case exercised so far, but the control is now exact by construction.
* Add Matcher_NDT_Blend: a smooth alternative to the nearest-plane argmin
  Matcher_Point2Plane resolves its candidate planes with a hard argmin, so an
  arbitrarily small pose change can swap the winner and move the residual by the
  full disagreement between the two candidates. Matcher_NDT_Blend keeps the same
  inputs and emits the same one point-to-plane pairing per local point, but takes
  a likelihood-weighted combination of the candidates instead of the best one, so
  the residual varies continuously with the pose. Solvers are unchanged.
  Candidates are read through a new NearestPlaneCapable::nn_visit_pt2pl_candidates(),
  whose default implementation reports only the best match, so maps that expose
  just an argmin keep working.
  Three details carry the behavior:
  - temperature 0 takes the plain nn_search_pt2pl() path and therefore reproduces
  Matcher_Point2Plane exactly, which makes it a usable control.
  - The weight fades to zero with zero derivative at searchRadius, so a candidate
  entering or leaving the window does so continuously; a hard cutoff would
  reintroduce the discontinuity at the window edge instead.
  - Normals are combined through their outer products rather than as vectors. The
  point-to-plane cost is invariant to a plane's normal sign, and this keeps the
  blend invariant too; a vector average would instead depend on each map cell's
  own sign bookkeeping. The cost is that two exactly perpendicular candidates
  carrying exactly equal weight switch rather than interpolate, which no single
  plane could represent anyway.
  Defaults reproduce the previous behavior, so no shipped pipeline changes.
* Merge pull request `#90 <https://github.com/MOLAorg/mp2p_icp/issues/90>`_ from MOLAorg/feat/decimate-adaptive-dynamic-budget
  FilterDecimateAdaptive: dynamic point budgets, and a bound on the voxel stride
* Clamp the stride-derived budget at the input point count
  An arbitrarily small maximum_voxel_stride made ceil(nVoxels/stride) exceed
  uint32_t, and the narrowing conversion is undefined there. The input is the
  real ceiling anyway, since no decimation method can emit more points than it
  was given. Strides below 1 stay legal: the methods that take several points
  out of one voxel can honor them.
* FilterDecimateAdaptive: dynamic point budgets, and a bound on the voxel stride
  The point budget and the voxel size were loaded with MCP_LOAD\_*, which parses
  a YAML scalar up to its first non-numeric character and keeps the truncated
  value without complaint. A budget written as an expression therefore became
  something else entirely, silently. Both are now declared parameters, so they
  accept formulas over the pipeline's variables like the range and map-size
  parameters already do.
  The sampling walk uses a stride of nVoxels/desired_output_point_count and
  stops as soon as the count is reached, so a stride above 1 means only one in
  every `stride` occupied cells is ever visited. An absolute point count thus
  expresses very different sampling densities depending on how many occupied
  cells the input happens to have, which is a property of the scene, the sensor
  and the voxel size rather than of the configuration. The new optional
  `maximum_voxel_stride` bounds that ratio directly. It only ever raises the
  effective count, so the absolute budget keeps acting as a floor and leaving it
  unset preserves today's behavior exactly.
  Also: size the `outputs` vector up front. Declaring a dynamic parameter stores
  a pointer to its target field, and growing the vector afterwards would dangle
  the ones already declared.
  A DEBUG line now reports nVoxels, the effective budget, the stride and the
  emitted count per output, so a configuration can be checked against its actual
  input instead of assumed.
* Merge pull request `#89 <https://github.com/MOLAorg/mp2p_icp/issues/89>`_ from MOLAorg/feat/cov2cov-ambiguity-gating
  Matcher_Cov2Cov: optional range-adaptive matching distance and ambiguity gate
* Merge branch 'develop' into feat/cov2cov-ambiguity-gating
* Remove the ambiguity gate from MatchingDistanceProfile
  firstToSecondDistanceMin/firstToSecondMinRange were not yet justified by
  results (regressed translation error ~62% on a held-out sequence relative to
  the range-adaptive distance alone). Drop the fields, Matcher_Cov2Cov params,
  NearestPointWithCovCapable's guard against them, and the corresponding test
  coverage, keeping only the range-adaptive matching distance.
* Keep nn_search_cov2cov() source-compatible with older map implementations
  The flat-threshold overload goes back to being the pure virtual, and the
  MatchingDistanceProfile one becomes a non-pure overload whose default
  implementation forwards to it. A map class written against an earlier release
  therefore keeps compiling, overriding and behaving exactly as before, which is
  what the downstream CI jobs need: they resolve mola_metric_maps to the last
  released binary package, which is older than this source tree.
  The forwarding default refuses, rather than silently ignores, a profile such an
  implementation cannot honor: a range-adaptive distance or an active ambiguity
  gate throws instead of quietly degrading to a flat threshold.
  Adds MP2P_ICP_HAS_MATCHING_DISTANCE_PROFILE so downstream packages can detect
  the new API. Since MatchingDistanceProfile.h is an entirely new header, a plain
  __has_include() settles it for consumers; the macro is what survives if the
  struct later grows members.
  Also documents the name-hiding consequence: a derived class declaring only one
  of the two overloads hides the other for calls on that derived type. Harmless
  for the matchers, which always dispatch through a base reference.
* Matcher_Cov2Cov: optional range-adaptive matching distance and ambiguity gate
  Introduce mp2p_icp::MatchingDistanceProfile, the acceptance criteria used by
  NearestPointWithCovCapable::nn_search_cov2cov(). It replaces the flat
  `float max_search_distance` argument, and is implicitly constructible from a
  float, so every existing caller keeps compiling and behaving as before.
  Two opt-in refinements over a single flat distance, both disabled by default:
  - Range-adaptive distance: `thresholdFar`, `thresholdKneeRange` and
  `thresholdTransitionWidth` turn the acceptance distance into a logistic
  function of the query point's range from the sensor. Map point density falls
  off with range, so one flat threshold is loose near the sensor and tight far
  away.
  - Ambiguity test: `firstToSecondDistanceMin` rejects a correspondence whose
  runner-up candidate is within that ratio of the winner's distance, i.e. the
  match is too close to call. `firstToSecondMinRange` restricts the test to
  beyond a given range, leaving the dense near field, where the runner-up is
  the same surface seen again, untouched.
  All five are declared with DECLARE_PARAMETER_OPT so they accept dynamic
  formulas like "3.0*ADAPTIVE_THRESHOLD_SIGMA", exactly as `threshold` does. A
  static YAML load would convert such a string by stopping at the first
  non-numeric character and silently yield a fixed value in meters.
  No shipped pipeline enables either refinement; defaults reproduce the previous
  behavior bit for bit, including the k=1 KD-tree query and no per-point range
  computation.
* Merge pull request `#88 <https://github.com/MOLAorg/mp2p_icp/issues/88>`_ from MOLAorg/fix/matcher-param-formula-parsing
  fix: load matcher/filter distance parameters as dynamic formulas
* fix: load matcher/filter distance parameters as dynamic formulas
  Several numeric parameters that pipelines legitimately want to express as
  formulas were read with MCP_LOAD_REQ/MCP_LOAD_OPT, a static YAML load.
  mrpt::containers::yaml::as<double>() truncates a string at the first
  non-numeric character, so "2.0*ADAPTIVE_THRESHOLD_SIGMA" was silently
  turned into the constant 2.0, with no warning and no error, and was never
  re-evaluated afterwards. Since ADAPTIVE_THRESHOLD_SIGMA is the output of
  the LO adaptive-threshold controller, the affected parameters were left
  frozen at a value that is not even in the right units.
  Switched to DECLARE_PARAMETER\_*, the same macros the equivalent parameters
  of the other matchers already use:
  * Matcher_Point2Line::distanceThreshold. Same name and same role as
  Matcher_Point2Plane::distanceThreshold, which is dynamic and is set to
  "1.0*ADAPTIVE_THRESHOLD_SIGMA" in the shipped lidar3d-ndt pipeline.
  * Matcher_Adaptive::absoluteMaxSearchDistance, minimumCorrDist and
  planeMinimumDistance: all distances in meters gating pairing acceptance.
  * FilterCurvature::max_cosine, min_clearance and max_gap, which the
  sibling GeneratorEdgesFromCurvature already declares dynamically.
  Also added checkAllParametersAreRealized() to Matcher_Point2Line,
  Matcher_Adaptive and FilterCurvature, so an unrealized formula now fails
  loudly instead of running with a stale value.
  Tests: test-mp2p_matcher_pt2pt_parameterizable now covers Point2Plane,
  Point2Line and Adaptive; test-mp2p_matcher_pt2ln gets an end-to-end check
  that the formula actually gates pairings across two realize() calls; new
  test-mp2p_FilterCurvature. All of them fail against the previous code.
* Merge pull request `#87 <https://github.com/MOLAorg/mp2p_icp/issues/87>`_ from MOLAorg/perf/decimate-adaptive-flat-voxel-index-array
  Store voxel point indices in one flat array (recovers the FilterDecimateAdaptive regression)
* Store voxel point indices in one flat array
  PointCloudToVoxelGrid kept a std::vector per voxel, that is, one heap
  allocation per voxel, rebuilt from scratch for every processed cloud.
  The point indices now live in a single contiguous array, with each voxel
  holding just an offset and a count into it, and are stored as 32 bit to
  halve the memory traffic.
  FilterDecimateAdaptive also merges all of its per-thread grids in one
  single call, so that the flat array is laid out once instead of once per
  merged grid. This matters because the thread-local grids accumulate over
  successive runs, and most of them are empty on any given one.
  The output is unchanged: voxel contents and their canonical ordering are
  the same as before.
* Merge per-thread voxel grids in FilterDecimateAdaptive (`#86 <https://github.com/MOLAorg/mp2p_icp/issues/86>`_)
  * Merge per-thread voxel grids in FilterDecimateAdaptive
  Under TBB the input cloud is binned in parallel into one grid per
  thread, but the voxel key is global, so a single spatial voxel was left
  split into one fragment per thread that saw points in it. The fragments
  were then visited as if they were independent voxels.
  That had three consequences:
  - The output depended on how TBB happened to split and steal work, so
  two identical runs gave different decimated clouds.
  - Decimation emitted up to one representative per thread per voxel
  instead of one per voxel, so the output was denser and less uniform
  than requested.
  - minimum_input_points_per_voxel was applied per fragment, so a voxel
  legitimately above the threshold was dropped whenever its points were
  split across threads. For a rotating lidar, whose consecutive point
  indices sweep azimuth, this affected most voxels.
  Reassemble the per-thread grids by voxel key before sampling, into an
  owning grid, which keeps voxel_t a plain non-owning span. Voxel point
  indices are then sorted so the contents no longer depend on the binning
  order.
  The voxel list is also given a canonical order, by the first input
  point of each voxel, in both the parallel and the sequential paths.
  The resampling walks this list with a stride, so which voxels reach the
  output would otherwise depend on hash map traversal order, which is an
  implementation detail of the map type.
  Adds regression tests over a cloud that revisits voxels at distant
  input indices, as a rotating lidar does. All three checks fail before
  this change.
  * Apply clang-format
  * Test mergeFrom() directly, independently of worker count
  The end-to-end fragmentation checks only exercise the merge when TBB
  actually runs the blocks concurrently, so on a low-core machine they
  would pass without touching it. Bin a cloud in three unequal pieces,
  merge, and assert the result equals binning it in one pass.
  * Require a matching map type in mergeFrom(), and cover both
  The four-way dispatch handled combinations that cannot occur: only
  FilterDecimateAdaptive merges grids, and it always uses robin maps.
  Assert the two agree and keep the two reachable branches.
  The merge test now runs over both backing map types and asserts they
  produce the same voxel contents, which is what the canonical voxel
  ordering is built from.
* update robin-map to latest version (avoid cmake deprecation)
* Merge pull request `#85 <https://github.com/MOLAorg/mp2p_icp/issues/85>`_ from MOLAorg/fix/decimate-voxels-maybe-uninitialized-warning
  fix: spurious -Wmaybe-uninitialized in FilterDecimateVoxels::initialize_filter
* fix: spurious -Wmaybe-uninitialized in FilterDecimateVoxels::initialize_filter
  Both optional grid members were unconditionally .reset() before
  .emplace()-ing only one of them, so the one about to be (re)constructed
  was torn down twice in quick succession (once explicitly, once inside
  emplace()). GCC 15's flow analysis loses track of the pimpl's
  function-pointer deleter across that redundant double-teardown and
  warns on an unrelated unique_ptr destructor.
  Only reset the *other* optional, which was the only thing the explicit
  reset() was ever needed for. Verified by compiling the function in
  isolation before/after: warning count goes from 1 to 0, no new
  warnings, and the pre-existing test suite (112/112) is unaffected.
* Merge pull request `#83 <https://github.com/MOLAorg/mp2p_icp/issues/83>`_ from MOLAorg/feat/decimate-adaptive-multi-output
  FilterDecimateAdaptive: several output layers from one voxelization pass, plus decimate_method
* FilterDecimateAdaptive: several output layers from one voxelization pass, plus decimate_method
  Two independent additions to FilterDecimateAdaptive:
  1) An optional "outputs" YAML sequence, each entry holding its own
  output_pointcloud_layer + desired_output_point_count. All of them are
  sampled from ONE voxelization pass (the dominant cost), each walking the
  shared voxel list with its own stride. The single-output keys keep working
  exactly as before, and are mutually exclusive with "outputs".
  This lets a consumer obtain a dense cloud for its local map and a sparse one
  for ICP without paying for two full passes. Chaining two filters also
  computed the second cloud from already-decimated voxel statistics rather
  than from the full scan.
  Measured on test-mp2p_gicp_pipeline_benchmark (new env var
  MP2P_BENCH_SINGLE_DECIMATION=1 selects the single-pass variant there),
  1st-pass filtering drops from ~15.0 ms to ~13.0 ms per 100k-point scan,
  with unchanged ICP quality and recovered pose.
  2) Support for decimate_method, as in FilterDecimateVoxels. The DecimateMethod
  enum moves to its own header, mp2p_icp_filters/DecimateMethod.h, now shared
  by both filters (FilterDecimateVoxels.h includes it, so its users are
  unaffected).
  Since this filter revisits voxels in as many rounds as needed to reach the
  requested point count, the methods behave differently here:
  - FirstPoint (default, previous behavior) and RandomPoint take successive
  points out of each voxel, in insertion order or from a random per-voxel
  offset.
  - ClosestToAverage and VoxelAverage summarize the whole voxel and hence emit
  one point per voxel only, so the output cannot exceed the number of valid
  voxels. VoxelAverage synthesizes points, so per-point fields are not
  propagated. Their per-voxel results are precomputed once and reused by all
  output targets.
  Unit tests cover the multiple outputs, the mutually-exclusive parameters and
  the four decimation methods.
* Merge pull request `#82 <https://github.com/MOLAorg/mp2p_icp/issues/82>`_ from MOLAorg/feat/filter-transform-pointcloud
  Add FilterTransformPointCloud: rigidly transform a point-cloud layer by a pose or its inverse
* test: cover view-direction field rotation in FilterTransformPointCloud
  Addresses CodeRabbit nitpick: the existing tests only exercised XYZ
  transform, never the rotateViewDirectionFields() path the filter also
  runs. Mirrors the fixture pattern already used in
  test-mp2p_FilterMerge_view.cpp.
* fix: apply clang-format-14 to test-mp2p_FilterTransformPointCloud.cpp
  CI's pinned clang-format-14 wants different alignment on the pose
  assignment line than my local (newer) clang-format produced.
* Add FilterTransformPointCloud: rigidly transform a point-cloud layer by a pose or its inverse
  Some odometry systems (e.g. DLIO) voxelize incoming scans after transforming
  them into the map/world frame, so voxel membership is anchored to the map
  rather than to the vehicle's instantaneous local frame. No existing filter
  in this library exposes that as a composable building block: FilterMerge
  only inserts local points into an existing target map (one direction, and
  tied to insertObservation()), so there was no way to get a plain, freshly
  transformed point-cloud layer usable as input to another filter (e.g.
  FilterDecimateAdaptive), nor any way to transform back by a pose's inverse.
  FilterTransformPointCloud fills that gap: input layer -> output layer,
  replacing each point p_i by pose (+) p_i (or pose^-1 (+) p_i if invert_pose
  is set), reusing CPointsMap::insertAnotherMap() plus the same
  rotateViewDirectionFields() call FilterMerge already relies on to keep
  view-direction fields consistent.
* fix format
* Add robust_max_range(): percentile-based, outlier-robust point cloud extent
  A plain bounding-box max norm is dominated by a single spurious far
  return. Observed on Livox sensors (TIERS dataset): specular-reflection
  artifacts hundreds of meters away in an otherwise <20 m scene inflate
  any consumer's observation-radius estimate 10-40x, which then starves
  a downstream range filter or voxel grid of the real, near-field points.
  robust_max_range() returns a percentile of the per-point range
  (default 0.95) instead of the raw maximum, computed with
  std::nth_element (O(n) average, no full sort). Not wired into any
  consumer yet; that is a separate follow-up.
* Merge pull request `#80 <https://github.com/MOLAorg/mp2p_icp/issues/80>`_ from MOLAorg/fix/viz-skip-non-finite-points
  fix: skip non-finite points when rendering a point map layer
* fix: also drop non-finite points on the map's own-renderer viz path
  get_visualization_map_layer() returns early when the layer is not a
  CPointsMap, or when keep_original_cloud_color asks for the map's own colors:
  both delegate to map->getVisualizationInto(). That return happens before the
  source-map filtering added previously, so a map holding blank (NaN) storage
  slots could still hang the renderer through this path.
  Filtering the source beforehand is not possible here: the map draws itself,
  so there is no input cloud, and in the non-CPointsMap case there is nothing
  to filter either. Swapping in a filtered copy would discard the custom
  renderer that this path exists to use. The non-finite points are therefore
  dropped from the already-built render objects, in place, in the same loops
  that already walk them to apply the point size. Each surviving point keeps
  its own color, since colors are per point in the render object.
* fix: skip non-finite points when rendering a point map layer
  get_visualization_map_layer() copied the raw point buffers of a map layer
  into a CPointCloud / CPointCloudColoured verbatim. Map classes that recycle
  storage slots expose slots holding no point: mola::IncrementalPointCloud
  blanks them to NaN, precisely so that generic code walking the inherited
  CPointsMap buffers does not resurrect evicted geometry.
  A single non-finite coordinate is enough to hang the GUI thread: MRPT's LOD
  octree splits a node at the mean of its points, so the mean becomes NaN,
  every comparison against it is false, all points land in the same child and
  the recursive split repeats identically forever. Each level costs two passes
  over the whole cloud, so it presents as a frozen viewer rather than a crash.
  It only triggers once a cloud exceeds OCTREE_RENDER_MAX_POINTS_PER_NODE
  (1e6 by default), which is why it appears at one reproducible point of a
  mapping run.
  The source map is filtered, not the render object, because recolorize3Dpc()
  skips colorization unless both hold the same number of points. Maps whose
  points are all finite (the usual case) are passed through untouched, with no
  copy.
* Merge pull request `#78 <https://github.com/MOLAorg/mp2p_icp/issues/78>`_ from MOLAorg/feature/serialize-gravity-prior-in-icplog
  Record the GravityPrior in .icplog files
* Record the GravityPrior in .icplog files
  mp2p_icp::GravityPrior was passed to ICP::align() but never serialized, so the
  verticality observation the solver actually received could not be recovered
  from a log afterwards. That matters because its sigma is not necessarily the
  configured one: callers may widen it at run time (mola_lidar_odometry's
  adaptive_sigma does), and the only way to see the effective value was to re-run
  the whole dataset with debug-level logging.
  - Add CArchive operators for GravityPrior, plus DECLARE_TTYPENAME_CLASSNAME,
  which std::optional serialization requires.
  - Add LogRecord::gravityPrior, bumping the serialization version 2 -> 3. Older
  logs still load: the new field is read only when version >= 3.
  - Populate it in ICP::align(), next to the existing motion prior.
* fix: remove racy per-point pt2pt weight path in Gauss-Newton solver (`#77 <https://github.com/MOLAorg/mp2p_icp/issues/77>`_)
  * fix: remove racy per-point pt2pt weight path in Gauss-Newton solver
  Pairings::point_weights fed a sequential run-length-encoded cursor that
  optimal_tf_gauss_newton.cpp consumed from inside a tbb::parallel_reduce
  lambda captured by reference. Concurrent threads processing different
  point sub-ranges raced on the same shared cursor state, and the cursor
  was never reset across inner Gauss-Newton iterations either, making the
  per-layer `weight:` in pointLayerMatches silently unreliable in TBB
  builds. visit_correspondences.h had the same cursor pattern, though it
  was safe there since it only ever runs sequentially.
  The per-layer weighting granularity was never exercised with differing
  weights in any real pipeline (every pointLayerMatches block in-tree
  uses a single layer pair), so instead of making the cursor
  thread-safe, this drops per-point/per-layer pt2pt weighting entirely.
  All pt2pt pairs now use the single PairWeights::pt2pt scalar, matching
  every other pairing type. Matcher_Points_Base keeps pointLayerMatches
  for selecting which local/global layers to pair, just without a
  per-entry weight.
  Pairings serialization bumped to v3; older archives are still read
  correctly (the removed field is skipped).
  * docs: fix stale pointLayerMatches doc comment in Matcher_Points_Base
  Flagged by CodeRabbit on `#77 <https://github.com/MOLAorg/mp2p_icp/issues/77>`_: initialize()'s docstring still described
  pointLayerMatches as relative weights after the per-layer weighting
  removal; the class member's own docstring had already been updated.
* feat: yaw-free rank-2 gravity prior for the Gauss-Newton solver (`#76 <https://github.com/MOLAorg/mp2p_icp/issues/76>`_)
  Adds an optional gravity ("verticality") observation, independent of the
  existing SE(3) `prior`, that constrains only the two tilt DOFs and leaves
  rotation about gravity and all three translations exactly free.
  Residual: r(R) = B^T (R u_body) in R^2, with B an orthonormal basis of the
  plane orthogonal to `up_map`, weighted isotropically by 1/sigma^2. The
  isotropy is what makes the cost invariant to rotation about gravity: the
  induced 6x6 information has rank 2, a zero translation block, and null
  space exactly span(u_body), so yaw stays free at any attitude.
  This is the correct way to express a verticality constraint, versus
  encoding tilt into a 6x6 SE(3) prior information matrix, whose diagonal
  only isolates roll/pitch near yaw=0 and inevitably couples into
  translation.
  Purely additive and inert unless a caller sets it. Also documents that the
  `prior` information matrix is expressed in the SE(3) Lie tangent, not
  MRPT's (x,y,z,yaw,pitch,roll) Euler ordering; the two swap indices 3 and 5.
  New unit test with 4 cases: yaw recovery to <5e-7 deg at yaw in
  {0,45,90,170,-120}; leveling of a tilted solution; zero translation
  injected by a consistent prior; convergence with a non-level map frame.
* Merge pull request `#75 <https://github.com/MOLAorg/mp2p_icp/issues/75>`_ from MOLAorg/feat/expose-cov2cov-alpha
  Expose cov2cov_alpha and cov2cov_auto_balance_with_prior in Solver_Ga…
* Expose cov2cov_alpha and cov2cov_auto_balance_with_prior in Solver_GaussNewton
  These OptimalTF_GN_Parameters knobs were hard-coded to their struct defaults
  (1.0 and true). Load them via MCP_LOAD_OPT and forward them to gnParams so a
  pipeline YAML can scale the cov-to-cov data block against the pose prior (e.g.
  to let an IMU gravity pitch/roll prior carry more relative weight).
* sm-cli info: report the first keyframe's pose
  Useful to recover the absolute orientation a map was actually built with,
  which is not always the one it was seeded with: MOLA-LO's IMU-based initial
  leveling overwrites the configured pitch/roll at mapping time, so the only
  reliable source is the stored keyframe itself.
* fix: dont throw if saving icplogs to cwd
* fix(sm-cli): restore brackets stripped by CLI11 from tf pose argument
  CLI11 unconditionally strips a leading/trailing bracket pair from
  variadic option values, so the "[x y z yaw pitch roll]" pose string
  passed to `sm-cli tf` loses its brackets before CPose3D::fromString()
  sees it, which then throws since it requires them.
* cmake: add git submodule early fail
* Contributors: Jose Luis Blanco-Claraco

2.12.0 (2026-07-10)
-------------------
* add missing changelogs
* Merge pull request `#74 <https://github.com/MOLAorg/mp2p_icp/issues/74>`_ from MOLAorg/split-core-viz-packages
  Split repo into mp2p_icp_core (headless) + mp2p_icp_viz (GUI) + mp2p_icp (metapackage)
* fix: apply clang-format-14 to TCLAP->CLI11 converted files
  Address CI failure: several files from the TCLAP-to-CLI11 conversion
  weren't run through clang-format-14 before the previous commit.
* Split repo into mp2p_icp_core (headless) + mp2p_icp_viz (GUI) + mp2p_icp (metapackage)
  Consumers that only need the headless C++ libraries and CLI apps no
  longer need to pull in mrpt_libgui (nanogui/GLFW/X11) or build the GUI
  apps. mp2p_icp remains as a backward-compatible metapackage depending
  on both, so existing <depend>mp2p_icp</depend> consumers are unaffected.
  - mp2p_icp_core: mp2p_icp_common/map/filters + the ICP algorithms lib +
  13 headless CLI apps. Root find_package(MRPT COMPONENTS ...) drops
  "gui"; mrpt::opengl/system/expr remain available transitively via
  maps->obs and tfest->poses->bayes->config.
  - mp2p_icp_viz: mm-viewer and icp-log-viewer, the only two apps that
  actually use mrpt::gui (sm-cli's dependency on it was vestigial and
  dropped instead).
  - Removed the one real GUI leak in the mp2p_icp (ICP) library: an
  interactive CDisplayWindow debug feature in
  QualityEvaluator_RangeImageSimilarity (debug_show_all_in_window),
  which was the only reason that library linked mrpt::gui at all. The
  headless debug_save_all_matrices option remains.
  - Dropped the mola_common git submodule and its standalone-build
  bootstrap logic; standalone (non-colcon) plain-CMake builds are no
  longer supported, matching every other MOLAorg package in this
  ecosystem (find_package(mola_common REQUIRED) via the ROS workspace).
  robin-map stays vendored (no rosdep key, private dep of filters).
  - Replaced TCLAP with CLI11 for all CLI apps' argument parsing
  (rosdep key "cli11", same pattern already used elsewhere in
  MOLAorg repos). Behavior-preserving: same flags, defaults, and help
  text everywhere.
  - Removed .circleci/config.yml: its bare-cmake + PPA-installed
  libmrpt-dev jobs tested exactly the standalone-build path being
  dropped, and can't provide mola_common without reintroducing the
  bootstrap complexity just removed from CMakeLists. The colcon-based
  GH Actions jobs (humble/jazzy) are now the CI.
  - Updated docs (agents.md, README.md, docs/source/index.rst) and
  check-clang-format.yml/formatter.sh for the new layout.
  Verified: mp2p_icp_core, mp2p_icp_viz, mp2p_icp all build via colcon;
  all 104 mp2p_icp_core tests pass; mp2p_icp_core binaries have zero
  GUI/nanogui/GLFW linkage; mm-viewer still loads and renders a real
  georeferenced .mm file; a real downstream consumer (mola_metric_maps)
  builds unmodified against the new layout.
  Note: mp2p_icp/CMakeLists.txt shows as "modified" rather than
  deleted+added in the diff -- this is a git rename-detection artifact
  from the old mp2p_icp/ (ICP library) subfolder and the new mp2p_icp/
  (metapackage) folder sharing the same repo-relative path after the
  library moved one level deeper into mp2p_icp_core/mp2p_icp/.
* Contributors: Jose Luis Blanco-Claraco

2.11.0 (2026-07-04)
-------------------
* Merge pull request `#72 <https://github.com/MOLAorg/mp2p_icp/issues/72>`_ from MOLAorg/fix/filterdeskew-graceful-imu-anchor
  fix(FilterDeskew): never let IMU trajectory errors crash the pipeline
* Merge pull request `#71 <https://github.com/MOLAorg/mp2p_icp/issues/71>`_ from MOLAorg/feat/prior-referenced-robust-kernel
  Prior-referenced robust kernel for the Gauss-Newton solver
* Merge pull request `#70 <https://github.com/MOLAorg/mp2p_icp/issues/70>`_ from MOLAorg/fix/filtermerge-view-vector-rotation
  Fix view-direction vector frame mismatch in FilterMerge
* chore: demo sm2mm files updated for LIO localization maps
* demos: add sm2mm_pointcloud_voxelize_merged_keyframe_map.yaml
* Merge pull request `#69 <https://github.com/MOLAorg/mp2p_icp/issues/69>`_ from MOLAorg/feat/filter-decimate-range-adaptive
  feat: FilterDecimateRangeAdaptive (EllipseLIO range-adaptive scan filter)
* feat: add FilterBase::enabled for env-based pipeline toggling
  Adds an `enabled` boolean field to FilterBase (default: true)
* feat: add FilterDecimateRangeAdaptive (EllipseLIO range-adaptive scan filter)
* Fix build status badge for ROS 2 Lyrical arm64
* docs: add ROS 2 Lyrical badge row, update Rolling to Ubuntu 26.04 (resolute)
* Merge pull request `#68 <https://github.com/MOLAorg/mp2p_icp/issues/68>`_ from MOLAorg/feature/filter-polygon-2d
* Contributors: Jose Luis Blanco-Claraco

2.10.3 (2026-05-24)
-------------------
* Merge pull request `#67 <https://github.com/MOLAorg/mp2p_icp/issues/67>`_ from MOLAorg/fix/deskew-empty-trajectory
  FIX: robust against temporary lack of IMU in Deskew
* Merge pull request `#66 <https://github.com/MOLAorg/mp2p_icp/issues/66>`_ from MOLAorg/add-gicp-benchmark
  test: add new end-to-end gicp test as benchmark
* test: add new end-to-end gicp test as benchmark
* fix: icp-log-viewer bug in translations if view prior was enabled
* Contributors: Jose Luis Blanco-Claraco

2.10.2 (2026-05-11)
-------------------
* Merge pull request `#65 <https://github.com/MOLAorg/mp2p_icp/issues/65>`_ from MOLAorg/simplify-ci
  CI: simplify CI scripts and docker install
* chore: don't use anymore map classes deprecated and to be removed in mrpt 3.0.0
* fix: maps creating multiple CPointsCloud won't have all with pointSize honored
* Merge pull request `#64 <https://github.com/MOLAorg/mp2p_icp/issues/64>`_ from MOLAorg/bump-cmake
  bump min req cmake version to 3.22
* CI: Use sensible names for jobs matrix
* bump min req cmake version to 3.22
* Contributors: Jose Luis Blanco-Claraco

2.10.1 (2026-05-04)
-------------------
* FIX: sm2mm pipeline for keyframe maps need valid KF poses
* FIX: copy/paste error in guard against missing layer
* mm-viewer: UI now has an easier near/far clipping plane tool
* chore: map contents as string made less verbose (hide full covariance)
* Merge pull request `#63 <https://github.com/MOLAorg/mp2p_icp/issues/63>`_ from MOLAorg/feat/optional-final-run-quality-matchers
  feat: optional last run of matchers with final ICP_ITERATION for adaptive thresholds to see final thresholds
* feat: optional last run of matchers with final ICP_ITERATION for adaptive thresholds to see final thresholds
* fix: wrong decimation applied in sm2mm filters
* Merge pull request `#62 <https://github.com/MOLAorg/mp2p_icp/issues/62>`_ from MOLAorg/feat/prior-weight-mitigations
  feat: Implement Birge-ratio auto-balance for cov2cov and prior weighting
* feat: Implement Birge-ratio auto-balance for cov2cov and prior weighting
* fix: icp-log-viewer didn't show the prior covariance at its correct location
* Contributors: Jose Luis Blanco-Claraco

2.10.0 (2026-05-02)
-------------------
* CI: Update actions for new ROS rolling
* icp-log-viewer: better formatting of uncertainties
* demo sm2mm file: store as independent keyframes
* Merge pull request `#60 <https://github.com/MOLAorg/mp2p_icp/issues/60>`_ from MOLAorg/feat/censi3d-covariance
  Feat: Censi3D covariance method
* feat: Add new covariance method (Censi, 3D version)
* demo sm2mm files: add Keyframe map variant
* Contributors: Jose Luis Blanco-Claraco

2.9.1 (2026-04-29)
------------------
* Merge pull request `#59 <https://github.com/MOLAorg/mp2p_icp/issues/59>`_ from MOLAorg/fix/cov2cov-covariance-whitening
  Fix cov2cov whitening and add residual-variance scaling in covariance()
* Fix cov2cov whitening and add residual-variance scaling in covariance()
  The cov2cov branch in covariance.cpp whitened residuals with the full
  information matrix (cov_inv * e), so the assembled Hessian became
  J^T * cov_inv^2 * J instead of J^T * cov_inv * J. Combined with hundreds
  of pairings this drove det(cov) down to ~1e-20 and made the estimate
  unusable.
  - Use the Cholesky factor L^T (with L L^T = cov_inv) to whiten the
  cov2cov residual, matching what optimal_tf_gauss_newton accumulates.
  - Multiply the inverse-Hessian by chi^2 / (m - 6), the standard
  a-posteriori unit-weight variance, to rescale the (otherwise
  optimistic) result by the empirical residual level.
* Merge pull request `#58 <https://github.com/MOLAorg/mp2p_icp/issues/58>`_ from MOLAorg/feat/icp-viewer-show-prior
  feat: icp-logs now store the prior SE(3) PDF
* feat: icp-logs now store the prior SE(3) PDF
* icp-log-viewer: safer against exceptions in gui thread
* demo sm2mm files: ignore_accelerometer=true in all deskew stages by default (prevent noisy maps from low-quality IMUs)
* Contributors: Jose Luis Blanco-Claraco

2.9.0 (2026-04-22)
------------------
* Merge pull request `#57 <https://github.com/MOLAorg/mp2p_icp/issues/57>`_ from MOLAorg/feat/deskew-filter-ignore-acc
  FilterDeskew: add new option "ignore_accelerometer"
* Merge pull request `#55 <https://github.com/MOLAorg/mp2p_icp/issues/55>`_ from MOLAorg/feat/mm-viewer-read-bin-files
  mm-viewer: can be also open .bin files with serialized CGenericPointsMap
* Merge pull request `#54 <https://github.com/MOLAorg/mp2p_icp/issues/54>`_ from MOLAorg/feat/mm-apps-plugins
  mm-info, mm2grid, mm2las, mm2ply, mm2txt now have a --load-plugins flag
* Optimization in PointCloudToVoxelGridSingle
* Add <stdexcept> to all required files (don't depend on transitive includes)
* mm-info, mm2grid, mm2las, mm2ply, mm2txt now have a --load-plugins flag
* Merge branch 'generator-generic-cloud' into develop
* cloud rendering: Implement observing the autoBoundingBoxOutliersPercentile
* Merge pull request `#53 <https://github.com/MOLAorg/mp2p_icp/issues/53>`_ from MOLAorg/generator-generic-cloud
  Generator: creates CGenericPointsMap by default; add sanity checks in most filters
* remove more old mrpt version guards
* New sanity check function: warn_on_field_padding_mismatch()
* FilterDeskew: guard against new MRPT behavior to keep all field lengths in sync
* Add sanity checks in filters
* Bump minimum MRPT version to 2.15.4 (and remove now old dead code)
* Generator new param 'filterOutPointsAtZero', set to true in demo pipelines
* Generator: now has a param 'default_pointcloud_class' which defaults to 'CGenericPointsMap'
* Code clean up (remove now dead code; mrpt backwards compatibility)
* Merge pull request `#52 <https://github.com/MOLAorg/mp2p_icp/issues/52>`_ from MOLAorg/icp-log-viewer-quality-filter
  icp-log-viewer: add --min-quality filter CLI flag
* Contributors: Jose Luis Blanco-Claraco

2.8.1 (2026-04-06)
------------------
* Merge pull request `#51 <https://github.com/MOLAorg/mp2p_icp/issues/51>`_ from MOLAorg/fix/new-mrpt-api
  Update to build against mrpt >=2.15.13
* Update to build against mrpt >=2.15.13 (pointcloud field names as std::string instead of string_view)
* Contributors: Jose Luis Blanco-Claraco

2.8.0 (2026-04-01)
------------------
* BUGFIX: Fix potential crash (regression in former voxel parallelization)
* Merge pull request `#50 <https://github.com/MOLAorg/mp2p_icp/issues/50>`_ from MOLAorg/fix/ram-usage
  Fix/ram usage
* Process points by chunks to limit RAM usage
* reduce memory allocations in voxel views
* Smarter usage of reserve() in tsl maps
* Add agents.md
* Merge pull request `#49 <https://github.com/MOLAorg/mp2p_icp/issues/49>`_ from MOLAorg/mm2las/georef
  mm2las: export flag '--frame geodetic' for georeferenced clouds
* Use geodetic coords cache
* Correctly honor exportGeodetic
* mm2las: Add sanity checks
* Fix: X=lon, Y=lat coord order for WKT2
* mm2las: export flag '--frame geodetic' for georeferenced clouds
* Contributors: Jose Luis Blanco-Claraco

2.7.1 (2026-03-08)
------------------
* Merge pull request `#48 <https://github.com/MOLAorg/mp2p_icp/issues/48>`_ from MOLAorg/fix/filter-adaptive-avoid-fpe
  FIX: Avoid potential division by zero in FilterDecimateAdaptive
* Update minimum required MRPT version 2.15.0
* FIX: Avoid potential division by zero in FilterDecimateAdaptive
* Merge pull request `#47 <https://github.com/MOLAorg/mp2p_icp/issues/47>`_ from MOLAorg/fix/georef-yaml-missing-fields
  FIX: georeferencing yaml serialization missed orientation fields
* FIX: georeferencing yaml serialization missed orientation fields
* Merge pull request `#46 <https://github.com/MOLAorg/mp2p_icp/issues/46>`_ from MOLAorg/feat/sm2mm-in-enu-frame
  sm2mm: new  feature to directly use georef as input and produce maps (and apply filters!) in ENU frame
* sm2mm: new  feature to directly use georef as input and produce maps in ENU frame
* Contributors: Jose Luis Blanco-Claraco

2.7.0 (2026-03-03)
------------------
* Merge pull request `#45 <https://github.com/MOLAorg/mp2p_icp/issues/45>`_ from MOLAorg/feat/mm2grid
  Add new cli app: mm2grid
* Add new cli app: mm2grid
* Merge pull request `#44 <https://github.com/MOLAorg/mp2p_icp/issues/44>`_ from MOLAorg/feat/view-vector
  Generators now optionally generate a 'view' direction vector per point
* FilterMLS: now uses the view-direction vectors to ensure normals point outwards
* Generator: add safety consistency check
* Generators are now also included into sm2mm profiler
* Generators now optionally generate a 'view' direction vector per point
* mm-viewer: FIX: show/hide all buttons did not apply to extra viz layers
* mm-georef: Fix creation of empty output map if input didn't exist
* mm2txt: Fix wrong console message saying mm-info instead of mm2txt
* mm-viewer UI: show XY grid plane at the root frame of reference (ENU or map)
* Merge pull request `#43 <https://github.com/MOLAorg/mp2p_icp/issues/43>`_ from MOLAorg/feat/export-enu-frame
  Export tools now accept "--frame enu" to generate XYZ data in ENU frame
* Export tools now accept "--frame enu" to generate XYZ data in ENU frame
* Contributors: Jose Luis Blanco-Claraco

2.6.0 (2026-02-16)
------------------
* Merge pull request `#42 <https://github.com/MOLAorg/mp2p_icp/issues/42>`_ from MOLAorg/feat/filter-remove-several-point-fields
  FilterRemovePointCloudField now accepts multiple fields
* FilterRemovePointCloudField now accepts multiple fields
* Merge pull request `#41 <https://github.com/MOLAorg/mp2p_icp/issues/41>`_ from MOLAorg/feat/functor-save-log-file
  Add functor_should_generate_debug_file to override the decision of whether to write .icplog files
* Add functor_should_generate_debug_file to override the decision of whether to write .icplog files
* Merge pull request `#40 <https://github.com/MOLAorg/mp2p_icp/issues/40>`_ from MOLAorg/feat/new-clear-remove-field-filters
  Add new filters: FilterClear and FilterRemovePointCloudField
* Add new filters: FilterClear and FilterRemovePointCloudField
* docs: refer to the online pipeline editor
* docs: add missing docs for FilterRenameLayer
* Contributors: Jose Luis Blanco-Claraco

2.5.0 (2026-02-04)
------------------
* Merge pull request `#39 <https://github.com/MOLAorg/mp2p_icp/issues/39>`_ from MOLAorg/feat/permit-missing-externals
  sm2mm: Add optional flag --permit-missing-externals
* sm2mm: Add optional flag --permit-missing-externals
* Merge pull request `#38 <https://github.com/MOLAorg/mp2p_icp/issues/38>`_ from MOLAorg/feat/mls-optimizations
  FilterMLS performance optimizations
* Merge pull request `#37 <https://github.com/MOLAorg/mp2p_icp/issues/37>`_ from MOLAorg/feat/use-zstd
  Use ZStd compression by default (for MRPT>=2.15.7)
* docs: sm2mm add "--compression-method"
* Explicitly include mrpt/core/Clock.h where used
* Use ZStd compression by default (for MRPT>=2.15.7)
* Merge pull request `#36 <https://github.com/MOLAorg/mp2p_icp/issues/36>`_ from MOLAorg/feat/mm2txt-missing-fields-dont-emit
  mm2txt, mm2ply: don't emit columns of zeros for missing user-given fields
* Merge pull request `#35 <https://github.com/MOLAorg/mp2p_icp/issues/35>`_ from MOLAorg/feat/mm2txt-ignore-fields
  mm2txt and mm2ply: Add new option --ignore-missing-fields
* Merge pull request `#34 <https://github.com/MOLAorg/mp2p_icp/issues/34>`_ from MOLAorg/feat/mm2txt-mm2ply-more-precision
  Exploit the maximum float32/float64 precision in exported txt formats
* Merge pull request `#33 <https://github.com/MOLAorg/mp2p_icp/issues/33>`_ from MOLAorg/feat/sm2mm-new-options
  Feat/sm2mm-new-options
* sm2mm cli app: add new flags for downsampling options
* sm2mm: add new downsample options and refactor into update_velocity_buffer_from_obs()
* Contributors: Jose Luis Blanco-Claraco

2.4.1 (2026-01-27)
------------------
* Add more unit tests
* Update README to keep it in sync with the provided apps and libraries
* Merge pull request `#29 <https://github.com/MOLAorg/mp2p_icp/issues/29>`_ from MOLAorg/fix/cov
  Fix bugs in covariance estimation for some cases
* fix covariance estimation bugs; add unit tests for cov2cov
* Update commit for mola_common
* Merge pull request `#28 <https://github.com/MOLAorg/mp2p_icp/issues/28>`_ from MOLAorg/feat/mm-viewer-3d-layers
  mm-viewer: add CLI flags to load overlaid 3D scenes for visualization
* mm-viewer: add CLI flags to load overlaid 3D scenes for visualization
* Contributors: Jose Luis Blanco-Claraco

2.4.0 (2026-01-21)
------------------
* Merge pull request `#27 <https://github.com/MOLAorg/mp2p_icp/issues/27>`_ from MOLAorg/feat/new-filter-voxel-sor
* Add new unit test for class factory
* Add new FilterVoxelSOR filter
* Merge pull request `#26 <https://github.com/MOLAorg/mp2p_icp/issues/26>`_ from MOLAorg/feat/mm2las
* Add mm2las CLI tool
* Contributors: Jose Luis Blanco-Claraco

2.3.1 (2026-01-14)
------------------
* Merge pull request `#25 <https://github.com/MOLAorg/mp2p_icp/issues/25>`_ from MOLAorg/feat/naive-decimate
  Add trivial FilterDecimate for fast downsampling without spatial awareness
* lint fixes
* Add trivial FilterDecimate for fast downsampling without spatial awareness
* Parameterizable: add virtual base dtor
* Remove the NormalizeIntensity stage in the demo pipelines; visualization does that already
* docs: fill missing manpages
* docs: add sm2mm pipelines page
* Clarify map layers and simple maps descriptions
  Updated references to CMetricMap and CGenericPointsMap in the documentation for clarity and accuracy.
* Contributors: Jose Luis Blanco-Claraco

2.3.0 (2026-01-08)
------------------
* Merge pull request `#24 <https://github.com/MOLAorg/mp2p_icp/issues/24>`_ from MOLAorg/feat/mm2txt-select-fields
  mm2txt and mm2ply now have a --export-fields flag
* mm2txt and mm2ply now have a --export-fields flag
* Merge pull request `#23 <https://github.com/MOLAorg/mp2p_icp/issues/23>`_ from MOLAorg/fix/some-deprecations
  Fix usage of deprecated cloud types
* Provide shortcut names for common cloud field names
* More deprecated cloud usage
* FIX bug: FilterDecimateVoxel, if using flatten, did not propagate all input cloud fields
* Fix usage of some deprecated cloud types
* FilterSOR: create output layers even if input is empty
* FilterDeskew: propagate input fields even if the cloud is empty
* FilterByExpression: show debug-level stats
* FilterNormalizeIntensity: do not throw on empty clouds
* Merge pull request `#22 <https://github.com/MOLAorg/mp2p_icp/issues/22>`_ from MOLAorg/feat/new-filters
  Add new filter FilterRenameLayer
* Added filter FilterRenameLayer
* mm2txt: prepare for deprecated classes in 3.0.0
* FilterAdjustTimestamps: new method 'None' to bypass filter
* Fix: sm2mm did not attach to ParameterSource the final_filter elements
* sm2mm: did not observe the optional profiler parameter for the final_filter stage
* Fix: FilterMLS did not properly copy all point fields when using upsampling
* Fix: FilterAbsoluteTimestamp now also works for accumulated points in one layer
* Contributors: Jose Luis Blanco-Claraco

2.2.1 (2026-01-06)
------------------
* Merge pull request `#21 <https://github.com/MOLAorg/mp2p_icp/issues/21>`_ from MOLAorg/feat/abs-stamp-filter
  Added new filter: FilterAbsoluteTimestamp
* Fix the logic of the FilterEdgePlane filter parameters
* Added new filter: FilterAbsoluteTimestamp
* mm2txt: also export uint8 fields (missing in last release)
* Merge pull request `#20 <https://github.com/MOLAorg/mp2p_icp/issues/20>`_ from MOLAorg/feat/more-unit-tests
  More unit tests
* Add generators unit tests
* More unit tests
* Contributors: Jose Luis Blanco-Claraco

2.2.0 (2025-12-28)
------------------
* docs: explain FilterSOR
* Merge pull request `#19 <https://github.com/MOLAorg/mp2p_icp/issues/19>`_ from MOLAorg/feat/mm2ply
  Add mm2ply CLI tool
* Merge pull request `#18 <https://github.com/MOLAorg/mp2p_icp/issues/18>`_ from MOLAorg/feat/new-sor-filter
  Add FilterSOR: Statistical Outlier Rejection
* More unit tests: cover MLS
* Merge pull request `#17 <https://github.com/MOLAorg/mp2p_icp/issues/17>`_ from MOLAorg/feat/filter-by-expr
  Add new filter: FilterByExpression
* More code coverage; fix protected-level initialize methods
* Add new filter: FilterByExpression
* FIX: missing uint8 fields in Deskew
* sanityCheck: also check all double/uint8 fields
* ui: fix case without geo-ref
* mm-viewer: show lat/lon coordinates for mouse selected points
* mm-viewer: GUI now shows the ENU & map miniviews for orientation hints
* mm-viewer: handle special coloring channel groups 'rgb' and 'rgbf'
* clean non used code in deskew test
* map viz: use new mrpt 2.15.3 coloring modes
* sm2mm: auto-guess lazy-load externals directory for .simplemap files
* mm-viewer: implement colorize clouds by any field
* MLS filter: add 'output_layer_class' parameter
* mm2txt: export all fields of CGenericPointsMap layers
* Merge pull request `#16 <https://github.com/MOLAorg/mp2p_icp/issues/16>`_ from MOLAorg/fix/generic-cloud-deskew-fields
  Add test for missing fields in generic cloud deskew
* FIX: correctly register arbitrary pointcloud fields into output clouds
* Add test for missing fields in generic cloud deskew
* sm2mm: FIx console message on "FinalFilters" did not obey verbosity level
* Fix clang-tidy warnings
* Contributors: Jose Luis Blanco-Claraco

2.1.2 (2025-11-28)
------------------
* mm-viewer: Automatic retry to load maps if missing plugins, trying to reload with libmola_metric_maps.so
* Enable coverage run for noble docker image
* Add new unit tests
* docs: fix broken formatting of filters page
* Merge pull request `#14 <https://github.com/MOLAorg/mp2p_icp/issues/14>`_ 
  deskew filter: refactoring to handle arbitrary point fields
* get code ready for API update in MRPT 2.15.3
* Fix build w/o imu library
* mm-viewer: add keystrokes shift+cursor arrows to move up/down
* Add Codecov badge to README
* MLS filter: add progress report logging
* Contributors: Jose Luis Blanco-Claraco

2.1.1 (2025-11-08)
------------------
* FIX: SanityCheck was triggering as errors optional pointcloud fields in XYZIRT clouds
* FIX: Throw exception instead of crashing if FilterDeskew is invoked with an empty local velocity buffer
* Fix yaml file for not using mola_yaml extensions
* Add more sm2mm demo pipelines
* Contributors: Jose Luis Blanco-Claraco

2.1.0 (2025-10-28)
------------------
* Merge pull request `#13 <https://github.com/MOLAorg/mp2p_icp/issues/13>`_ from MOLAorg/fix/filterdecimate-bug
* Add unit test for FilterDecimateVoxel
* mm-viewer: ensure proper order of opengl object destruction
* Add more debug traces for Filters
* Generator: Add more info on layer contents in debug traces
* FIX: Avoid crash in FilterVoxelSlice if there are no points in the ROI
* Merge pull request `#12 <https://github.com/MOLAorg/mp2p_icp/issues/12>`_ from MOLAorg/feat/mls
  Implemented MLS filter
* Merge pull request `#11 <https://github.com/MOLAorg/mp2p_icp/issues/11>`_ from MOLAorg/feat/use-faster-insertion
  Use the faster insertion-with-context in MRPT 2.15.0
* Update to build using CGenericPointsMap in upcoming MRPT >=2.15
* Merge pull request `#10 <https://github.com/MOLAorg/mp2p_icp/issues/10>`_ from MOLAorg/fix/ci
* Upgrade CircleCI images to u24.04
* Fix build out of ROS and w/o IMU preintegration
* Update mola_common version
* Fix: Warning message missing new line
* FilterDecimateVoxels: add new parameter 'minimum_points_per_voxel'
* Fix docs typo
* Update docs for Deskew filter
* Update README.md badges
* Contributors: Jose Luis Blanco-Claraco

2.0.0 (2025-10-13)
------------------
* Merge pull request `#9 <https://github.com/MOLAorg/mp2p_icp/issues/9>`_ from MOLAorg/feature/better-lio
  Better LIO
* sm2mm cli app: add --profiler flag
* demo sm2mm pipelines: add deskew method entry
* CI: Add another pipeline without TBB
* FIX: bug in non-TBB serial implementation of GN optimizer
* CI: add running unit tests
* FIX: potential crash in FilterDeskew
* Add deskew unit tests
* Add unit test for cov2cov optimizer
* Add 'name' property to all generators and filters for disaggregated stats
* Allow building without the IMU library
* Update mola_common to 0.5.1
* clang-tidy fixes
* Remove dead code
* Refactor errorTerm for pt2pt for better reusability
* mm-viewer: add combo box to select intensity colormap
* Add docs for filters
* BUGFIX: FilterMerge would lost all point fields except XYZ
* Remove external libpointmatcher
* Update formatter script
* icp log viewer: more options for cov2cov visualization
* Define virtual API MetricMapMergeCapable
* fix bug in FilterByIntensity params parser
* Render cov2cov pairings
* FilterByRange new parameter: metric_l_infinity
* FilterDecimateAdaptive now exploits parallelization
* Progress visualizing cov2cov pairings
* New cov2cov ICP optimizer
* Implement a new Cov2Cov matcher
* Make Matcher_Points_Base::transform_local_to_global() to use TBB, and remove unused parameters
* Add [[nodiscard]] to estimate_points_eigen()
* BBox filter: fix target layer must be same type than input
* Use fractional integers for faster sampling
* New FPS filter
* Remove FilterDecimateVoxelsQuadratic
* Style: public 'params\_' rename as 'params'
* New interface 'IcpPrepareCapable'
* Add new virtual interface NearestPointWithCovCapable
* Refactor mp2p_icp_map into mp2p_icp_common for IMU-related parts
* Finished integration of new IMU API package
* Depend on imu external library
* Move code out to the imu preintegration package
* Move LocalVelocityBuffer to the IMU repository
* Add [[nodiscard]] to icp_pipeline_from_yaml()
* deskew filter: new option 'in_place' to avoid allocating a new cloud whenever possible
* Finish implementation of higher-order IMU interpolator
* Implement trajectory reconstruction for deskew
* cmake files: prefer spaces indentation
* Add imu preintegration package as dependency
* FilterNormalizeIntensity can now use a fixed min/max range given by hand
* Docs: update mp2p_icp_basics for better searchability of simplemaps
* New option to set pointcloud alpha channel
* Contributors: Jose Luis Blanco-Claraco

1.8.0 (2025-08-26)
------------------
* Modernize and unify license notes in all files
* Merge pull request `#8 <https://github.com/MOLAorg/mp2p_icp/issues/8>`_ from MOLAorg/feat/precise-deskew
  Precise scan deskew:
  - Implement LocalVelocityBuffer inside ParameterSource's
  - Update LocalVelocityBuffer from IMU data from Generators.
  - Export / Import LocalVelocityBuffer to/from YAML
  - Implement precise cloud undistortion in FilterDeskew
  - Use precise cloud undistortion in the context of sm2mm.
* sm2mm: Use local velocity buffer if available
* add serialization to velocity buffer
* Generators now handle IMU readings and forward them to the velocity buffer
* Update to latest mola_common for embedded builds
* linter: clang-tidy fixes
* fix param name for better consistency
* feature: Option to use std::map instead of tsl robin_map in voxelization filters
* docs: fill txt2mm man page
* Feature: txt2mm new import format 'xyzrgb_normalized'
* remove code to support older MRPT versions; code style clean ups
* Fix: FilterAdjustTimestamps may trigger exception if input cloud is empty
* Contributors: Jose Luis Blanco-Claraco

1.7.1 (2025-06-20)
------------------
* docs: Populate sm2mm app page
* New feature: all pipeline modules now has an optional "plugin" YAML field to load them from user-provided plugins.
* Update REAME ROS badges
* Contributors: Jose Luis Blanco-Claraco

1.7.0 (2025-06-02)
------------------
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

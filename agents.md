# mp2p_icp — Agent Context Guide

Quick-start reference for AI agents and new contributors. Keep it up-to-date when
the library changes, but keep it short: invariants and pointers, not changelog prose.

## Project identity

**mp2p_icp** (Multi Primitive-to-Primitive ICP): C++ library and CLI toolkit for point
cloud registration and map building, part of the [MOLA](https://github.com/MOLAorg/mola)
framework.

- BSD-3-Clause, maintainer Jose Luis Blanco-Claraco (version: see `package.xml`)
- Minimum MRPT: 2.15.4 (`mp2p_icp_core`); 2.15.11 for `mp2p_icp_viz`

## Repository layout

Three sibling ROS packages, split so headless consumers don't pull GUI deps (`mrpt_libgui`):

```
mp2p_icp/                    (repo root)
├── mp2p_icp_core/            # headless libs + CLI apps
│   ├── mp2p_icp_common/       # base utilities, Parameterizable
│   ├── mp2p_icp_map/          # metric_map_t, .mm I/O, georeferencing
│   ├── mp2p_icp/              # ICP: Matchers, Solvers, QualityEvaluators
│   ├── mp2p_icp_filters/      # filters, generators, voxel grid utilities
│   ├── apps/ tests/ demos/ 3rdparty/ scripts/
├── mp2p_icp_viz/             # GUI apps (mm-viewer, icp-log-viewer)
├── mp2p_icp/                 # metapackage (backward compat, no code)
└── docs/                     # Sphinx sources
```

Build via colcon only (plain-CMake standalone builds unsupported). Each library still
exports its own CMake config (e.g. `mola::mp2p_icp_map`). Downstream packages needing
only headless code: `<depend>mp2p_icp_core</depend>`; `<depend>mp2p_icp</depend>` pulls
in everything.

## Core data structure: `metric_map_t`

`mp2p_icp_map/include/mp2p_icp/metricmap.h`: named point cloud `layers` (CMetricMap),
`lines`, `planes`, optional `id` / `label` / `metadata` / `georeferencing`.
Standard layer names are the `PT_LAYER_*` constants (`"raw"`, `"plane_centroids"`).

`Georeferencing` holds `geo_coord` (WGS-84 of origin) and `T_enu_to_map`
(pose of {map} as seen from {enu}). Map to geodetic:

```
map → T_enu_to_map.mean.composePoint() → ENU → ENUToGeocentric() → geocentricToGeodetic()
```

No inverse is needed for that direction.

## .mm file format

Binary MRPT `CSerializable`, gzip-compressed, serialization version 5.
`mm.load_from_file()` / `save_to_file()`; inspect with `mm-info` or `mm-viewer`.

## CLI applications

Argument parsing uses CLI11 (`CLI11::CLI11`), not TCLAP.

| App | Package | Purpose |
|-----|---------|---------|
| `mm2las` | core | Export layers → LAS 1.4; `--frame map\|enu\|geodetic` |
| `mm2ply` / `mm2txt` / `mm2grid` | core | Export layers → PLY / CSV / grid maps |
| `mm-filter` | core | Apply filter pipelines to .mm files |
| `mm-info` | core | Print .mm summary |
| `mm-georef` | core | Inject/extract georeferencing |
| `sm2mm` / `sm-cli` | core | CSimpleMap → .mm / manipulate CSimpleMap |
| `icp-run` | core | Run ICP pipeline from CLI |
| `kitti2mm` / `txt2mm` | core | KITTI .bin / TXT-CSV → .mm |
| `rawlog-filter` | core | Filter MRPT RawLog files |
| `mm-viewer` / `icp-log-viewer` | viz | GUI viewer / ICP session debugger |

`mm-filter` pipelines may include a `generators:` section: since there is no observation
stream, each generator with a `metric_map_definition` only pre-creates its `target_layer`
as an empty map of that class (via `Generator::createTargetLayerIfNeeded()` /
`CreateMetricMapFromDefinition()`), so `FilterMerge` can insert into an arbitrary
`CMetricMap` subclass. See `demos/mm-filter_create_keyframe_map_layer.yaml`.

`mm2las --frame geodetic` writes lon/lat/ellipsoidal-height with an EPSG:4979 WKT2 VLR;
it uses per-point `latitude`/`longitude`/`altitude` fields when present, else converts
on the fly through `T_enu_to_map`.

## GUI apps (mp2p_icp_viz): Dear ImGui port

Migrating from `mrpt::gui::CDisplayWindowGUI`+nanogui to Dear ImGui. ImGui comes from the
`mrpt_imgui_vendor` ROS package (docking branch, plus ImPlot and portable-file-dialogs),
exported as a static `imgui::imgui`; it is no longer vendored in this repo. Both apps share
`apps/imgui_app_common/` (`ImGuiAppShell`, `SimpleFileDialog`). 3D views use
`mrpt::imgui::CImGuiSceneView`.

**Do not delete as dead code:** the axis-corner mini-viewports
(`FIRST_MINI_VIEW_NAME`/`SECOND_MINI_VIEW_NAME`, `MINI_VIEW_NAME`) are kept in sync but
invisible, because MRPT 2.x only renders the `"main"` viewport. Expected to work again
unmodified under MRPT 3.x.

## ICP pipeline

```
metric_map_t (local) + metric_map_t (global) + CPose3D (initial guess)
    → Matchers (pt-pt, pt-plane, pt-line, cov-cov pairings)
    → Solvers (Horn, OLAE, Gauss-Newton)
    → QualityEvaluators → iterate until convergence
```

All components are `Parameterizable`, configured via YAML (`mp2p_icp::Parameters`).

**Determinism is a hard invariant.** Offline runs of one configuration must be
byte-reproducible, so `tbb::parallel_reduce` is never used: pairing concatenation and
`optimal_tf_gauss_newton()`'s H/g/cost accumulation use
`tbb::parallel_deterministic_reduce` with an explicit grain size (the grain size is what
fixes the partition). Guarded by `test-mp2p_solver_determinism` (1/2/3/4/8 workers,
bit-identical poses).

### Matchers: load-bearing properties

- `Matcher_Cov2Cov`: acceptance via `MatchingDistanceProfile` (implicitly constructible
  from `float`, so a flat threshold is the default fast path; optional logistic
  range-adaptive variant). Pairing weights via `PointWeightByRange`
  (`pointWeightAlpha`/`RefRange`/`Min`/`Max`); `alpha = 0` disables it and reproduces
  earlier releases. It scales `cov_inv`, so it reaches gradient, Hessian and robust
  kernel alike; `Solver_GaussNewton`'s Birge-ratio balancing absorbs a constant factor,
  so only the curve shape matters.
- `Matcher_NDT_Blend`: `Matcher_Point2Plane` with the `argmin` replaced by a
  likelihood-weighted blend, so residuals vary continuously with the pose. Invariants:
  `temperature: 0` reproduces `Matcher_Point2Plane` bit for bit; weights fade to zero
  with zero derivative at `searchRadius`; normals accumulate as outer products (sign
  invariant). At `DEBUG` verbosity only, logs a `blendstats` line per layer match.
  See `test-mp2p_matcher_ndt_blend`.
- `Matcher_Points_Blend`: point counterpart, blends map points inside `searchRadius`.
  `temperature: 0` reproduces `Matcher_Points_DistanceThreshold` with
  `pairingsPerPoint: 1`; radius query, never top-k; zero-derivative fade at the radius.
  `globalIdx` carries the nearest neighbor (for already-paired bookkeeping),
  `errorSquareAfterTransformation` the blended residual. See
  `test-mp2p_matcher_points_blend`.
- `Matcher_Points_KnnPlane`: deliberately non-smooth Fast-LIO2 reproduction: knn points,
  least-squares plane, three hard gates, emits `paired_pt2pl`. The plane is fitted by a
  column-pivoting Householder QR of `A x = -1` in single precision (not PCA), and no gate
  reads back registration quality. Ships as `lidar3d-fastlio-matching.yaml` with the
  adaptive controller disabled. See `test-mp2p_matcher_knn_plane`.

## Filter pipeline

Filters are chained and applied in-place to `metric_map_t`:

```yaml
filters:
  - class_name: mp2p_icp_filters::FilterDecimateVoxels
    params:
      voxel_filter_resolution: 0.5
```

Categories: decimation (including range-adaptive EllipseLIO-style), outlier removal,
range/ring/intensity gating, deskew, edge/plane extraction, layer management.

`FilterDecimateAdaptive` accepts either the single-output keys
(`output_pointcloud_layer` + `desired_output_point_count`) or an `outputs` sequence; all
output layers are sampled from ONE voxelization pass, each with its own stride.

`FilterDecimateVoxels` picks one of three voxel grids per `decimate_method`, because
what each method needs from a voxel differs: `PointCloudToVoxelGridSingle` (one point,
for `FirstPoint`), `PointCloudToVoxelGridAverage` (sums plus the closest point, for
`ClosestToAverage`/`VoxelAverage`), and `PointCloudToVoxelGrid` (the full index list, for
`RandomPoint`, and for the other filters that walk a voxel's points). Only the last one
pays for a second hashed pass and an index-array relayout, so a method must not be moved
onto it without reason. Two things keep the grids interchangeable: voxel keys come from
`coord2idx`, which **divides** by the resolution (multiplying by a precomputed reciprocal
assigns boundary points to different voxels at resolutions that are not powers of two),
and per-voxel sums are accumulated in ascending point order, so the average is bit-identical
whichever grid produced it.

Both decimation filters share `DecimateMethod` (`mp2p_icp_filters/DecimateMethod.h`).
Because `FilterDecimateAdaptive` revisits voxels in several rounds,
`FirstPoint`/`RandomPoint` take successive points out of each voxel, while
`ClosestToAverage`/`VoxelAverage` summarize the voxel and emit at most one point per
voxel (output capped at the voxel count). `VoxelAverage` synthesizes points, so per-point
fields are not propagated.

### Standalone utilities (not `Filter` subclasses)

- `mp2p_icp_filters::robust_max_range()` (`PointCloudRobustRange.h`): percentile
  (default 0.95) of per-point range instead of the raw maximum, so far specular outliers
  cannot dominate an observation-radius estimate. `O(n)` via `std::nth_element`. Not yet
  wired into any consumer. See `test-mp2p_PointCloudRobustRange.cpp`.

## Build and test

```bash
cd ~/ros2_ws
colcon build --packages-select mp2p_icp        # everything (metapackage)
colcon build --packages-select mp2p_icp_core   # headless only
colcon test  --packages-select mp2p_icp_core   # or: ctest --test-dir build/mp2p_icp_core -V
```

CMake options (`mp2p_icp_core`): `MP2PICP_BUILD_TESTING`, `MP2PICP_BUILD_APPLICATIONS`
(both default ON), `MP2PICP_USE_TBB` (auto-detected). SIMD translation units are compiled
separately with `-mavx` / `-msse2`. Tests use gtest; each filter, matcher, solver and
serializer has its own file in `tests/`.

## Dependencies

| Dependency | Notes |
|-----------|-------|
| MRPT ≥ 2.15.4 (`mp2p_icp_core`); ≥ 2.15.11 (`mp2p_icp_viz`) | containers, tfest, maps, topography; `gui` only for mp2p_icp_viz |
| CLI11 | CLI parsing for all apps (rosdep key `cli11`) |
| TBB | Optional; parallel ICP iterations |
| mola_common | CMake scripts, from the colcon workspace (not vendored) |
| mola_imu_preintegration | Optional; advanced deskew in FilterDeskew |

## Coding conventions

- Public headers in `<module>/include/mp2p_icp/`, implementations in `<module>/src/`
- Register classes with `DEFINE_MRPT_OBJECT` / `IMPLEMENTS_MRPT_OBJECT`
- Parameters follow `Parameterizable::initialize(mrpt::containers::yaml)`
- Numeric parameters a user might write as a formula (distances, thresholds, radii, kernel
  scales) must use `DECLARE_PARAMETER_{REQ,OPT}` / `DECLARE_PARAMETER_IN_{REQ,OPT}`, never
  `MCP_LOAD_*`, which silently truncates `"2.0*ADAPTIVE_THRESHOLD_SIGMA"` at the first
  non-numeric character. `MCP_LOAD_*` stays correct for strings, enums, booleans, counts
  and intentionally static numbers
- Frame naming: `T_A_to_B` = pose of {B} as seen from {A}; `composePoint` maps B → A
- Always brace `if`/`for`/`while`; one variable per declaration; anonymous namespaces
  instead of `static`; American spelling; no en/em dashes
- Honor clang-format-14 and clang-tidy; don't sign commits as an AI agent

## Update rules

- Keep this file current, but short: state invariants and where the test lives, not the
  history of how a feature was fixed
- New/updated `mp2p_icp_filters` classes: sync `docs/source/mp2p_icp_filters.rst` and
  `~/code/mp2p-pipeline-editor-sources` if it exists (read its `README.md`)

## Related MOLA packages (same workspace)

- `mola_state_estimation/mola_georeferencing` — `mola-mm-add-geodetic` adds per-point
  lat/lon/alt fields (prerequisite for the mm2las geodetic fast path)
- `mola_lidar_odometry` — primary consumer, real-time SLAM

## MRPT3 to-do

Once ported to MRPT 3, drop the glut/freeglut3-dev dependency and delete this section.

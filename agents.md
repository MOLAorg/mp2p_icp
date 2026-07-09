# mp2p_icp — Agent Context Guide

Quick-start reference for AI agents and new contributors.

## Project identity

**mp2p_icp** (Multi Primitive-to-Primitive ICP) is a C++ library and CLI toolkit for point cloud registration and map building. It is part of the [MOLA](https://github.com/MOLAorg/mola) framework.

- **Version**: 2.11.0
- **License**: BSD-3-Clause
- **Maintainer**: Jose Luis Blanco-Claraco
- **Minimum MRPT**: 2.15.4

Whenever a change is done to the library by an AI agent, keep this document up-to-date so it
always reflects the current state.

---

## Repository layout

Since mid-2026 this repo hosts 3 sibling ROS packages, split so that headless consumers
don't need to pull in GUI/display dependencies (`mrpt_libgui`):

```
mp2p_icp/                    (repo root)
├── mp2p_icp_core/            # ROS package: headless libs + CLI apps, no GUI deps
│   ├── mp2p_icp_common/       # Base utilities, Parameterizable class
│   ├── mp2p_icp_map/          # metric_map_t container, .mm file I/O, georeferencing
│   ├── mp2p_icp/               # ICP algorithms, Matchers, Solvers, QualityEvaluators
│   ├── mp2p_icp_filters/       # 38+ filters, 2 generators, voxel grid utilities
│   ├── apps/                   # 13 headless CLI applications (see below)
│   ├── tests/                  # 50+ gtest unit tests
│   ├── demos/                  # Example configs, demo point clouds
│   ├── 3rdparty/                # robin-map (vendored, header-only, private dep)
│   └── scripts/                 # formatter.sh, etc.
├── mp2p_icp_viz/              # ROS package: GUI apps (mm-viewer, icp-log-viewer)
│   └── apps/
├── mp2p_icp/                  # metapackage (backward compat, build_type cmake, no code): depends on both above
└── docs/                      # Sphinx documentation source
```

Standalone (non-colcon) plain-CMake builds are no longer supported; build via colcon
in a ROS workspace. Each library still exports its own CMake config (e.g.
`find_package(mp2p_icp_map REQUIRED)` + `mola::mp2p_icp_map`, unchanged), so consumers can
depend on individual libraries directly. New downstream ROS packages that only need the
headless libraries/CLI apps should use `<depend>mp2p_icp_core</depend>` (no `mrpt_libgui`);
`<depend>mp2p_icp</depend>` still pulls in everything (core + GUI apps), unchanged, via the
metapackage.

---

## Core data structure: `metric_map_t`

Defined in `mp2p_icp_map/include/mp2p_icp/metricmap.h`.

```cpp
struct metric_map_t {
    // Named point cloud layers (CPointsMap subclasses)
    std::map<layer_name_t, mrpt::maps::CMetricMap::Ptr> layers;

    // Geometric features
    std::vector<mrpt::math::TLine3D>  lines;
    std::vector<plane_patch_t>        planes;

    // Optional metadata
    std::optional<uint64_t>           id;
    std::optional<std::string>        label;
    mrpt::containers::yaml            metadata;

    // Optional georeferencing (added in .mm format v4)
    std::optional<Georeferencing>     georeferencing;
};
```

**Standard layer names** (defined as `PT_LAYER_*` constants):
- `"raw"` — original full point cloud
- `"plane_centroids"` — one point per extracted plane

### `metric_map_t::Georeferencing`

```cpp
struct Georeferencing {
    mrpt::topography::TGeodeticCoords   geo_coord;       // WGS-84 lat/lon/height of origin
    mrpt::poses::CPose3DPDFGaussian     T_enu_to_map;    // pose of {map} as seen from {enu}
};
```

**Convention**: `T_enu_to_map` is the pose of the map frame as seen from ENU. To transform a point from map → ENU, use `T_enu_to_map.mean.composePoint(ptMap)`. The "-" (inverse) is **not** needed for this direction.

**Coordinate conversion chain** (map → geodetic):
```
map coords → T_enu_to_map.mean.composePoint() → ENU
ENU → mrpt::topography::ENUToGeocentric() → geocentric
geocentric → mrpt::topography::geocentricToGeodetic() → WGS-84 lat/lon/alt
```

---

## .mm file format

- Binary, MRPT `CSerializable`-based, gzip-compressed
- Current serialization version: 5
- Load/save: `mm.load_from_file(path)` / `mm.save_to_file(path)`
- Inspect: `mm-info mymap.mm` or `mm-viewer mymap.mm`

---

## CLI applications

CLI argument parsing uses CLI11 (`find_package(CLI11 REQUIRED)`, `CLI11::CLI11`), not TCLAP.

| App | Package | Purpose |
|-----|---------|---------|
| `mm2las` | mp2p_icp_core | Export layers → LAS 1.4 (Point Format 8); supports `--frame map\|enu\|geodetic` |
| `mm2ply` | mp2p_icp_core | Export layers → PLY |
| `mm2txt` | mp2p_icp_core | Export layers → CSV/TXT |
| `mm2grid` | mp2p_icp_core | Generate grid maps from point clouds |
| `mm-filter` | mp2p_icp_core | Apply filter pipelines to .mm files |
| `mm-info` | mp2p_icp_core | Print .mm file summary |
| `mm-georef` | mp2p_icp_core | Inject/extract georeferencing from .mm files |
| `mm-viewer` | mp2p_icp_viz | GUI viewer for .mm files (needs `mrpt_libgui`) |
| `sm2mm` | mp2p_icp_core | Convert SLAM CSimpleMap → .mm |
| `sm-cli` | mp2p_icp_core | Manipulate CSimpleMap files (cut, join, export…) |
| `icp-run` | mp2p_icp_core | Run ICP pipeline from CLI |
| `icp-log-viewer` | mp2p_icp_viz | Debug ICP sessions interactively (needs `mrpt_libgui`) |
| `kitti2mm` | mp2p_icp_core | Convert KITTI .bin → .mm |
| `txt2mm` | mp2p_icp_core | Convert TXT/CSV point clouds → .mm |
| `rawlog-filter` | mp2p_icp_core | Filter MRPT RawLog files |

### `mm2las` geodetic export (`--frame geodetic`)

Exports WGS-84 lon/lat/alt with EPSG:4979 WKT2 CRS embedded as VLR.

- X = longitude, Y = latitude, Z = ellipsoidal height
- Scale: 1e-8 deg for lat/lon (~1mm), 0.001m for altitude
- `global_encoding` bit 4 set only when WKT VLR is actually written
- Fast path: if the map already has per-point `latitude`/`longitude`/`altitude` double fields (added by `mola-mm-add-geodetic`), they are used directly without recomputation
- Fallback: on-the-fly map→ENU→geocentric→geodetic conversion using `T_enu_to_map`
- Results cached during bounds pass to avoid double computation

---

## ICP pipeline

```
metric_map_t (local) + metric_map_t (global) + CPose3D (initial guess)
    → Matchers  (generate pairings: pt-pt, pt-plane, pt-line, cov-cov)
    → Solvers   (compute optimal SE(3): Horn, OLAE, Gauss-Newton)
    → QualityEvaluators (score alignment)
    → iterate until convergence
```

All components are `Parameterizable` — configured via YAML at runtime, loaded with `mp2p_icp::Parameters`.

---

## Filter pipeline

Filters are chained and applied in-place to `metric_map_t`. Configured via YAML:

```yaml
filters:
  - class_name: mp2p_icp_filters::FilterDecimateVoxels
    params:
      voxel_filter_resolution: 0.5
  - class_name: mp2p_icp_filters::FilterByRange
    params:
      range_min: 1.0
      range_max: 50.0
```

Key filter categories: decimation (including range-adaptive EllipseLIO-style), outlier removal, range/ring/intensity gating, deskew, edge/plane extraction, layer management.

---

## Build system

```bash
# ROS 2 workspace build (metapackage: builds everything, same as before the split)
cd ~/ros2_ws
colcon build --packages-select mp2p_icp

# Or build just the headless libs/apps (no mrpt_libgui pulled in):
colcon build --packages-select mp2p_icp_core
# Or just the GUI apps (depends on mp2p_icp_core):
colcon build --packages-select mp2p_icp_viz

# CMake options of interest (mp2p_icp_core)
-DMP2PICP_BUILD_TESTING=ON      # unit tests (default ON)
-DMP2PICP_BUILD_APPLICATIONS=ON # CLI apps (default ON)
-DMP2PICP_USE_TBB=ON            # TBB parallelism (auto-detected)
```

SIMD-optimized translation units are compiled separately with `-mavx` / `-msse2`.

---

## Dependencies

| Dependency | Notes |
|-----------|-------|
| MRPT ≥ 2.15.4 | containers, tfest, maps, topography — mandatory in mp2p_icp_core (no `gui` component: opengl/system/expr come transitively via maps→obs and tfest→poses→bayes→config). `gui` is only required by mp2p_icp_viz. |
| CLI11 | CLI argument parsing for all apps (rosdep key `cli11`) |
| TBB | Optional; enables parallel ICP iterations |
| mola_common | CMake scripts; resolved via `find_package(mola_common REQUIRED)`, provided by the ROS/colcon workspace (no longer vendored as a submodule) |
| mola_imu_preintegration | Optional; advanced deskew in FilterDeskew |

Note: the `mp2p_icp` (ICP algorithms) library used to publicly link `mrpt::gui` solely for
an interactive debug feature in `QualityEvaluator_RangeImageSimilarity` (`debug_show_all_in_window`,
opened 4 `CDisplayWindow`s). That feature has been removed; only the headless
`debug_save_all_matrices` (file-dump) option remains.

---

## Testing

```bash
colcon test --packages-select mp2p_icp_core
# or directly:
ctest --test-dir build/mp2p_icp_core -V
```

Tests use gtest. Each filter, matcher, solver, and serializer has its own test file in `tests/`.

---

## Coding conventions

- Headers in `<module>/include/mp2p_icp/` (public API)
- Implementations in `<module>/src/`
- All classes registered with MRPT's RTTI: `DEFINE_MRPT_OBJECT` / `IMPLEMENTS_MRPT_OBJECT`
- Parameters follow the `Parameterizable` interface: `initialize(mrpt::containers::yaml)`
- Always use braces `{}` for all `if`/`for`/`while` blocks
- Coordinate frame naming: `T_A_to_B` = pose of {B} as seen from {A}; `composePoint` transforms FROM the local (B) frame TO the reference (A) frame
- Don't use long hyphens. Use American spelling.
- Don't use static, prefer anynomous namespaces.

---

## Update rules

- Use American spelling, do not use en/em dashes, don't sign commits as an AI agent.
- Honor clang-format-14 and clang-tidy; in particular, don't declare multiple variables 
  in the same line, prefer "if (x) {\n ...;\n }" to single line statements.
- If you update or create a new `mp2p_icp_filters` class, keep it in synch with `docs/source/mp2p_icp_filters.rst` and 
  with `~/code/mp2p-pipeline-editor-sources` if it exists (read its `README.md` file).


---

## Related MOLA packages (same workspace)

- `mola_state_estimation/mola_georeferencing` — `mola-mm-add-geodetic` tool that adds per-point lat/lon/alt double fields to .mm layers (prerequisite for mm2las geodetic fast path)
- `mola_lidar_odometry` — primary consumer of mp2p_icp for real-time SLAM

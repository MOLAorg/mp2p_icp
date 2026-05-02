# mp2p_icp — Agent Context Guide

Quick-start reference for AI agents and new contributors.

## Project identity

**mp2p_icp** (Multi Primitive-to-Primitive ICP) is a C++ library and CLI toolkit for point cloud registration and map building. It is part of the [MOLA](https://github.com/MOLAorg/mola) framework.

- **Version**: 2.7.1 (March 2026)
- **License**: BSD-3-Clause
- **Maintainer**: Jose Luis Blanco-Claraco
- **Minimum MRPT**: 2.15.4

---

## Repository layout

```
mp2p_icp/
├── mp2p_icp_common/   # Base utilities, Parameterizable class
├── mp2p_icp_map/      # metric_map_t container, .mm file I/O, georeferencing
├── mp2p_icp/          # ICP algorithms, Matchers, Solvers, QualityEvaluators
├── mp2p_icp_filters/  # 38+ filters, 2 generators, voxel grid utilities
├── apps/              # 15 CLI applications (see below)
├── tests/             # 50+ gtest unit tests
├── demos/             # Example configs, demo point clouds
└── docs/              # Sphinx documentation source
```

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

| App | Purpose |
|-----|---------|
| `mm2las` | Export layers → LAS 1.4 (Point Format 8); supports `--frame map\|enu\|geodetic` |
| `mm2ply` | Export layers → PLY |
| `mm2txt` | Export layers → CSV/TXT |
| `mm2grid` | Generate grid maps from point clouds |
| `mm-filter` | Apply filter pipelines to .mm files |
| `mm-info` | Print .mm file summary |
| `mm-georef` | Inject/extract georeferencing from .mm files |
| `mm-viewer` | GUI viewer for .mm files |
| `sm2mm` | Convert SLAM CSimpleMap → .mm |
| `sm-cli` | Manipulate CSimpleMap files (cut, join, export…) |
| `icp-run` | Run ICP pipeline from CLI |
| `icp-log-viewer` | Debug ICP sessions interactively |
| `kitti2mm` | Convert KITTI .bin → .mm |
| `txt2mm` | Convert TXT/CSV point clouds → .mm |
| `rawlog-filter` | Filter MRPT RawLog files |

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

Key filter categories: decimation, outlier removal, range/ring/intensity gating, deskew, edge/plane extraction, layer management.

---

## Build system

```bash
# ROS 2 workspace build
cd ~/ros2_ws
colcon build --packages-select mp2p_icp

# CMake options of interest
-DMP2PICP_BUILD_TESTING=ON      # unit tests (default ON)
-DMP2PICP_BUILD_APPLICATIONS=ON # CLI apps (default ON)
-DMP2PICP_USE_TBB=ON            # TBB parallelism (auto-detected)
```

SIMD-optimized translation units are compiled separately with `-mavx` / `-msse2`.

---

## Dependencies

| Dependency | Notes |
|-----------|-------|
| MRPT ≥ 2.15.4 | containers, tfest, maps, gui, topography — mandatory |
| TBB | Optional; enables parallel ICP iterations |
| mola_common | CMake scripts; fetched as git submodule |
| mola_imu_preintegration | Optional; advanced deskew in FilterDeskew |

---

## Testing

```bash
colcon test --packages-select mp2p_icp
# or directly:
ctest --test-dir build/mp2p_icp -V
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

---

## Related MOLA packages (same workspace)

- `mola_state_estimation/mola_georeferencing` — `mola-mm-add-geodetic` tool that adds per-point lat/lon/alt double fields to .mm layers (prerequisite for mm2las geodetic fast path)
- `mola_lidar_odometry` — primary consumer of mp2p_icp for real-time SLAM

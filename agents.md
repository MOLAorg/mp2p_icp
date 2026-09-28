# mp2p_icp: Agent Context Guide

Quick-start reference for AI agents and new contributors.

**How to maintain this file:** keep it factual and high level: project layout, core data
structures, invariants that must not be broken, and where to look. Do not document every
feature, and do not write narrative about why or how things evolved over time (that belongs
in commit messages and code comments). Update it only when something listed here changes.

## Project identity

**mp2p_icp** (Multi Primitive-to-Primitive ICP): C++ library and CLI toolkit for point
cloud registration and map building, part of the [MOLA](https://github.com/MOLAorg/mola)
framework. BSD-3-Clause. Version: see `package.xml`.

## Repository layout

Three sibling ROS packages, split so headless consumers don't pull GUI deps:

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

Build via colcon only. Each library exports its own CMake target (e.g.
`mola::mp2p_icp_map`). Headless consumers depend on `mp2p_icp_core`.

## Core data structure: `metric_map_t`

`mp2p_icp_map/include/mp2p_icp/metricmap.h`: named point cloud `layers` (CMetricMap),
`lines`, `planes`, optional `id` / `label` / `metadata` / `georeferencing`
(`geo_coord` + `T_enu_to_map`). Standard layer names: `PT_LAYER_*` constants.

`.mm` files: gzip-compressed MRPT `CSerializable` (`load_from_file()` / `save_to_file()`).

## CLI applications

Argument parsing uses CLI11. Core: `mm2las`, `mm2ply`, `mm2txt`, `mm2grid`, `mm-filter`,
`mm-info`, `mm-georef`, `sm2mm`, `sm-cli`, `icp-run`, `kitti2mm`, `txt2mm`,
`rawlog-filter`. Viz: `mm-viewer`, `icp-log-viewer`.

## GUI apps (mp2p_icp_viz)

Dear ImGui (from the `mrpt_imgui_vendor` ROS package, target `imgui::imgui`). Shared code
in `apps/imgui_app_common/`; 3D views use `mrpt::imgui::CImGuiSceneView`.

**Do not delete as dead code:** the axis-corner mini-viewports in mm-viewer are kept in
sync but invisible, because MRPT 2.x only renders the `"main"` viewport.

## ICP pipeline

```
metric_map_t (local) + metric_map_t (global) + CPose3D (initial guess)
    → Matchers → Solvers → QualityEvaluators → iterate until convergence
```

All components are `Parameterizable`, configured via YAML.

**Determinism is a hard invariant:** offline runs must be byte-reproducible. Never use
`tbb::parallel_reduce`; use `tbb::parallel_deterministic_reduce` with an explicit grain
size. Guarded by `test-mp2p_solver_determinism`.

## Filter pipeline

Filters are chained and applied in-place to `metric_map_t`, configured via YAML
(`filters:` list of `class_name` + `params`). Invariants:

- Voxel keys come from `coord2idx`, which divides by the resolution (never multiply by a
  precomputed reciprocal).
- Never `insertPointFast()` into a layer with registered per-point fields: use
  `insertPointFrom()`, or field vectors fall out of sync with x/y/z.

## Build and test

```bash
cd ~/ros2_ws
colcon build --packages-select mp2p_icp        # everything (metapackage)
colcon test  --packages-select mp2p_icp_core   # gtest, one file per component in tests/
```

LTO is disabled for GCC < 12 (it miscompiles devirtualized calls into MRPT classes).

## Dependencies

MRPT (≥ 2.15.4 core, ≥ 2.15.11 viz), CLI11, TBB (optional), `mola_common` (colcon
workspace), `mola_imu_preintegration` (optional, for `FilterDeskew`).

## Coding conventions

- Public headers in `<module>/include/mp2p_icp/`, implementations in `<module>/src/`
- Register classes with `DEFINE_MRPT_OBJECT` / `IMPLEMENTS_MRPT_OBJECT`
- Numeric parameters a user might write as a formula must use `DECLARE_PARAMETER_*`,
  never `MCP_LOAD_*` (which silently truncates expressions)
- Frame naming: `T_A_to_B` = pose of {B} as seen from {A}
- Always brace `if`/`for`/`while`; one variable per declaration; anonymous namespaces
  instead of `static`; American spelling; no en/em dashes; clang-format-14
- Don't sign commits as an AI agent

## Update rules

- New/updated `mp2p_icp_filters` classes: sync `docs/source/mp2p_icp_filters.rst` and
  `~/code/mp2p-pipeline-editor-sources` if it exists (read its `README.md`)

## MRPT3 to-do

Once ported to MRPT 3, drop the glut/freeglut3-dev dependency and delete this section.

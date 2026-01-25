[![CI ROS](https://github.com/MOLAorg/mp2p_icp/actions/workflows/ros-build.yml/badge.svg)](https://github.com/MOLAorg/mp2p_icp/actions/workflows/ros-build.yml)
[![CI Check clang-format](https://github.com/MOLAorg/mp2p_icp/actions/workflows/check-clang-format.yml/badge.svg)](https://github.com/MOLAorg/mp2p_icp/actions/workflows/check-clang-format.yml)
[![CircleCI](https://img.shields.io/circleci/build/gh/MOLAorg/mp2p_icp/develop.svg)](https://circleci.com/gh/MOLAorg/mp2p_icp) 
[![codecov](https://codecov.io/gh/MOLAorg/mp2p_icp/graph/badge.svg?token=DK35PMKR3T)](https://codecov.io/gh/MOLAorg/mp2p_icp)
[![Docs](https://img.shields.io/badge/docs-latest-brightgreen.svg)](https://docs.mola-slam.org/latest/module-mp2p-icp.html)


| Distro | Build dev | Build releases | Stable version |
| ---    | ---       | ---            | ---         |
| ROS 2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mp2p_icp__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mp2p_icp__ubuntu_jammy_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mp2p_icp__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mp2p_icp__ubuntu_jammy_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Hbin_ujv8_uJv8__mp2p_icp__ubuntu_jammy_arm64__binary/badge/icon)](https://build.ros2.org/job/Hbin_ujv8_uJv8__mp2p_icp__ubuntu_jammy_arm64__binary/) | [![Version](https://img.shields.io/ros/v/humble/mp2p_icp)](https://index.ros.org/?search_packages=true&pkgs=mp2p_icp) |
| ROS 2 Jazzy @ u24.04 | [![Build Status](https://build.ros2.org/job/Jdev__mp2p_icp__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mp2p_icp__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Jbin_uN64__mp2p_icp__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mp2p_icp__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Jbin_unv8_uNv8__mp2p_icp__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Jbin_unv8_uNv8__mp2p_icp__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/jazzy/mp2p_icp)](https://index.ros.org/?search_packages=true&pkgs=mp2p_icp) | 
| ROS 2 Kilted @ u24.04 | [![Build Status](https://build.ros2.org/job/Kdev__mp2p_icp__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Kdev__mp2p_icp__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Kbin_uN64__mp2p_icp__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mp2p_icp__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Kbin_unv8_uNv8__mp2p_icp__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Kbin_unv8_uNv8__mp2p_icp__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/kilted/mp2p_icp)](https://index.ros.org/?search_packages=true&pkgs=mp2p_icp) | 
| ROS 2 Rolling (u24.04) | [![Build Status](https://build.ros2.org/job/Rdev__mp2p_icp__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mp2p_icp__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Rbin_uN64__mp2p_icp__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mp2p_icp__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Rbin_unv8_uNv8__mp2p_icp__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Rbin_unv8_uNv8__mp2p_icp__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/rolling/mp2p_icp)](https://index.ros.org/?search_packages=true&pkgs=mp2p_icp) |

| EOL Distro | Last release |
| ---    | ---       |
| ROS1 Noetic (u20.04) | [![Version](https://img.shields.io/ros/v/noetic/mp2p_icp)](https://index.ros.org/?search_packages=true&pkgs=mp2p_icp) |
| ROS 2 Iron (u22.04) | [![Version](https://img.shields.io/ros/v/iron/mp2p_icp)](https://index.ros.org/?search_packages=true&pkgs=mp2p_icp) |


# mp2p_icp
C++ libraries for multi primitive-to-primitive (MP2P) ICP algorithms and flexible point cloud processing pipelines. `mp2p_icp` is used in the [mola_lidar_odometry](https://github.com/MOLAorg/mola_lidar_odometry) framework:

![mola_lidar_short_demo_stairs](https://github.com/MOLAorg/mp2p_icp/assets/5497818/af5c7250-85bf-4017-a983-4883ac9fb972)


- [Main library documentation and C++ API](https://docs.mola-slam.org/latest/module-mp2p-icp.html)
- License: New BSD 3-Clause (Note that each module of [MOLA](https://github.com/MOLAorg/mola) has its own license)

## Introduction

The project provides these C++ libraries:
 * `mp2p_icp_map`: Provides the [`mp2p_icp::metric_map_t`](https://docs.mola-slam.org/latest/class_mp2p_icp_metric_map_t.html) generic metric map container and related utilities for working with point cloud data structures. Metric map files with extension `*.mm` are serializations of instances of this class.
 * `mp2p_icp`: With ICP algorithms, matchers (point-to-point, point-to-plane, point-to-line, covariance-to-covariance, adaptive), solvers (Horn, OLAE, Gauss-Newton), and quality evaluators. It depends on `mp2p_icp_map`.
 * `mp2p_icp_filters`: With point cloud filtering and manipulation algorithms including decimation, voxelization, statistical outlier removal, edge/plane extraction, and various geometric transformations. It depends on `mp2p_icp_map`.

## Command-Line Applications

The full repository also includes these applications (see [apps/](https://github.com/MOLAorg/mp2p_icp/tree/develop/apps) directory):

### Data Conversion Tools
 * **[kitti2mm](https://docs.mola-slam.org/latest/app_kitti2mm.html)**: Converts KITTI dataset LIDAR binary files (`.bin`) with (X,Y,Z, Intensity) data into mp2p_icp metric map files (`.mm`). Supports custom layer names, numeric IDs, and label strings.
 
 * **[txt2mm](https://docs.mola-slam.org/latest/app_txt2mm.html)**: Converts plain-text point cloud data (TXT/CSV) into metric map (`.mm`) files. Supports multiple formats: XYZ, XYZI, XYZIRT, XYZRGB, with configurable column mapping and layer names.
 
 * **[sm2mm](https://docs.mola-slam.org/latest/app_sm2mm.html)**: Converts a [simple map](https://docs.mrpt.org/reference/latest/class_mrpt_maps_CSimpleMap.html) (`.simplemap`) from a SLAM mapping session into a metric map (`.mm`) using a configurable processing pipeline. Essential for post-processing SLAM outputs into structured metric representations.

### Data Export Tools
 * **[mm2ply](https://docs.mola-slam.org/latest/app_mm2ply.html)**: Exports metric map layers to PLY point cloud files. Supports both ASCII and binary formats, selective field export, and preserves all point attributes (coordinates, colors, intensities, etc.).
 
 * **[mm2txt](https://docs.mola-slam.org/latest/app_mm2txt.html)**: Exports metric map layers as space-delimited CSV/TXT files with header rows. Ideal for data analysis in spreadsheet applications or custom processing pipelines. Supports selective layer and field export.
 
 * **[mm2las](https://docs.mola-slam.org/latest/app_mm2las.html)**: Exports metric maps to industry-standard LAS 1.4 format with Point Format 8 support. Includes automatic color mapping, extra dimensions for custom fields. Compatible with CloudCompare, QGIS, and ArcGIS.

### Map Processing & Analysis Tools
 * **[mm-filter](https://docs.mola-slam.org/latest/app_mm-filter.html)**: Applies mp2p_icp_filters pipelines to metric map files. Can operate in pipeline mode (applying complete YAML-defined filter chains) or rename mode (simply renaming layers). Supports custom plugin loading.
 
 * **[mm-info](https://docs.mola-slam.org/latest/app_mm-info.html)**: Displays a summary of metric map contents including layers, point counts, and metadata.
 
 * **[mm-georef](https://docs.mola-slam.org/latest/app_mm-georef.html)**: Extracts or injects geo-referencing information between metric map files (`.mm`) and standalone georeferencing files (`.georef`).

### Visualization Tools
 * **[mm-viewer](https://docs.mola-slam.org/latest/app_mm-viewer.html)**: GUI application to visualize metric map (`.mm`) files. Supports loading additional 3D scenes and trajectory files in TUM format.
 
 * **[icp-log-viewer](https://docs.mola-slam.org/latest/app_icp-log-viewer.html)**: Interactive GUI for debugging ICP pipelines. Visualizes ICP log files (`.icplog`) with autoplay mode and detailed inspection of registration results. Essential for understanding and optimizing ICP algorithm performance.

### ICP Execution Tools  
 * **[icp-run](https://docs.mola-slam.org/latest/app_icp-run.html)**: Standalone program to execute ICP pipelines from the command line.

### SimpleMaps Manipulation
 * **[sm-cli](https://docs.mola-slam.org/latest/app_sm-cli.html)**: Swiss-army knife for [simple map](https://docs.mrpt.org/reference/latest/class_mrpt_maps_CSimpleMap.html) (`.simplemap`) manipulation. Commands include: `info` (analyze contents), `cut` (extract by keyframe index), `trim` (extract by bounding box), `join` (merge maps), `level` (make horizontal), `tf` (apply SE(3) transform), `export-keyframes` (save trajectories as TUM), and `export-rawlog` (convert to RawLog format).


## Key Features and Components

Key C++ classes provided by this project (see [full documentation](https://docs.mola-slam.org/latest/module-mp2p-icp.html)):

### Core Data Structures
 * [`mp2p_icp::metric_map_t`](https://docs.mola-slam.org/latest/class_mp2p_icp_metric_map_t.html): A generic
   data type to store raw or processed point clouds, including support for multiple layers,
   segmented data, and discrete extracted features.

### ICP Algorithms and Matchers
 * [`mp2p_icp::ICP`](https://docs.mola-slam.org/latest/class_mp2p_icp_I_c_p.html): A uniform API
   for matching generic point clouds with support for:
   - **Point-to-Point** matching (various distance metrics)
   - **Point-to-Plane** matching (planes extracted from point clouds)
   - **Point-to-Line** matching (edges/lines extracted from point clouds)
   - **Covariance-to-Covariance** matching (Gaussian distributions)
   - **Adaptive** matching (automatically selects best matcher)

### Solvers
 * **Horn's method**: Closed-form solution for point-to-point alignment
 * **OLAE** (Optimal Linear Attitude Estimator): For attitude/rotation estimation
 * **Gauss-Newton**: Iterative solver for complex matching scenarios

### Filters and Generators
The `mp2p_icp_filters` library provides extensive filtering capabilities:
 * **Decimation**: `FilterDecimate`, `FilterDecimateVoxels`, `FilterDecimateAdaptive`
 * **Outlier removal**: `FilterSOR` (Statistical Outlier Removal), `FilterVoxelSOR`
 * **Geometric filtering**: `FilterByRange`, `FilterBoundingBox`, `FilterByRing`
 * **Feature extraction**: `FilterEdgesPlanes`, `FilterCurvature`
 * **Data manipulation**: `FilterDeskew`, `FilterMerge`, `FilterAdjustTimestamps`
 * **Generators**: `GeneratorEdgesFromCurvature`, `GeneratorEdgesFromRangeImage`

### Quality Evaluators
 * `QualityEvaluator_PairedRatio`: Evaluates alignment quality based on paired point ratios
 * `QualityEvaluator_Voxels`: Voxel-based quality metrics
 * `QualityEvaluator_RangeImageSimilarity`: Range image comparison metrics

## Multi-Primitive Pairings

The library supports matching between different geometric primitives (points, planes, lines, covariances), allowing for more robust and accurate registration compared to traditional point-to-point ICP. See the [documentation](https://docs.mola-slam.org/latest/module-mp2p-icp.html) for more details on the pairing strategies and matching algorithms.

![mp2p_pairings](docs/source/imgs/mp2p_pairings.png)

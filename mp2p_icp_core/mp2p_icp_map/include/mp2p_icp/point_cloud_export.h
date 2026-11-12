/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   point_cloud_export.h
 * @brief  Writers of point clouds, with all their per-point fields, to PLY, LAS and TXT files
 * @author Jose Luis Blanco Claraco
 * @date   Nov 12, 2026
 */
#pragma once

#include <mp2p_icp/metricmap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/poses/CPose3D.h>

#include <optional>
#include <string>
#include <vector>

namespace mp2p_icp
{
/** \addtogroup mp2p_icp_map_grp
 * @{ */

/** Options for save_point_cloud_to_ply() */
struct ply_export_options_t
{
    /** Binary little endian (true) or ASCII (false) PLY format. */
    bool binary = false;

    /** Fields to export, in this order. If empty, all fields are exported:
     *  x, y, z, then the float, double, uint16 and uint8 fields. The
     *  color_{r,g,b} and color_{rf,gf,bf} fields are written with the standard
     *  PLY names red, green, blue. */
    std::vector<std::string> fields;

    /** If true, requested fields missing in the cloud are written as zeros;
     *  otherwise an exception is thrown. */
    bool ignore_missing_fields = false;

    /** If set, the x, y, z coordinates are transformed with this pose before
     *  being written (e.g. T_enu_to_map, to export in the ENU frame). */
    std::optional<mrpt::poses::CPose3D> transform;
};

/** Writes a point cloud, with all or a subset of its per-point fields, as a PLY
 *  file.
 *
 * \exception std::exception On any error (cannot create the file, missing
 * fields, etc.)
 */
void save_point_cloud_to_ply(
    const mrpt::maps::CPointsMap& pc, const std::string& filename,
    const ply_export_options_t& options = {});

/** Options for save_point_cloud_to_las() */
struct las_export_options_t
{
    /** Fields to export. If empty, all fields are exported. Fields matching
     *  the LAS 1.4 point format 8 standard ones (intensity, return_number,
     *  number_of_returns, classification, red, green, blue, nir, gps_time,
     *  scan_angle, user_data, point_source_id) go into the point records,
     *  the rest as "extra bytes" dimensions. */
    std::vector<std::string> fields;

    /** System identifier and generating software in the LAS header (up to 31
     *  characters each). */
    std::string system_id           = "mp2p_icp";
    std::string generating_software = "MOLA mp2p_icp";

    /** If set, the x, y, z coordinates are transformed with this pose before
     *  being written (e.g. T_enu_to_map, to export in the ENU frame). Ignored
     *  if `geodetic` is true. */
    std::optional<mrpt::poses::CPose3D> transform;

    /** If true, points are written as WGS-84 longitude, latitude, ellipsoidal
     *  height, with the EPSG:4979 CRS embedded in the file. Per-point
     *  "latitude", "longitude", "altitude" double fields are used if present;
     *  otherwise, points are converted with `georeferencing`. */
    bool geodetic = false;

    /** Georeferencing of the cloud, used if `geodetic` is true. */
    std::optional<metric_map_t::Georeferencing> georeferencing;

    /** Print information messages to std::cout. Warnings (e.g. on missing
     *  fields) always go to std::cerr. */
    bool verbose = false;
};

/** Writes a point cloud as a LAS 1.4 file, with point data record format 8.
 *  Empty point clouds are skipped, with a warning, without creating the file.
 *
 * \exception std::exception On any error (cannot create the file, the extent
 * does not fit into the LAS integer coordinates, etc.)
 */
void save_point_cloud_to_las(
    const mrpt::maps::CPointsMap& pc, const std::string& filename,
    const las_export_options_t& options = {});

/** Options for save_point_cloud_to_txt() */
struct txt_export_options_t
{
    /** Write a first line with the space-separated field names. */
    bool header = true;

    /** Fields to export, in this order. If empty, all fields are exported:
     *  x, y, z, then the float, uint16, double and uint8 fields. Requested
     *  fields missing in the cloud are written as zeros. */
    std::vector<std::string> fields;

    /** If set, the x, y, z coordinates are transformed with this pose before
     *  being written (e.g. T_enu_to_map, to export in the ENU frame). */
    std::optional<mrpt::poses::CPose3D> transform;
};

/** Writes a point cloud as a space-separated text file, one point per line.
 *
 * \exception std::exception If the file cannot be created.
 */
void save_point_cloud_to_txt(
    const mrpt::maps::CPointsMap& pc, const std::string& filename,
    const txt_export_options_t& options = {});

/** @} */

}  // namespace mp2p_icp

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
 * @file   sm2mm.h
 * @brief  simplemap-to-metricmap utility function
 * @author Jose Luis Blanco Claraco
 * @date   Dec 18, 2023
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/optional_ref.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>

#include <string>
#include <utility>
#include <vector>

namespace mp2p_icp_filters
{
/** \addtogroup mp2p_icp_filters_grp
 *  @{ */

/// Options for simplemap_to_metricmap()
struct sm2mm_options_t
{
    sm2mm_options_t() = default;

    mrpt::system::VerbosityLevel                  verbosity       = mrpt::system::LVL_INFO;
    bool                                          showProgressBar = false;
    bool                                          throw_on_missing_external_files = true;
    std::vector<std::pair<std::string, double>>   customVariables                 = {};
    std::optional<size_t>                         start_index;
    std::optional<size_t>                         end_index;
    mrpt::optional_ref<mrpt::system::CTimeLogger> profiler;
    std::optional<size_t>                         decimate_every_nth_frame;
    std::optional<size_t>                         decimate_maximum_frame_count;

    /** If provided, robot poses from the simplemap are transformed into the
     *  ENU (East-North-Up) frame before being passed to the generators.
     *  Specifically, the effective pose used for each keyframe becomes:
     *
     *    T_enu_to_map \oplus robotPose
     *
     *  so that the generated metric map is expressed in ENU coordinates.
     *  This is relevant for filters that expect an exact upward direction of
     *  the map +Z axis, for example.
     *
     *  When this field is set, the output metric_map_t will be populated with
     *  a copy of the georeferencing information, **but** with
     *  `T_enu_to_map` reset to the identity pose (while preserving its
     *  covariance matrix), because the map points are already expressed in the
     *  ENU frame and no further rigid-body correction is needed at query time.
     *
     *  The geodetic reference point (`geo_coord`) is copied verbatim from the
     *  input.
     *
     *  \note This option is mutually exclusive with any post-hoc injection of
     *        georeferencing data via `mm-georef --inject-to-map`; use one or
     *        the other.
     */
    std::optional<mp2p_icp::metric_map_t::Georeferencing> georeferencing;
};

/** Utility function to build metric maps ("*.mm") from raw observations
 *  as a simple map ("*.sm"). For a ready-to-use CLI application exposing
 *  this function, as well as documentation on the meaning of each argument,
 *  see
 * [sm2mm](https://github.com/MOLAorg/mp2p_icp/tree/develop/apps/sm2mm).
 *
 * The former contents of outMap are cleared.
 *
 * If `options.georeferencing` is set, robot poses are transformed into the
 * ENU frame (via `T_enu_to_map \oplus robotPose`) before being handed to the
 * generators, and the output map is tagged with the corresponding
 * georeferencing metadata (with `T_enu_to_map` set to identity).
 */
void simplemap_to_metricmap(
    const mrpt::maps::CSimpleMap& sm, mp2p_icp::metric_map_t& outMap,
    const mrpt::containers::yaml& pipeline, const sm2mm_options_t& options = {});

/** @} */

}  // namespace mp2p_icp_filters

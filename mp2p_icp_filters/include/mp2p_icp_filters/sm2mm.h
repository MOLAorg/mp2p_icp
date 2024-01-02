/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   sm2mm.h
 * @brief  simplemap-to-metricmap utility function
 * @author Jose Luis Blanco Claraco
 * @date   Dec 18, 2023
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/system/COutputLogger.h>

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
    sm2mm_options_t()  = default;
    ~sm2mm_options_t() = default;

    mrpt::system::VerbosityLevel verbosity       = mrpt::system::LVL_INFO;
    bool                         showProgressBar = false;
    std::vector<std::pair<std::string, double>> customVariables = {};
    std::optional<size_t>                       start_index;
    std::optional<size_t>                       end_index;
};

/** Utility function to build metric maps ("*.mm") from raw observations
 *  as a simple map ("*.sm"). For a ready-to-use CLI application exposing
 *  this function, as well as documentation on the meaning of each argument,
 *  see
 * [sm2mm](https://github.com/MOLAorg/mp2p_icp/tree/master/apps/sm2mm).
 *
 * The former constents of outMap are cleared.
 *
 */
void simplemap_to_metricmap(
    const mrpt::maps::CSimpleMap& sm, mp2p_icp::metric_map_t& outMap,
    const mrpt::containers::yaml& pipeline,
    const sm2mm_options_t&        options = {});

/** @} */

}  // namespace mp2p_icp_filters

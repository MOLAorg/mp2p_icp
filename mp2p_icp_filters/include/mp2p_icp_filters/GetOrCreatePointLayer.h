/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   GetOrCreatePointLayer.h
 * @brief  Auxiliary function GetOrCreatePointLayer
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mrpt/maps/CPointsMap.h>

#include <string>

namespace mp2p_icp_filters
{
/** \addtogroup mp2p_icp_filters_grp
 *  @{ */

[[nodiscard]] mrpt::maps::CPointsMap::Ptr GetOrCreatePointLayer(
    mp2p_icp::metric_map_t& m, const std::string& layerName, bool allowEmptyName = true,
    const std::string& classForLayerCreation = "mrpt::maps::CSimplePointsMap");

/** @} */

}  // namespace mp2p_icp_filters

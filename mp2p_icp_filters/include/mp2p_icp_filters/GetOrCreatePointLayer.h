/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
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

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
 * @file   pointcloud_sanity_check.h
 * @brief  Checks for consistent length of field vectors.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 11, 2024
 */

//
#include <mrpt/maps/CPointsMap.h>

namespace mp2p_icp
{
/** \addtogroup mp2p_icp_map_grp
 * @{
 */

/** Returns false (and optionally prints a warning to std::cerr) if the point
 * cloud fields are not correctly sized.
 */
[[nodiscard]] bool pointcloud_sanity_check(
    const mrpt::maps::CPointsMap& pc, bool printWarnings = true);

/** @} */

}  // namespace mp2p_icp

/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
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

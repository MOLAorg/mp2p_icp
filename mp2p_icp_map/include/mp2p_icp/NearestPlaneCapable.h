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
 * @file   NearestPlaneCapable.h
 * @brief  Defines a virtual interface for maps capable of finding pt-plane
 * pairings.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 8, 2024
 */
#pragma once

#include <mp2p_icp/point_plane_pair_t.h>

#include <optional>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_map_grp
 * @{ */

/** Virtual interface for nearest plane to a point search algorithms */
class NearestPlaneCapable
{
   public:
    NearestPlaneCapable() = default;
    virtual ~NearestPlaneCapable();

    struct NearestPlaneResult
    {
        NearestPlaneResult() = default;

        /// Found pairing:
        std::optional<point_plane_pair_t> pairing;

        /// Absolute value of plane-point distance, if a pairing is found:
        float distance = 0;
    };

    virtual NearestPlaneResult nn_search_pt2pl(
        const mrpt::math::TPoint3Df& point, const float max_search_distance) const = 0;
};

/** @} */

}  // namespace mp2p_icp

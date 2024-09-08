/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
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

        float                             distance_squared = 0;
        std::optional<point_plane_pair_t> pairing;
    };

    virtual NearestPlaneResult nn_search_pt2pl(
        const mrpt::math::TPoint3Df& point) const = 0;
};

/** @} */

}  // namespace mp2p_icp

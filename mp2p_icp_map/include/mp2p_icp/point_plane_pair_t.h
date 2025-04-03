/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   point_plane_pair_t.h
 * @brief  Defines point_plane_pair_t
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */
#pragma once

#include <mp2p_icp/plane_patch.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/typemeta/TTypeName.h>

#include <vector>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_map_grp
 * @{ */

/** Point-to-plane pair */
struct point_plane_pair_t
{
    plane_patch_t         pl_global;
    mrpt::math::TPoint3Df pt_local;

    point_plane_pair_t() = default;
    point_plane_pair_t(const plane_patch_t& p_global, const mrpt::math::TPoint3Df& p_local)
        : pl_global(p_global), pt_local(p_local)
    {
    }

    DECLARE_TTYPENAME_CLASSNAME(mp2p_icp::point_plane_pair_t)
};
using MatchedPointPlaneList = std::vector<point_plane_pair_t>;

/** @} */

}  // namespace mp2p_icp

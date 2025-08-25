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

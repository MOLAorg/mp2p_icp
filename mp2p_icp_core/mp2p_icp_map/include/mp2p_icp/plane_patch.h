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
 * @file   plane_patch.h
 * @brief  Defines plane_patch_t
 * @author Jose Luis Blanco Claraco
 * @date   Oct 17, 2021
 */
#pragma once

#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPoint3D.h>

#include <cstdint>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_map_grp
 * @{ */

struct plane_patch_t
{
    mrpt::math::TPlane   plane;
    mrpt::math::TPoint3D centroid;

    /** Surface area [m2] the patch was fitted from, and the number of points
     *  in it. Both are zero for patches from a source that does not measure
     *  an extent, and for anything read back from a map serialized before
     *  they existed.
     *
     *  A patch without an extent cannot be weighted against another, which is
     *  what any estimator combining several of them needs: a 3 m2 fragment
     *  and a 50 m2 wall are not equally strong evidence of the same thing.
     */
    double   area       = 0;
    uint32_t num_points = 0;

    plane_patch_t() = default;
    plane_patch_t(const mrpt::math::TPlane3D& pl, const mrpt::math::TPoint3D& center)
        : plane(pl), centroid(center)
    {
    }
    plane_patch_t(
        const mrpt::math::TPlane3D& pl, const mrpt::math::TPoint3D& center, double patchArea,
        uint32_t nPoints)
        : plane(pl), centroid(center), area(patchArea), num_points(nPoints)
    {
    }
};

/** @} */

}  // namespace mp2p_icp
